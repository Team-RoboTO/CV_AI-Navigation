# Autoaim update record

Integrated on 2026-06-15 into the existing `src/autoaim` package. The package
name, source path, include path, generated-message namespace, executables, node
names, and runtime topics were deliberately preserved. Active documentation,
runtime entrypoints, timing guidance, serial guidance, and model-path guidance
were aligned with this delivery. Generated build/install artifacts were cleaned
before rebuilding the updated package.

Sections 0-7 retain the review chronology. Sections 8-9 describe the final
measured-latency behavior and external model-path behavior.

## Delivery changes

Same review as the previous `auto_aim_2` delivery, re-checked against this
package. **All the previously found bugs were still present here**, plus a few
new ones specific to this package. Files touched:

- `src/tracker.cpp` — Mahalanobis gate fix, seed-distance fix, vyaw-timing guard
- `src/autoaim_node.cpp` — image↔gimbal-angle time sync (new), measurement-time
  passed to the tracker
- `zed_detector.py` — capture timestamp, broken IMU publisher, cached CameraInfo
- `serial_bridge` — C++ transport with shoot watchdog, stronger sanity checks,
  and optional framed protocol
- `launch/*.launch.py` (all three) — corrected/retuned parameters
- `viewer_node.py` — **was missing from the package** but CMakeLists installs it
  → `colcon build` would have failed. Restored from the old package with imports
  renamed to `autoaim`.

Rebuild: `colcon build --packages-select autoaim --symlink-install`

---

## 0. New findings specific to this package

### ZED X MINI: the half-baseline is 2.5 cm, NOT 6 cm
The detector reveals this is a ZED X Mini (50 mm baseline). My earlier "~6 cm"
figure was for a ZED 2/X — if `barrel_offset_y: -0.06` came from that advice,
it overshoots by ~2× . The active lens is only ~2.5 cm from the housing center.
Re-measure with the lens-cover trick (below).

### `barrel_offset_z: -0.03` has the WRONG SIGN
You measured the camera 3 cm BELOW the barrel → the muzzle is ABOVE the lens →
`barrel_offset_z = +0.03`. The launch had −0.03 (muzzle below lens), a 6 cm
total error in the assumed muzzle height.

### The two pitch flags contradict each other (this can BE the low-shots / no-fire bug)
`micro_pitch_feedback_opposite_sign: True` + `micro_pitch_lock_opposite_sign: False`
is internally inconsistent. Both flags describe the same physical fact (does the
micro report pitch feedback with the opposite sign of its command?). Writing the
loop out: command c = pitch_sign·p, feedback f = k·c (k = firmware truth), the
geometry pitch is pitch_sign·(±f) = (flag-dependent)·k·p — **pitch_sign cancels
(it appears squared)**, so it cannot make the two flags independent. With one
flag right and one wrong, exactly one of these is guaranteed:
- geometry pitch mirrored → systematic vertical miss that grows with pitch
  (shoots under the plate), or
- lock error = 2×command → fire only when the commanded pitch happens to be ~0
  ("fire gate sometimes at 1").

Both flags are now set True (consistent). **Determine the truth empirically**:
`ros2 topic echo /micro_status` — field [1] = pitch feedback, field [11] = pitch
command echo. With the gimbal settled on a target:
feedback ≈ −command → both flags **True**; feedback ≈ +command → both **False**.
Then verify in RViz: tilt the gimbal up → the detected armor's odom z must rise.

### zed_detector: `/zed/imu_data` was publishing garbage
The `get_sensors_data()` call was commented out, so accel/gyro/orientation were
read from an empty `SensorsData`. Restored with silent early-return guards.

### zed_detector: per-frame SDK call
`get_camera_information()` was called every frame at 120 Hz to build CameraInfo.
The message is now built once at init and only re-stamped.

### `time_bias: 0.005` — essentially zero latency compensation
This alone makes every shot trail a moving target. Set to 0.045 as a start;
measure it properly now that stamps are capture-based (see §3).

---

## 1. Calibrate the static aim FIRST (the low/left miss)

Stationary robot, stationary plate at ~3 m, in this order:

**a) Find the active lens.** The ZED is upside down with `FLIP_MODE.ON`
(confirmed in zed_detector.py), which swaps which physical sensor produces the
"left image". Cover one lens with a finger → the one that blacks out the
detector feed is the active one. Measure everything from THAT lens.

**b) Measure the offsets** (gimbal body frame: x fwd, y left, z up):
```
barrel_offset_y = y_muzzle − y_active_lens     (robot-left positive)
barrel_offset_z = z_muzzle − z_active_lens     (camera 3 cm below barrel → +0.03)
```

**c) Measure the real bullet speed.** `bullet_speed: 25.0` is still the
placeholder. At 16 m/s real vs 25 configured, you shoot ~10 cm LOW at 3 m —
under the plate. Chronograph or high-fps video against a ruler.

**d) Resolve the pitch-flag question** (§0) before trusting any vertical tuning.

**e) Bore-sight the residual** with `pitch_offset_deg` / `yaw_offset_deg`:
burst at 3 m, measure the miss, `offset_deg ≈ atan(miss/3)·180/π`, iterate once.

**f) Sanity-check PnP distance** — plate at exactly 2.00 m vs the `rel=` log.
If it reads ~2× too far, the 140×125 mm model in pnp_solver.hpp doesn't match
your keypoint annotation (light-bar endpoints are ~135×56 mm).

---

## 2. tracker.cpp

- **Mahalanobis gate now uses the same obliquity-inflated R as ekfUpdate.**
  This was the TRACKING↔TEMP_LOST flicker → stuttering fire flag on a
  stationary, slightly oblique plate. (Raising `maha_threshold` to 16.9 only
  papered over it — consider lowering back to ~13 after testing.)
- **vyaw-from-jump-timing now only accepts ~90° jumps.** A ~180° wrap (skipped
  face from occlusion/missed frames) would have fed (π/2)/dt = HALF the true
  spin rate into x_(7) with an 80–100% blend.
- Seed-distance vertical term fixed (armor-vs-gimbal height, not above-ground).
- The tracker now receives the MEASUREMENT time (capture stamp) instead of
  `this->now()`, so jump intervals are jitter-free.

## 3. autoaim_node.cpp — image↔gimbal-angle time sync

Ring buffer of time-stamped gimbal angles (filled from /micro_status), with the
camera→odom transform interpolated AT THE IMAGE TIMESTAMP. Removes the
projection error proportional to (gimbal slew rate × pipeline latency) that
made tracking feel slow/laggy and biased aim while moving. New param:
`angle_sync_enable` (True). Works together with the detector stamp fix:
`zed_detector.py` now stamps with `sl.TIME_REFERENCE.IMAGE` (capture time).

With capture stamps you can finally MEASURE the pipeline latency:
The final implementation feeds `(now − header.stamp)` to the tracker and adds
the separately calibrated `actuation_latency`; see section 8.

## 4. Serial bridge safety and protocol

Shoot watchdog (`cmd_timeout: 0.3` — forces shoot=0 if the autoaim node dies;
previously the last shoot=1 was re-sent at 100 Hz forever), all-field sanity
checks (NaN/inf, |pitch| < 2 rad), and the optional framed protocol
(`use_framed_protocol: False` until the micro firmware is updated):

```
TX (Jetson→micro):  0xA5 0x5A | 28-byte payload (7 floats LE) | CRC8(payload)
RX (micro→Jetson):  0x5A 0xA5 | 40-byte payload (10 floats LE) | CRC8(payload)
```

CRC8 (poly 0x07, init 0x00) for the micro:

```c
uint8_t crc8(const uint8_t *data, uint16_t len) {
    uint8_t crc = 0x00;
    while (len--) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
}
```

## 5. Launch parameter changes (all three launch files)

| Parameter | Old | New | Why |
|---|---|---|---|
| `barrel_offset_y` | -0.06 | 0.0 ← **measure** | ZED X Mini lens is 2.5 cm from center, not 6; measure from the active lens |
| `barrel_offset_z` | -0.03 | +0.03 | camera 3 cm BELOW barrel → positive |
| `time_bias` | 0.005 | 0.045 | was ~zero latency compensation; measure properly now |
| `min_keypoint_score` | 0.0 | 0.3 | garbage keypoints → jitter → lock dropouts |
| `q_pos` / `q_yaw` | 30 / 40 | 10 / 20 | likely inflated to fight the sync bug; high q = jittery commands |
| `alpha_pos` | 0.98 | 0.995 | velocity decay was under-leading movers |
| `micro_pitch_lock_opposite_sign` | False | True | must equal the feedback flag — see §0 and run the test |
| `angle_sync_enable` | — | True | new |
| serial `cmd_timeout` | — | 0.3 | new, shoot watchdog |
| serial `use_framed_protocol` | — | False | new, enable after firmware update |
| viewer pitch flag | False | True | must match the node's lock convention |
| `ref_freq` | 60 | 60 + comment | verify with `ros2 topic hz` (ZED grabs at 120) |
| `angular_window` | 1.0 | 1.0 + comment | tighten to 0.10–0.18 @ ref 3.0 for matches |

## 6. Test order

1. Build (viewer restored — it would not have built before), bench-test
   stationary: steady fire flag, no "header.stamp is zero" warning.
2. Run the pitch-flag test (§0) and set both flags accordingly.
3. §1: lens, offsets, bullet speed, bore-sight → expect near-100% stationary.
4. Verify measured capture latency, then tune `actuation_latency` on a moving target.
5. Spinner test: tighten `angular_window`; check vyaw timing locks in ~2 jumps.
6. Micro firmware: framed protocol + watchdog verification, flip the flag.

---

## 7. Serial bridge ported to C++ (serial_bridge.cpp)

The previous Python serial bridge sat inside the tightest part of the ±17 ms timing budget;
CPython GC pauses and scheduler jitter on a loaded Jetson add unpredictable
milliseconds exactly on the TX path. The C++ port is a drop-in replacement:

- Same node name (`micro_communications_node`), topics, parameter names,
  packet layouts (raw 28/40-byte and framed 31/43-byte modes), CRC8, shoot
  watchdog, sanity checks, auto-reconnect, and /micro_status layout
  (RX 10 floats + TX echo 6 floats). NO firmware change required.
- All three launch files start `executable="serial_bridge"`.
- The Python serial bridge was removed; C++ is the only installed transport.

New OPT-IN parameters (defaults preserve old behavior):

| Param | Default | Meaning |
|---|---|---|
| `serial_parity` | "even" | pyserial used PARITY_EVEN; set "none" for 8N1 |
| `low_latency`   | False  | ASYNC_LOW_LATENCY ioctl (FTDI-style adapters) |
| `thread_priority` | 0    | >0 = SCHED_FIFO priority for the bridge thread |

To use `thread_priority` without root:
`sudo setcap cap_sys_nice+ep install/autoaim/lib/autoaim/serial_bridge`
or add an rtprio limit in /etc/security/limits.d/.

Build: `colcon build --packages-select autoaim --symlink-install`
(compile-tested syntax only off-robot — do one bench TX/RX check before a match).

---

## 8. Existing autoaim naming retained + measured-latency prediction (#1)

Build with: `colcon build --packages-select autoaim --symlink-install`

Preserved: package.xml name, CMake project, `include/autoaim/`, generated
message namespace (`autoaim::msg::...` in C++, `from autoaim.msg import` in
Python), share-directory lookups, launch package references, `autoaim_node`,
internal C++ `namespace autoaim`, node names, and all runtime topics.

### #1 — time_bias split into measured + constant
- The node now measures the true pipeline latency per frame:
  `(now − capture stamp)` at command time, passed to the tracker via
  `setPipelineLatency()` (clamped 0–250 ms — a missing stamp can't explode the
  horizon).
- New params (all launch files): `use_measured_latency: True`,
  `actuation_latency: 0.020` (serial TX + gimbal settle + muzzle exit — the
  only part you still calibrate by hand, in 5 ms steps on a moving target).
- `time_bias` is now ONLY the fallback when `use_measured_latency: False`,
  so you can A/B old-vs-new behavior with one launch flag.
- The node logs `pipeline latency (capture->aim) = X ms` every 2 s — verify it
  sits in the 15–40 ms range; if it reads ~0 or >100 ms the detector stamp or
  clock source is wrong.

NOT changed in this delivery: the fire window. It is still ONLY
distance-scaled (`win = angular_window * min(window_ref_dist/range, 2)`), as
before. The vyaw-adaptive tightening for spinners is proposal #2, pending
approval.

---

## 9. TensorRT engine moved OUTSIDE the package

The model is no longer expected inside the package share. Default path is now:

    /workspaces/isaac_ros-dev/AI-models/yolov26_keypoints.engine

Changed:
- All three launch files: `ENGINE_DEFAULT` constant = the absolute path above;
  the old `models/jetson16|jetson64/...` share-dir lookup and the
  `find_models_dir`/`MODEL_FILE` helpers were removed.
- `zed_detector.py`: `default_engine_path()` returns the same absolute path
  (its `find_models_dir` helper removed).
- `CMakeLists.txt`: the `install(DIRECTORY models/ ...)` block was removed, so
  colcon no longer copies the engine into install/.

Resolution order (highest first), unchanged in spirit:
1. launch arg  `engine_path:=/some/other.engine`
2. env var     `AUTOAIM_ENGINE_PATH=/some/other.engine`
3. `ENGINE_DEFAULT` (the absolute path above)

So a per-Jetson or per-robot engine still overrides without editing code, e.g.:
`ros2 launch autoaim standard.launch.py engine_path:=/path/to/x.engine`

Note: the detector deserializes the engine at startup and aborts if the file
is missing — make sure the path exists on the target before launching.

---

## 10. RealSense C++ detector path + ZED config refactor (2026-06-15)

### Goal
Add a high-performance C++ RealSense detector for the competition while keeping
the ZED path working, selectable at launch (`camera:=realsense|zed`). The
autoaim/tracker/serial/viewer nodes are **unchanged** and never learn which
camera is active — both detectors publish the identical topics/messages
(`/detector/armors_keypoints`, `/detector/armors`, `/detector/armors_keypoints_json`,
`/camera_info`, `/yolo/debug_image`) with the same class-id convention
(0 blue, 1 grey, 2 red) and keypoint order (TL,TR,BR,BL).

### New RealSense detector — `src/realsense_detector.cpp` (+ `src/realsense_preprocess.cu`, `include/autoaim/realsense_preprocess.h`)
- **librealsense2 used directly** (no `realsense2_camera` node, no image-topic
  round trip). **Color stream only** by default — this is the competition path,
  so the extra latency/copies of a ROS image hop are avoided.
- **Reuses the existing pipeline logic.** The CUDA letterbox/normalize kernel is
  the same math as the one baked into `zed_detector.py` (only the source stride
  differs: BGR8 vs BGRA), and the YOLO-pose decode (raw `[1,C,A]`/`[1,A,C]` and
  post-NMS `[1,max_det,6+K*3]` layouts, auto-detected from the engine) is a
  faithful C++ port of the Python `_parse_output_shape`/`_decode_*` functions.
  Raw-mode NMS reuses `cv::dnn::NMSBoxes`.
- **CameraInfo from the live color intrinsics** (`rs2_intrinsics`: fx/fy/ppx/ppy,
  5 distortion coeffs, model→`plumb_bob`/`equidistant`). Cached once; intrinsics
  do not change at runtime.
- **No per-frame allocations / minimal CPU copies.** Device input/output/color
  buffers, a pinned host output buffer, the CUDA stream, and the decode scratch
  vectors are all preallocated; tensor addresses are set once. Per frame: one
  H2D color copy, kernel, `enqueueV3`, one D2H copy, decode, publish.
- **Debug image throttled** (`publish_debug_every`, default every 4 frames), not
  full rate. CameraInfo also throttled (`camera_info_every`).
- **Capture-time stamping** like the ZED path: uses the RealSense frame
  timestamp when the domain is host-epoch aligned (SYSTEM/GLOBAL), else `now()`,
  so the autoaim image↔gimbal-angle sync (§3) keeps working.
- TensorRT 10 API (`enqueueV3`/`setTensorAddress`); `initLibNvInferPlugins` is
  called so end2end/NMS-plugin engines deserialize.

### ZED detector refactor — `zed_detector.py`
All camera/runtime values that were **hard-coded constants** are now ROS
parameters (declared with the old values as defaults, so behavior is unchanged
unless overridden): `resolution`, `fps`, `image_flip`, `auto_exposure`,
`exposure`, `gain`, `auto_white_balance`, `threshold`, `nms_iou`,
`publish_debug_every`, `camera_info_every`, `frame_id`, `imu_frame_id`,
`engine_path`. **Why:** these are competition-tuning values; baking them into
source forced a rebuild/edit to retune. The native left-image size now comes
from a `resolution`-name table (SVGA→960×600 default) instead of the
`NATIVE_W/NATIVE_H` constants, so it tracks the selected resolution.
`/camera_info` is now throttled (`camera_info_every`, default 1 = unchanged).
The model/dataset invariants (class ids, keypoint order, decode array sizing)
are intentionally **kept** as constants — they are not field-tunable.

### Config files (new)
- `config/sensors/zed.yaml` and `config/sensors/realsense.yaml` — one file per
  camera, same parameter style. **Camera/detector params only.** Robot
  calibration (barrel offsets, gimbal signs, bullet speed, fire window, EKF
  gains) stays in the autoaim-node params inside the launch files — it was never
  moved here, to keep calibration separate from camera config.
- ROS YAML uses the `/**` node wildcard so the file applies regardless of the
  detector's remapped node name.

### Launch (all three profiles)
- New arg `camera` (default **`realsense`** for standard/hero/sentry). `camera:=zed`
  restores the ZED node. Exactly one detector is started via `IfCondition`.
- The matching `config/sensors/*.yaml` is passed to the chosen detector; the
  `engine_path` launch arg/env still overrides the YAML value (resolution order
  unchanged — see §9).
- The old inline `detector_params` dict was removed (its values live in YAML now).

### Build — `CMakeLists.txt` / `package.xml`
- `find_package(realsense2)`, `find_package(CUDAToolkit)`, manual TensorRT
  find (`NvInfer.h` + `libnvinfer`/`libnvinfer_plugin`, no CMake config on Jetson).
- The CUDA kernel is built as an **isolated static lib** (`realsense_preprocess`)
  so `nvcc` never sees ROS compile flags; the node itself is plain C++ linking
  only the CUDA runtime + TensorRT + realsense2 + OpenCV + the autoaim message
  typesupport. `CMAKE_CUDA_ARCHITECTURES` defaults to `72;87` (Xavier/Orin),
  overridable with `-DCMAKE_CUDA_ARCHITECTURES=`.
- Installs the `realsense_detector` executable and `config/` into the package
  share. `package.xml` gains `<depend>librealsense2</depend>`.

### Verification done on this machine
Built clean (`colcon build --packages-select autoaim --symlink-install`). With a
RealSense attached, the node opened the color stream, read 640×480 intrinsics,
and built CameraInfo; it then failed **only** at engine deserialize because the
on-disk engine was built with a newer TensorRT than the device's 10.7 (same
constraint the ZED path has — see §9 / models.md). Full inference + topic-rate
checks require an engine built with the target's TensorRT version.

### Fix: Python entrypoints needed the execute bit
`ros2 launch ... hero.launch.py` failed with `executable 'viewer_node.py' not
found on the libexec directory`. Cause: `viewer_node.py` and `zed_detector.py`
were committed `0644` (no execute bit). With `colcon build --symlink-install`,
the libexec entry is a symlink to the **source** file, and launch_ros only
accepts an executable file — so the non-`+x` source made both Python nodes
"not found". Fixed with `chmod +x src/autoaim/{viewer_node.py,zed_detector.py}`
(symlink reflects it immediately; no rebuild). `install(PROGRAMS ...)` already
sets `+x` on non-symlink installs, so this only bites in `--symlink-install`
mode — keep the source files executable.

### Fix: RealSense `exposure`/`gain` param type
`realsense_detector` aborted at startup with `parameter 'exposure' has invalid
type: ... {double} ... setting it to {integer} is not allowed`. The node
declared `exposure`/`gain` as `double` but `realsense.yaml` writes them as
integers (`6000`, `64`), and ROS refuses to load an int onto a double param.
These are integer-valued RealSense options, so they are now declared `int`
(cast to float at `set_option`). The natural YAML form `exposure: 6000` loads
correctly; rebuild required.

### Fix: RealSense default exposure overexposed (white image)
`/yolo/debug_image` came through as a valid `bgr8` 640×480 frame (correct stride
and length) but ~70% of pixels were saturated (mean 232/255) → pure white in
RViz. Cause: the default `auto_exposure: false` + fixed `exposure: 6000` (6 ms)
overexposes a normally-lit room. Default changed to `auto_exposure: true` so the
camera adapts and produces a usable image out of the box; the manual
`exposure`/`gain` remain for arena tuning (lowered the manual fallback to 1500
us). **Why:** white image was a config default, not a data-path bug — the
detector→viewer→RViz image chain is correct. Note: the low framerate seen
alongside this is unrelated — it is the `net=960` engine running on a Jetson it
was not built for (TRT cross-device tactic penalty), not the RealSense code.
