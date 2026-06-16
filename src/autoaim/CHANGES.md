# autoaim — Changes (June 2026 review, v2 — applied to the new package)

Same review as the previous `auto_aim_2` delivery, re-checked against this
package. **All the previously found bugs were still present here**, plus a few
new ones specific to this package. Files touched:

- `src/tracker.cpp` — Mahalanobis gate fix, seed-distance fix, vyaw-timing guard
- `src/autoaim_node.cpp` — image↔gimbal-angle time sync (new), measurement-time
  passed to the tracker
- `zed_detector.py` — capture timestamp, broken IMU publisher, cached CameraInfo
- `serial_bridge.py` — shoot watchdog, stronger sanity checks, optional framed protocol
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
`(now − header.stamp)` at command publish + ~15 ms serial/gimbal = `time_bias`.

## 4. serial_bridge.py

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
4. Measure latency from the capture stamps → set `time_bias`; moving-target test.
5. Spinner test: tighten `angular_window`; check vyaw timing locks in ~2 jumps.
6. Micro firmware: framed protocol + watchdog verification, flip the flag.

---

## 7. Serial bridge ported to C++ (serial_bridge.cpp)

`serial_bridge.py` sat inside the tightest part of the ±17 ms timing budget;
CPython GC pauses and scheduler jitter on a loaded Jetson add unpredictable
milliseconds exactly on the TX path. The C++ port is a drop-in replacement:

- Same node name (`micro_communications_node`), topics, parameter names,
  packet layouts (raw 28/40-byte and framed 31/43-byte modes), CRC8, shoot
  watchdog, sanity checks, auto-reconnect, and /micro_status layout
  (RX 10 floats + TX echo 6 floats). NO firmware change required.
- All three launch files now start `executable="serial_bridge"`. One-line
  revert to the Python version is documented inline in the launch files.
- `serial_bridge.py` is still installed as a fallback.

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

## 8. Package renamed to auto_aim_3 + measured-latency prediction (#1)

### Rename (autoaim → auto_aim_3)
Build with: `colcon build --packages-select auto_aim_3 --symlink-install`

Renamed: package.xml name, CMake project, include/auto_aim_3/, the generated
message namespace (`auto_aim_3::msg::...` in C++, `from auto_aim_3.msg import`
in Python), share-directory lookups, and the launch files
(package="auto_aim_3", executable="auto_aim_3_node").

NOT renamed (on purpose): topic names (/cmd_vel_AI, /detector/*, /micro_status
— micro and tooling contracts unchanged), the internal C++ `namespace autoaim`
and the node names. If any OTHER package subscribes to
/detector/armors_keypoints, it must rebuild against the new message package
name. Remove/unsource the old `autoaim` package from the workspace to avoid
two nodes publishing the same topics.

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
`ros2 launch auto_aim_3 standard.launch.py engine_path:=/path/to/x.engine`

Note: the detector deserializes the engine at startup and aborts if the file
is missing — make sure the path exists on the target before launching.
