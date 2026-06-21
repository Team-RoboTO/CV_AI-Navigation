# CV_AI-Navigation

Practical setup notes for the RoboMaster auto-aim pipeline.

## Build

```bash
colcon build --packages-select autoaim --symlink-install
source install/setup.bash
```

The RealSense detector lives in the optional `autoaim_realsense` package. Build
it only on machines that have librealsense2, CUDA, and TensorRT available:

```bash
colcon build --packages-select autoaim autoaim_realsense --symlink-install
```

Use `--symlink-install` during development. Python and launch-file edits apply
after relaunching; C++ edits still require a rebuild.

## Launch

```bash
ros2 launch autoaim standard.launch.py
ros2 launch autoaim hero.launch.py
ros2 launch autoaim sentry.launch.py
```

`standard.launch.py` and `hero.launch.py` default to RealSense.
`sentry.launch.py` defaults to ZED.

Override the camera path when needed:

```bash
ros2 launch autoaim standard.launch.py camera:=zed
ros2 launch autoaim sentry.launch.py camera:=realsense
```

To change a robot's normal camera, edit `DEFAULT_CAMERA` near the top of its
launch file. Use `camera:=...` only for temporary overrides.

The detector-specific YAML files remain in `autoaim/config` and contain
camera/inference settings only.
Robot tuning, such as barrel offsets, fire windows, bullet speed, and EKF gains,
stays in the launch files.

The C++ `serial_bridge` executable is used by default. The Python
`serial_bridge.py` remains installed as a fallback with the same node name,
parameters, topics, and raw packet layout.

`hero.launch.py` and `standard.launch.py` do not start `turret_yaw_mux` or a
navigation pipeline. They pass `/cmd_vel_AI` directly into the serial bridge's
`turret_cmd_topic` and set `enable_nav_pipeline` to `false`, so the nav TX
fields are forced to zero. `sentry.launch.py` keeps the mux/navigation wiring
and sends the mux output through `/turret/cmd`.

## Models

TensorRT engines are not installed by the ROS package. Keep device-specific
engines outside the package tree and point launch files to them with
`engine_path` or `AUTOAIM_ENGINE_PATH`.

```bash
ros2 launch autoaim standard.launch.py \
  engine_path:=/absolute/path/to/yolov26_keypoints.engine
```

TensorRT engines are tied to the JetPack, TensorRT, CUDA, and GPU architecture
that built them. Rebuild on the target Jetson if deserialization fails.

If you only have the `.pt` files, build the TensorRT engines on the target
Jetson. The current launch files use the keypoint model by default:

```bash
cd /workspaces/isaac_ros-dev

python3 -m pip install -U ultralytics onnx onnxslim

yolo export model=AI-models/yolov26_keypoints.pt \
  format=engine imgsz=640 batch=1 dynamic=False half=True nms=True \
  workspace=4 device=0
```

Build the bbox YOLO26 engine only if you are testing a bbox-only detector path;
the ZED and RealSense detector nodes in this repo expect the keypoint engine.

```bash
yolo export model=AI-models/yolo26_bbox.pt \
  format=engine imgsz=640 batch=1 dynamic=False half=True nms=True \
  end2end=False workspace=4 device=0
```

If TensorRT export through Ultralytics fails, use the explicit ONNX plus
`trtexec` path instead:

```bash
cd /workspaces/isaac_ros-dev

yolo export model=AI-models/yolov26_keypoints.pt \
  format=onnx imgsz=640 batch=1 dynamic=False simplify=True nms=True

/usr/src/tensorrt/bin/trtexec \
  --onnx=AI-models/yolov26_keypoints.onnx \
  --saveEngine=AI-models/yolov26_keypoints.engine \
  --fp16 --memPoolSize=workspace:4096

yolo export model=AI-models/yolo26_bbox.pt \
  format=onnx imgsz=640 batch=1 dynamic=False simplify=True nms=True \
  end2end=False

/usr/src/tensorrt/bin/trtexec \
  --onnx=AI-models/yolo26_bbox.onnx \
  --saveEngine=AI-models/yolo26_bbox.engine \
  --fp16 --memPoolSize=workspace:4096
```

The ONNX export is normally reusable across Jetsons if the model, input size,
and export flags are the same. The TensorRT engine is not reusable across all
Jetsons; rebuild it on the target device.

## Git Notes

Useful Git commands from the repo workflow notes:

To see all commits:

```bash
git log --oneline --graph --decorate --all
```

To come back, temporarily, to a commit committed before the latest one:

```bash
git switch --detached <commit>
```

To come to the previous commit and remove the latest if it is not working:

```bash
git revert HEAD
git push origin main
```

## Armor Classes

Current YOLO26 labels:

| ID | Meaning |
|---|---|
| `0` | blue armor |
| `1` | grey armor, ignored by the tracker |
| `2` | red armor |

Set `target_classes` to the enemy color for matches. Use both red and blue only
for mixed testing.

## Useful Parameters

| Parameter | Description |
|---|---|
| `camera` | Active detector override: `realsense` or `zed`. Normal defaults are set by `DEFAULT_CAMERA` near the top of each launch file. |
| `engine_path` | TensorRT engine used by the selected detector. Override this per robot instead of committing generated engines. |
| `serial_port` | Microcontroller device path. Prefer a stable udev symlink over a changing `/dev/ttyACM*` number. |
| `serial_baudrate` | Must match the micro firmware. Default is `500000`. |
| `cmd_timeout` | Safety watchdog: forces shoot to zero when autoaim commands become stale. |
| `use_framed_protocol` | Enables header plus CRC8 packets. Leave false until matching micro firmware is deployed. |
| `target_classes` | Armor class IDs to attack. Use `["0"]` for blue, `["2"]` for red. |
| `min_keypoint_score` | Drops low-confidence keypoints before PnP. Raise for cleaner detections; lower only if recall is too poor. |
| `max_reproj_error` | Rejects PnP solutions with high average reprojection error in pixels. |
| `angle_sync_enable` | Uses interpolated gimbal yaw/pitch at the image capture timestamp. Keep enabled. |
| `use_measured_latency` | Uses measured capture-to-aim latency from message timestamps. Keep enabled for normal operation. |
| `actuation_latency` | Post-command fixed delay only (serial TX + gimbal settle + muzzle exit). Do NOT make it cover capture→aim — that is measured per frame. Horizon = measured + actuation_latency. On a spinner, every 10 ms ≈ 18° of spin at 300 RPM, so keep it tight (~0.02–0.03) and tune in 5 ms steps. |
| `LEGACY_time_bias` | Fixed fallback prediction horizon, used ONLY when `use_measured_latency` is false. Renamed `LEGACY_` because the measured-latency path is the default. |
| `vyaw_conf_p_max` | Spin-confidence gate. Four-face spinning prediction (and any TEMP_LOST coast-fire) turn on only when the vyaw covariance `P(7,7)` is below this. |
| `vyaw_timing_max_reproj` | Anti-spurious-jump gate: a 90° face jump may drive the spin-rate estimator only if PnP reprojection error (px) is below this. Raise if real spins are being ignored. |
| `vyaw_timing_consistency` | Two consecutive same-direction face-jump vyaw estimates must agree within ±this fraction before the spin rate is trusted. Lower = stricter (harder to lock a wrong spin). |
| `ref_freq` | Detector/keypoint rate used by damping math. Set from the measured detector rate. |
| `bullet_speed` | Measured muzzle velocity. Wrong values usually appear as vertical misses. |
| `barrel_offset_x/y/z` | Offset from active camera lens to muzzle in the gimbal body frame. Measure from the active lens. |
| `angular_window` | Fire gate FACING tolerance on the selected plate: `win = angular_window * min(window_ref_dist/range, 1.0)`. The multiplier is capped at 1.0, so the window only SHRINKS far away — it no longer DOUBLES up close (that close-range expansion let the shot fire on a plate ~46° off-facing → "hits the wheels / 45° to the side"). Now that vyaw is gated you can tighten toward 0.25 for tighter groups. |
| `fire_lock_yaw` / `fire_lock_pitch` | LEGACY fixed lock thresholds. The ACTIVE fire-lock is range-scaled (`fire_lock_k_*/range`, clamped to `[fire_lock_min, fire_lock_max_*]`). These remain only as the DEFAULTS for `fire_lock_max_yaw/pitch` and are also read by the viewer HUD, so the names are kept. Set `fire_lock_max_*` directly. |
| `micro_pitch_feedback_opposite_sign` | Whether pitch feedback has the opposite sign from command echo. |
| `micro_pitch_lock_opposite_sign` | Must match `micro_pitch_feedback_opposite_sign`; mismatches cause vertical miss or no-fire behavior. |
| `ego_velocity_available` | Keep false until firmware sends validated chassis velocity. |
| `chassis_heading_index` | Index in micro status containing chassis heading, or `-1` if unavailable. |

## Spinning-Target Tuning

Static aim (a non-rotating enemy) depends only on geometry: bullet speed, barrel
offset, gimbal height, pitch sign. If static hits dead-center, that whole chain
is correct and must NOT be retuned to fix a spinning-target miss.

Missing a spinner (小陀螺) to the side, or hitting the wheels, is a SPIN-PHASE
problem: where will a plate be, and which way will it face, when the bullet
arrives. That depends on three things — the spin rate `vyaw`, the prediction
horizon, and the fire window — and on the PnP yaw, which is the weak, partly
unobservable input. The revision below targets exactly that path. Five changes:

1. **Adaptive `q_pos` removed.** It adapted only the horizontal-center noise,
   but a spinner's center is ~still, so it was reacting to yaw/PnP model error
   (not real maneuvers), cranking `q_pos` up and overshooting the lead. `q_pos`
   and `q_yaw` are now static. (Removed params: `q_adapt_*`.)
2. **Anti-spurious-jump gate on the spin estimator.** vyaw is estimated from the
   time between 90° face jumps. A PnP flip or a bad association produces the same
   "jump" signature; trusting it (old code: 80–100% blend + `P(7,7)→1.0` after
   ONE jump) locked a WRONG spin with high confidence → ~45° side shots. A jump
   now drives the estimator only when PnP is clean (`vyaw_timing_max_reproj`),
   the yaw is real (not faked from bearing), and two consecutive estimates agree
   (`vyaw_timing_consistency`); then it blends gently.
3. **Dual-solution PnP yaw.** A planar plate has two IPPE pose solutions whose
   yaw differs. The detector now passes both; the tracker keeps whichever is
   closer to its predicted yaw. This kills the frame-to-frame yaw flicker that
   polluted vyaw and could masquerade as a face jump. Position is unchanged.
4. **Fire window no longer expands up close.** `win = angular_window *
   min(window_ref_dist/range, 1.0)`. The cap was 2.0, so the facing tolerance
   DOUBLED at close range and the shot could fire on a plate ~46° off-facing.
   Capped at 1.0 it only shrinks far away.
5. **`alpha_yaw = 1.0`** (no spin damping): a top spins at a roughly constant
   rate, so damping `vyaw` between updates only makes the phase prediction lag.

Tuning order for a spinner, after static aim is confirmed:

1. Confirm `ref_freq` equals the real detector rate (`ros2 topic hz
   /detector/armors_keypoints`). Damping and timing math depend on it.
2. Calibrate the horizon: keep `actuation_latency` to the post-command delay
   only (~0.02–0.03 s). Too large leads the shot into the gap between plates.
3. Watch `/debug_state.vyaw_rpm` against the real spin. It should lock within
   ~2 jumps and stay steady. If it jumps around, lower `vyaw_timing_consistency`
   (stricter) or `vyaw_timing_max_reproj` (cleaner PnP only).
4. Only then tighten `angular_window` toward 0.25 for tighter groups.

## Viewer HUD

The viewer publishes `/tracker/debug_image`. The HUD is intentionally small and
shows only values that change during a run:

| Field | Meaning |
|---|---|
| `STATE` | `TRACKING` when a fresh autoaim command is active, `SEARCHING` when no target is active, or `CMD STALE` when the last command is too old to trust. |
| `Pitch tgt` | Absolute pitch command from `/cmd_vel_AI.angular.y`, in radians. |
| `Yaw tgt` | Absolute yaw command from `/cmd_vel_AI.angular.z`, in radians. |
| `Yaw now` | Current yaw feedback from `/micro_status[0]`, in radians. |
| `Pitch raw` | Raw pitch feedback from `/micro_status[1]`, in radians. |
| `Yaw err` | Yaw target minus current yaw, in radians. Green means it is inside `fire_lock_yaw`; orange means it is still outside lock. |
| `Pitch err` | Pitch target minus corrected pitch feedback, in radians. Green means it is inside `fire_lock_pitch`; orange means it is still outside lock. |
| `Lock y/p` | Whether yaw and pitch are both inside their fire-lock thresholds. Shooting is blocked unless both are `Y`. |
| `vx/vy` | Chassis velocity values from `/micro_status[2]` and `/micro_status[3]`, in m/s. |
| `Dist` | Target distance from `/cmd_vel_AI.linear.x`, in meters. |
| `AIM: missing` | No fresh aim marker is available and fallback projection cannot be computed. |
| `AIM src` | Source of the drawn aim marker: `topic` from `/tracker/aim_pixels`, `topic-hold` for a held command, or `cmd-now` fallback projection from current command and micro angles. |
| `>>> FIRE <<<` | Final fire command from `/cmd_vel_AI.angular.x`; shown only when autoaim is requesting shoot. |

The green/orange `FIRE` or `HOLD` marker near the target is the selected impact
point from `/tracker/aim_pixels`, not a separate fire command from the micro.

RealSense-only detector settings live in `config/realsense.yaml`:

| Parameter | Description |
|---|---|
| `serial_no` | Empty string uses the first connected device; set a serial to pin one camera. |
| `width` / `height` / `fps` | Color stream profile. |
| `auto_exposure` / `exposure` / `gain` | Exposure control. Use manual values only after testing on the field. |
| `flip_180` | Rotates image and CameraInfo together for upside-down mounts. |
| `enable_depth` / `enable_infrared` / `enable_imu` | Extra streams. Leave false unless something consumes them. |

ZED-only detector settings live in `config/zed.yaml`:

| Parameter | Description |
|---|---|
| `resolution` / `fps` | ZED capture mode. |
| `image_flip` | Enables the upside-down mount correction. |
| `auto_exposure` / `exposure` / `gain` | ZED exposure control. |
| `auto_white_balance` | ZED white-balance mode. |

## Calibration Order

1. Confirm the selected detector, serial bridge, autoaim node, and viewer all start.
2. Determine the active camera lens by covering one lens and watching the detector feed.
3. Measure barrel offsets from the active lens, not the camera housing center.
4. Measure real bullet speed before tuning vertical aim.
5. Verify pitch feedback sign by comparing pitch feedback with the command echo.
6. Tune `actuation_latency` on a moving target after static aim is correct.

## Serial Notes

Discover connected serial devices:

```bash
ls /dev/tty{ACM,USB}*
udevadm info -a -n /dev/ttyACM0 | grep -E "idVendor|idProduct"
```

If the device exists but cannot be opened during bench testing:

```bash
sudo chmod 666 /dev/ttyACM0
```

Use a udev rule for a permanent stable device name on the robot.

## Troubleshooting

If no commands publish, check detector startup, `target_classes`, camera info,
and PnP rejection logs.

If the tracker detects but never fires, check `fire_lock_yaw`,
`fire_lock_pitch`, `angular_window`, `min_fire_dist`, and the two pitch sign
parameters.

If shots trail a moving target, increase `actuation_latency`, verify `ref_freq`,
and make sure `use_measured_latency` is enabled.

If shots lead too much, reduce `actuation_latency` or `LEGACY_time_bias`.

If static aim is perfect but the robot misses a SPINNING enemy ~45° to the side
or hits the wheels, the spin phase prediction is off, not the geometry. Watch
`/debug_state`: `vyaw_rpm` should match the real spin and stay steady (not jump
around); `faces` should be 4 only when the spin is trusted; `margin` should pass
when a plate is near dead-on. See "Spinning-Target Tuning" below.

If the tracker flickers between tracking and lost states, inspect
`lost_timeout`, `maha_threshold`, `max_match_dist`, and detector keypoint
quality.

If serial cannot connect, check the device path, permissions, baudrate, parity,
and whether another process already has the port open.

If the viewer flickers or old HUD text alternates with new HUD text, check for
duplicate publishers:

```bash
ros2 topic info /tracker/debug_image --verbose
```

There should be exactly one publisher. Duplicate `autoaim_viewer` nodes usually
mean two launch stacks are running at the same time.
