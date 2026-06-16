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
| `actuation_latency` | Extra fixed latency for serial TX, gimbal settle, and muzzle exit. Tune in small steps on moving targets. |
| `time_bias` | Fixed fallback prediction horizon used only when measured latency is disabled. |
| `ref_freq` | Detector/keypoint rate used by damping math. Set from the measured detector rate. |
| `bullet_speed` | Measured muzzle velocity. Wrong values usually appear as vertical misses. |
| `barrel_offset_x/y/z` | Offset from active camera lens to muzzle in the gimbal body frame. Measure from the active lens. |
| `angular_window` | Fire gate angle. Large values are useful for tuning; tighten for match timing. |
| `fire_lock_yaw` / `fire_lock_pitch` | Required gimbal-command agreement before firing. Keep larger than command deadbands. |
| `micro_pitch_feedback_opposite_sign` | Whether pitch feedback has the opposite sign from command echo. |
| `micro_pitch_lock_opposite_sign` | Must match `micro_pitch_feedback_opposite_sign`; mismatches cause vertical miss or no-fire behavior. |
| `ego_velocity_available` | Keep false until firmware sends validated chassis velocity. |
| `chassis_heading_index` | Index in micro status containing chassis heading, or `-1` if unavailable. |

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

If shots lead too much, reduce `actuation_latency` or fallback `time_bias`.

If the tracker flickers between tracking and lost states, inspect
`lost_timeout`, `maha_threshold`, `max_match_dist`, and detector keypoint
quality.

If serial cannot connect, check the device path, permissions, baudrate, parity,
and whether another process already has the port open.
