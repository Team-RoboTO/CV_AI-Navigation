# CV_AI-Navigation

Practical setup and tuning notes for the RoboMaster/RoboTO auto-aim pipeline.

This document explains where each tunable parameter lives, what code path it affects,
and what usually happens when the value is increased, decreased, enabled, or disabled.
Use it as the first reference before changing launch files or camera YAML files.

---

## Build

```bash
colcon build --packages-select autoaim --symlink-install
source install/setup.bash
```

The RealSense detector lives in the optional `autoaim_realsense` package. Build it only
on machines that have librealsense2, CUDA, and TensorRT available:

```bash
colcon build --packages-select autoaim autoaim_realsense --symlink-install
```

Use `--symlink-install` during development. Python and launch-file edits apply after
relaunching; C++ edits still require a rebuild.

---

## Launch

```bash
ros2 launch autoaim standard.launch.py
ros2 launch autoaim hero.launch.py
ros2 launch autoaim sentry.launch.py
```

Current defaults:

| Launch file | Default camera | Turret mux | Serial turret input | Navigation TX |
|---|---:|---:|---|---:|
| `standard.launch.py` | RealSense | no | `/cmd_vel_AI` | disabled |
| `hero.launch.py` | RealSense | no | `/cmd_vel_AI` | disabled |
| `sentry.launch.py` | ZED | yes | `/turret/cmd` | enabled |

Override the camera path when needed:

```bash
ros2 launch autoaim standard.launch.py camera:=zed
ros2 launch autoaim sentry.launch.py camera:=realsense
```

To change a robot's normal camera, edit `DEFAULT_CAMERA` near the top of its launch file.
Use `camera:=...` only for temporary overrides.

The detector-specific YAML files live in `autoaim/config` and contain camera/inference
settings only. Robot tuning, such as barrel offsets, fire windows, bullet speed,
latency compensation, gimbal signs, and EKF gains, stays in the launch files.

The C++ `serial_bridge` executable is used by default. The Python `serial_bridge.py`
remains installed as a fallback with the same node name, parameters, topics, and raw
packet layout.

---

## Models

TensorRT engines are not installed by the ROS package. Keep device-specific engines
outside the package tree and point launch files to them with `engine_path` or
`AUTOAIM_ENGINE_PATH`.

```bash
ros2 launch autoaim standard.launch.py \
  engine_path:=/absolute/path/to/yolov26_keypoints.engine
```

TensorRT engines are tied to the JetPack, TensorRT, CUDA, and GPU architecture that
built them. Rebuild on the target Jetson if deserialization fails.

If you only have the `.pt` files, build the TensorRT engines on the target Jetson:

```bash
cd /workspaces/isaac_ros-dev

python3 -m pip install -U ultralytics onnx onnxslim

yolo export model=AI-models/yolov26_keypoints.pt \
  format=engine imgsz=640 batch=1 dynamic=False half=True nms=True \
  workspace=4 device=0
```

If TensorRT export through Ultralytics fails, use the explicit ONNX plus `trtexec` path:

```bash
cd /workspaces/isaac_ros-dev

yolo export model=AI-models/yolov26_keypoints.pt \
  format=onnx imgsz=640 batch=1 dynamic=False simplify=True nms=True

/usr/src/tensorrt/bin/trtexec \
  --onnx=AI-models/yolov26_keypoints.onnx \
  --saveEngine=AI-models/yolov26_keypoints.engine \
  --fp16 --memPoolSize=workspace:4096
```

Build the bbox YOLO26 engine only if you are explicitly testing a bbox-only detector path.
The ZED and RealSense detector nodes in this repo expect the keypoint engine.

---

## Armor classes

Current YOLO26 labels:

| ID | Meaning |
|---|---|
| `0` | blue armor |
| `1` | grey armor, ignored by the tracker |
| `2` | red armor |

For matches, attack only the enemy color. Use both red and blue only for mixed lab testing.

---

# Parameter Reference

## 1. Launch arguments

These are passed from the command line to launch files.

| Parameter | Current/default | Where it impacts code | Raise/lower/change effect |
|---|---:|---|---|
| `camera` | `realsense` on standard/hero, `zed` on sentry | Launch condition that starts either `realsense_detector` or `zed_detector.py`. | `camera:=realsense` starts the RealSense path; `camera:=zed` starts the ZED path. Autoaim should not need code changes because both detectors publish the same topics. Re-measure barrel offsets when the active camera changes. |
| `engine_path` | `/workspaces/isaac_ros-dev/AI-models/yolov26_keypoints.engine` | Detector TensorRT deserialization. | Wrong path or wrong engine build causes detector startup failure. Use per-robot override instead of committing engines. |
| `serial_port` | `/dev/ttyACM0` | `serial_bridge` port open. | Wrong path means no micro communication. Prefer a udev symlink such as `/dev/robot_micro` over a changing `/dev/ttyACM*`. |

---

## 2. Camera/inference YAML parameters

Camera YAML files affect detection quality, image timing, CameraInfo, and inference load.
They do not tune the ballistic solver, EKF, barrel offsets, or fire gates.

### Shared detector/inference parameters

| Parameter | Applies to | Effect | If increased/enabled | If decreased/disabled |
|---|---|---|---|---|
| `engine_path` | ZED + RealSense | TensorRT engine loaded by the detector. | Not a numeric tuning parameter. Must match model layout, JetPack/TensorRT/GPU. | Detector fails or decodes garbage if incompatible. |
| `threshold` | ZED + RealSense | YOLO detection confidence threshold. | Fewer false positives and cleaner PnP, but more missed distant/blurred armors. | More recall, but more false positives, PnP jitter, and fire-lock dropouts. |
| `nms_iou` | ZED + RealSense | Non-maximum suppression IoU for raw-output engines. Post-NMS engines mostly skip it. | More overlapping detections survive. Can duplicate the same armor. | More aggressive suppression. Can delete close/overlapping armors. |
| `publish_debug_every` | ZED + RealSense | Publishes `/yolo/debug_image` every N frames. | Higher number reduces ROS/CPU overhead but gives less visual feedback. | Lower number gives smoother debug view but increases overhead. `0` disables debug image. |
| `camera_info_every` | ZED + RealSense | Publishes `/camera_info` every N frames. | Higher number reduces repeated CameraInfo traffic. | Too low is fine; `0` can prevent PnP initialization if autoaim never receives intrinsics. |
| `frame_id` | ZED + RealSense | Header frame for camera image/CameraInfo. | Use only if the rest of the system expects the same frame name. | Wrong frame names confuse debugging/TF conventions. |

### ZED-specific parameters: `config/zed.yaml`

| Parameter | Current value | Effect | If increased/enabled | If decreased/disabled |
|---|---:|---|---|---|
| `debug_scores` | `true` | Draws/logs detection scores in debug output. | More useful debug visualization. | Cleaner debug output, slightly less overlay work. |
| `resolution` | `SVGA` | ZED left-image resolution used for inference geometry and CameraInfo. | Higher resolution improves keypoint precision at distance but increases latency/load. | Lower resolution reduces latency/load but worsens long-range PnP. |
| `fps` | `120` | ZED capture FPS. | Higher FPS helps spinner tracking only if inference keeps up. | Lower FPS reduces load but worsens face-jump timing and target motion tracking. |
| `image_flip` | `true` | Corrects upside-down camera mount with ZED SDK flip. | Correct when camera is mounted upside-down. Also affects which physical sensor becomes the active left image. | Wrong value flips keypoint geometry and can corrupt PnP/yaw. |
| `auto_exposure` | `false` | Enables/disables ZED AEC/AGC. | Adaptive brightness, useful when lighting changes, but frame-to-frame exposure variation can destabilize detections. | Manual exposure/gain gives stable images, but must be tuned on the real field. |
| `exposure` | `60` | Manual ZED exposure setting, used only when `auto_exposure=false`. | Brighter image, but more motion blur. | Darker image, less motion blur. Too low loses detections. |
| `gain` | `50` | Manual ZED gain. | Brighter image, but more noise. | Cleaner image, but darker. |
| `auto_white_balance` | `true` | ZED white balance mode. | Adapts to lighting, but color can drift. | More stable color if manually configured, useful if red/blue classification is unstable. |
| `imu_frame_id` | `zed_imu_link` | Header frame for ZED IMU message. | Use only if downstream expects it. | Wrong frame is mostly a debug/TF issue unless downstream consumes ZED IMU. |

### RealSense-specific parameters: `config/realsense.yaml`

| Parameter | Current value | Effect | If increased/enabled | If decreased/disabled |
|---|---:|---|---|---|
| `serial_no` | `""` | Selects a specific RealSense device. Empty string uses first connected device. | Set a real serial number when multiple RealSense cameras are connected. | Empty is simpler but can open the wrong camera if more than one exists. |
| `width` / `height` | `640` / `480` | Color stream resolution and CameraInfo. | Higher resolution can improve PnP but increases latency/load. | Lower resolution reduces load but hurts distance/keypoint precision. |
| `fps` | `60` | Color stream FPS. | Higher FPS helps latency and spinner timing if the camera supports it. | Lower FPS reduces load but worsens tracking of moving/spinning targets. |
| `auto_exposure` | `false` | Enables RealSense auto exposure. | Adaptive image, but can change brightness and effective exposure during a fight. | Manual control gives consistent frames but must be field-tuned. |
| `exposure` | `60` | Manual RealSense exposure in microseconds. | Brighter image, but more blur. Be careful: very high values overexpose normal rooms. | Darker image, less blur. Too low loses detections. |
| `gain` | `50` | Manual RealSense gain. | Brighter but noisier. | Cleaner but darker. |
| `pixel_format` | `yuyv` | Camera color stream format. | `yuyv` uses raw YUYV and GPU conversion; saves CPU, but verify color consistency. | `bgr8` lets librealsense convert on CPU; safer color match, more CPU work. |
| `enable_depth` | `false` | Opens depth stream. | More USB bandwidth/power; no current downstream benefit. | Correct default for RGB-only detector. |
| `enable_infrared` | `false` | Opens raw IR streams. | More bandwidth/power; no current downstream benefit. | Correct default. |
| `enable_imu` | `false` | Opens accel/gyro streams. | More sensors open, but not consumed by current autoaim path. | Correct default unless a downstream node needs IMU. |
| `flip_180` | `true` | Rotates color image, debug image, and CameraInfo together. | Correct for upside-down mount. | Wrong value mirrors/rotates the target geometry and breaks aiming. |
| `frame_id` | `camera_color_optical_frame` | CameraInfo/image frame name. | Keep consistent with the rest of the pipeline. | Wrong frame mostly affects visualization/TF conventions. |

---

## 3. Autoaim/tracker launch parameters

These parameters are consumed by `autoaim_node.cpp` and `tracker.cpp`. They are the main
robot-specific tuning surface.

### Target color and detector input

| Parameter | Current values | Effect | If changed |
|---|---:|---|---|
| `target_classes_from_micro_status` | standard/hero `false`, sentry `true` | If true, autoaim reads team/color from `/micro_status[target_color_status_index]` and selects the enemy class automatically. | If the micro color field is wrong, the robot shoots the wrong color or tracks nothing. If false, `target_classes` is used directly. |
| `target_classes` | `["0"]` | Fixed YOLO class list to attack when auto color is disabled. | Use `["0"]` for blue, `["2"]` for red. Do not use `["0", "2"]` in a real match unless intentionally testing mixed targets. |
| `target_color_status_index` | `4` | Index in `/micro_status` containing the micro color/team field. | Wrong index breaks automatic enemy selection. |
| `micro_color_target_classes` | `["0", "2"]` | Mapping from micro color to YOLO class to shoot. Index `0` means we are red -> shoot blue; index `1` means we are blue -> shoot red. | Reversing this mapping makes the robot shoot friendly armor. |
| `use_keypoints` | `true` | Uses `/detector/armors_keypoints` and PnP from 4 armor corners. | Keep true for the YOLO-pose model. False uses the older bbox path and is less accurate. |
| `keypoint_topic` | `/detector/armors_keypoints` | Topic consumed by autoaim in keypoint mode. | Wrong topic means no target updates. |
| `min_keypoint_score` | standard/hero `0.30`, sentry `0.15` | Drops low-confidence keypoints before PnP. | Higher = cleaner PnP but fewer detections. Lower = more recall but more jitter and false PnP. |
| `max_reproj_error` | `25.0` | Rejects PnP if average reprojection error is too high in pixels. | Higher accepts noisy poses; lower rejects more bad poses but can cause tracking dropouts. |
| `light_ratio` | `0.85` | Used mainly by legacy bbox PnP reconstruction. | Mostly irrelevant while `use_keypoints=true`. Do not tune first. |

### Camera-to-world geometry and filtering

| Parameter | Current value | Effect | If increased | If decreased |
|---|---:|---|---|---|
| `angle_sync_enable` | `true` | Uses gimbal yaw/pitch at the image capture timestamp, not the latest feedback. | Keep enabled. Disabling creates apparent tracker lag when the gimbal is moving. | Do not disable unless debugging timestamps. |
| `max_armor_distance` | `6.0` | Rejects detections farther than this range from the camera. | Allows farther shots but accepts noisier PnP. | Safer/cleaner but may refuse long-range targets. |
| `max_armor_z` | `4.0` | Rejects implausible armor height in odom/world frame. | More tolerant of transform errors but accepts bad PnP. | Stricter, but may reject valid targets if geometry is miscalibrated. |
| `confirm_frames` | `2` | Frames needed before DETECTING becomes TRACKING. | More stable lock, slower shooting. | Faster lock, more false locks. |
| `lost_timeout` | `0.50` | Time allowed without a valid match before target is considered lost. | More tolerant of occlusion/blur, but can chase a ghost target. | Faster reset, but more tracking flicker. |

### EKF process noise and measurement noise

The tracker state is:

```text
[xc, vxc, yc, vyc, za, vza, yaw, vyaw, r]
```

Position/yaw prediction uses a damped constant-velocity model. Measurement update fuses:

```text
z = [armor_x, armor_y, armor_z, armor_yaw]
```

with dynamic measurement noise that grows with distance and obliquity.

| Parameter | Current value | Effect | If increased | If decreased |
|---|---:|---|---|---|
| `q_pos` | `100.0` | EKF process noise for target translation/height acceleration. | More reactive to target motion; more jitter if PnP is noisy. | Smoother but more lag/under-lead on moving targets. |
| `q_yaw` | standard/hero `20.0`, sentry `10.0` | EKF process noise for target yaw/spin. | Tracks spin/yaw changes faster; noisier yaw. | Smoother yaw but slower spinner adaptation. |
| `q_r` | `1e-6` | Process noise for estimated armor radius. | Radius adapts more easily but can chase noise. | Radius stays nearly fixed. Usually safer. |
| `r_pos_base` | `0.05` | Base position measurement noise. | EKF trusts PnP less; smoother but laggier. | EKF follows PnP more; more jitter. |
| `r_pos_slope` | `0.04` | Extra position measurement noise per meter of range. | Far targets are trusted less; safer long-range behavior. | Far PnP is trusted more; can improve response but increase misses. |
| `r_yaw_base` | `0.05` | Base yaw measurement noise. | Trusts armor yaw less; stable but less informative. | Trusts yaw PnP more; can help spin estimation but noisy at oblique angles. |
| `r_yaw_slope` | `0.005` | Extra yaw measurement noise per meter. | Far yaw trusted less. | Far yaw trusted more. Risky if PnP yaw is noisy. |
| `max_oblique_deg` | `65.0` | Beyond this face obliquity, yaw noise is inflated/ignored. | Trusts yaw at more oblique angles; riskier. | Ignores yaw earlier; safer but less informative. |

### EKF damping

Damping is normalized by frame rate:

```text
damping = alpha^(dt * ref_freq)
```

| Parameter | Current value | Effect | If increased toward 1.0 | If decreased |
|---|---:|---|---|---|
| `alpha_pos` | `0.995` | Damps translational velocity while tracking. | Keeps velocity longer; less under-lead on movers. | Velocity decays faster; smoother but lags moving targets. |
| `alpha_yaw` | `1.00` | Damps yaw velocity while tracking. | Already no damping. | Lower values reduce spin persistence but can under-predict spinners. |
| `alpha_coast` | `0.98` | Damps velocity in TEMP_LOST/coast mode. | Coasts longer through short losses. | Stops prediction faster after detection loss. |
| `ref_freq` | `60.0` | Reference detector rate used by damping math. | Must match measured `/detector/armors_keypoints` rate. Wrong value changes effective damping. | Measure with `ros2 topic hz /detector/armors_keypoints`; do not guess from camera FPS. |

### Target geometry: spinning-top radius and height pair

| Parameter | Current value | Effect | If increased | If decreased |
|---|---:|---|---|---|
| `initial_radius` | `0.24` | Initial distance from robot center to visible armor plate. | Center estimate moves farther behind the armor; affects spinner prediction strongly. | Center estimate moves closer to armor; can under-predict face positions. |
| `radius_ema_alpha` | `0.05` | EMA update rate for learned radius. | Radius adapts faster but follows PnP noise. | Radius is stable but adapts slowly. |
| `initial_dz` | `0.05` | Initial vertical offset between armor pairs. | Larger assumed high/low armor separation. | Smaller high/low correction. Wrong value causes vertical miss on one armor pair. |

### Ballistics and barrel/camera calibration

These parameters should be measured physically. Do not use EKF gains to hide bad geometry.

| Parameter | Current values | Effect | If increased | If decreased |
|---|---:|---|---|---|
| `bullet_speed` | standard/hero `25.0`, sentry `20.0` | Muzzle velocity used by gravity compensation and flight-time prediction. | Solver predicts less drop and shorter flight time; too high usually shoots low. | Solver predicts more drop and longer flight time; too low usually shoots high. |
| `gravity` | `9.8` | Gravity used by ballistic solver. | Do not tune this to fix aim. | Leave at physical gravity. |
| `gimbal_height` | `0.400` | Height of gimbal/camera reference in the robot/world frame. | Changes vertical geometry. Measure physically. | Wrong value creates range-dependent pitch error. |
| `barrel_offset_x` | `0.0` | Barrel offset from active camera lens along forward/back axis. | Affects near targets most. | Usually less critical than Y/Z but should be measured. |
| `barrel_offset_y` | standard/hero `0.02`, sentry `0.08` | Lateral muzzle offset from active lens. Positive means muzzle left of active lens in the documented convention. | Shifts horizontal aim solution. If shots are systematically left/right, check this early. | Same. Wrong sign causes systematic horizontal misses. |
| `barrel_offset_z` | standard/hero `-0.05`, sentry `-0.15` | Vertical muzzle offset from active lens. Negative means muzzle below lens. | More negative means the solver compensates for a lower barrel. | Wrong value creates systematic vertical misses, especially at close/medium range. |
| `pitch_offset_deg` | code default `0.0` | Static pitch boresight correction in degrees. Not currently set in launch. | Use only after barrel offset and bullet speed are validated. | Do not use as first-line tuning. |
| `yaw_offset_deg` | code default `0.0` | Static yaw boresight correction in degrees. Not currently set in launch. | Use only for small residual static yaw bias. | Do not use to hide wrong `barrel_offset_y`. |

### Fire window and distance gates

| Parameter | Current values | Effect | If increased | If decreased |
|---|---:|---|---|---|
| `angular_window` | `1.0` | Angular window for deciding whether the predicted armor face is shootable. | Robot shoots more often; useful for tuning but can fire at bad face angles. | More precise timing, but may stay in HOLD against fast spinners. |
| `window_ref_dist` | standard/hero `1.0`, sentry `3.0` | Reference distance used to scale the angular fire window. | Changes how permissive the window is at range. | Too strict can block long-range shots; too loose wastes shots. |
| `min_fire_dist` | `0.2` | Minimum allowed shooting distance. | Larger value blocks close targets. | Smaller value allows very close shots; PnP/barrel geometry may be unstable. |
| `max_fire_dist` | `6.0` | Maximum allowed shooting distance. | Allows farther shots but accepts noisy long-range PnP. | Blocks far shots even if tracking works. |

### Latency and prediction horizon

| Parameter | Current value | Effect | If increased/enabled | If decreased/disabled |
|---|---:|---|---|---|
| `use_measured_latency` | `true` | Uses capture timestamp to measure pipeline latency and adds `actuation_latency`. | Keep enabled for normal operation. | If disabled, prediction uses fixed `time_bias`, which is less accurate. |
| `actuation_latency` | `0.020` | Extra fixed latency for serial TX, gimbal settling, and muzzle exit. | Predicts farther into the future. Increase if shots trail a moving target after static aim is correct. | Predicts less. Decrease if shots lead too much. Tune in 5 ms steps. |
| `time_bias` | `0.045` | Fixed fallback prediction horizon used only when `use_measured_latency=false`. | Larger = more lead. | Smaller = less lead. Usually not active with measured latency enabled. |

### Spinner / face-jump timing

| Parameter | Current value | Effect | If increased | If decreased |
|---|---:|---|---|---|
| `yaw_jump_thresh` | `0.55` | Threshold for detecting armor yaw/face jumps. | Fewer false jumps, but can miss real face switches. | More sensitive, but can treat noise as a jump. |
| `use_vyaw_from_timing` | `true` | Estimates spin rate from time between 90-degree face jumps. | Keep enabled for spinning-top targets. | EKF relies more on normal yaw updates and converges slower. |
| `vyaw_timing_min_dt` | `0.050` | Minimum valid time between face jumps. | Rejects faster apparent jumps. | Accepts faster jumps but may accept noise. |
| `vyaw_timing_max_dt` | `0.500` | Maximum valid time between face jumps. | Accepts slower rotations. | Rejects slow rotations/noisy delayed jumps. |
| `vyaw_fire_threshold` | `5.0` | Spin-rate threshold for spinner-aware fire behavior. | Requires faster spin before special handling. | Treats slower targets as spinners earlier. |

### Matching, Mahalanobis gate, and target switching

| Parameter | Current value | Effect | If increased | If decreased |
|---|---:|---|---|---|
| `max_match_dist` | `0.8` | Maximum distance between predicted armor and detection before hard reset/switch logic. | More tolerant, fewer lost states, but can merge different targets. | Stricter, cleaner identity, but more flicker. |
| `maha_threshold` | `16.9` | Mahalanobis gate threshold for accepting a measurement. | Accepts more outliers; fewer dropouts. | Rejects more outliers; can lose valid oblique/noisy targets. |
| `switch_range_ratio` | `0.85` | New target must be closer than current range times this ratio. | Closer to `1.0` means easier switching. | Lower means harder switching and more target loyalty. |
| `switch_cooldown` | `10` | Frames before another switch is allowed. | Less flip-flop between enemies. | More reactive but more unstable. |
| `same_target_identity_dist` | `1.0` | Same-color detections within this distance are treated as the same physical target. | Less switching between nearby detections, but can merge two robots. | More switching, but separates nearby robots better. |

### Command smoothing, rate limits, safety holds, and fire lock

| Parameter | Current values | Effect | If increased | If decreased |
|---|---:|---|---|---|
| `cmd_smooth_alpha` | `1.00` | EMA smoothing of yaw/pitch commands. `1.0` is effectively no smoothing. | More responsive as it approaches 1.0. | Smoother but more lag. |
| `cmd_deadband_yaw` | `0.005` | Yaw error below this is snapped to current feedback. | Less tiny jitter, but more residual error. | More precision, but gimbal can twitch. |
| `cmd_deadband_pitch` | `0.005` | Pitch deadband. | Same as yaw. | Same as yaw. |
| `cmd_rate_limit_yaw` | `0.0` | Max yaw command speed. `0.0` disables rate limit. | If set too low, gimbal lags. | `0.0` gives maximum response but less protection against jumps. |
| `cmd_rate_limit_pitch` | `0.0` | Max pitch command speed. `0.0` disables rate limit. | If set too low, pitch lags and fire-lock may fail. | `0.0` gives maximum response. |
| `fire_lock_yaw` | standard/hero `0.05`, sentry `0.5` | Required yaw agreement between command and micro feedback before firing. | Easier firing, but can shoot while not aligned. `0.5 rad` is very loose and should be considered tuning/debug only. | More precise firing, but more HOLD. Must stay larger than deadband. |
| `fire_lock_pitch` | standard/hero `0.04`, sentry `0.5` | Required pitch agreement before firing. | Easier firing, but vertical misses if too loose. | More precise, but can block shots if pitch sign/feedback is wrong. |
| `micro_pitch_feedback_opposite_sign` | `true` | Corrects pitch feedback for internal geometry/PnP. | Set true if raw micro pitch feedback is opposite sign from physical/command convention. | Wrong value mirrors vertical geometry and causes systematic pitch errors. |
| `micro_pitch_lock_opposite_sign` | `false` | Corrects only the pitch feedback used in the final fire-lock comparison. | Set true only if the lock comparison needs inverted raw pitch. | Wrong value creates artificial pitch error and can cause no-fire/hold behavior. This flag is logically separate from `micro_pitch_feedback_opposite_sign`. |
| `cmd_hold_time` | `0.25` | Holds last safe command after brief invalid tracking. | More tolerant of short dropouts, but can hold stale aim longer. | Safer reset, but more visible command dropouts. |
| `cmd_max_delta_yaw` | standard/hero `0.80`, sentry `1.0` | Maximum yaw jump allowed from current feedback to target command. | Allows larger snaps; can recover faster but risks jumps. | Safer but can prevent fast target acquisition. |
| `cmd_max_delta_pitch` | `0.80` | Maximum pitch jump allowed. | Same as yaw. | Same as yaw. |
| `require_aim_inside_frame` | `false` | Requires aim/impact projection to be inside the image before command is valid. | If true, safer visually but can block valid barrel-offset solutions. | False is better for match behavior once geometry is trusted. |

### Ego-motion compensation

Use this only when micro velocity data is validated. Bad ego-motion is worse than no ego-motion.

| Parameter | Current value | Effect | If increased/enabled | If decreased/disabled |
|---|---:|---|---|---|
| `use_ego_motion_compensation` | `true` | Master switch for compensating our robot motion. | Enables ego compensation only if `ego_velocity_available=true`. | Disables ego-motion correction. |
| `ego_velocity_available` | `true` | Claims `/micro_status[2]` and `[3]` contain valid vx/vy. | If true with bad firmware data, the world frame drifts and aim becomes wrong. | If false, ego compensation is fully disabled and safer for stationary testing. |
| `ego_velocity_body_frame` | `true` | Treats velocity as chassis/body-frame velocity and rotates it to world. | Correct only if chassis heading is known or fallback is acceptable. | If false, velocity is assumed already in world frame. |
| `ego_velocity_scale_x` | `1.0` | Scale factor for micro vx. | Fixes unit/scaling mismatch. | Wrong scale under/over-compensates robot motion. |
| `ego_velocity_scale_y` | `1.0` | Scale factor for micro vy. | Same as x. | Same as x. |
| `ego_velocity_max` | `3.0` | Velocity clamp in m/s. | Allows faster measured motion but also more spikes. | Rejects spikes but can clip real motion. |
| `ego_position_max_drift` | `0.0` | Drift cap for dead-reckoned ego position. `0.0` disables reset. | Positive value limits accumulated drift and shifts tracker frame during reset. | `0.0` avoids reset discontinuities but allows unbounded drift if velocity is wrong. |
| `chassis_heading_index` | `-1` | Index in `/micro_status` containing chassis heading. `-1` means unavailable. | Set to real index once firmware sends chassis yaw. Required for accurate spinning chassis compensation. | `-1` falls back to gimbal yaw, which is wrong during chassis spinning. |
| `gimbal.yaw_sign` | `1.0` | Sign correction for internal yaw geometry. | Use only for convention/sign correction. | Wrong sign mirrors horizontal tracking. |
| `gimbal.pitch_sign` | `-1.0` | Sign correction for internal pitch geometry. | Use only for convention/sign correction. | Wrong sign mirrors vertical tracking. |

---

## 4. Serial bridge parameters

The serial bridge sends this packet to the microcontroller:

```text
TX[0] timestamp
TX[1] turret yaw target
TX[2] turret pitch target
TX[3] shoot flag
TX[4] nav_x
TX[5] nav_y
TX[6] rotating chassis / autospin slot
```

It receives:

```text
RX[0] yaw
RX[1] pitch
RX[2] vx
RX[3] vy
RX[4] color/team
RX[5] game_progress
RX[6] HP
RX[7] resupply status
RX[8] center status
RX[9] reserved
```

### Runtime / safety parameters

| Parameter | Current value | Effect | If increased/enabled | If decreased/disabled |
|---|---:|---|---|---|
| `shooting_active` | `false` | Global bridge-side shoot enable. If false, TX shoot is forced to 0 even if autoaim requests fire. | Set true only when live firing is intended and the robot is safe. | False is correct for dry testing; robot aims but does not shoot. |
| `rotating_chassis` | `false` | Sends TX[6] = 1 when enabled. | Enables firmware-side rotating/autospin behavior if implemented. | No rotating/autospin request. |
| `serial_port` | launch arg, default `/dev/ttyACM0` | Microcontroller device path. | Use stable udev symlink. | Wrong path = no serial. |
| `serial_baudrate` | `500000` | Serial baudrate. | Must match firmware exactly. | Mismatch corrupts communication or prevents connection. |
| `serial_parity` | code default `even` | Serial parity mode. | Must match firmware. | Wrong parity corrupts RX/TX. |
| `serial_tx_hz` | `100.0` | TX timer frequency. | Lower command latency, more load. | More latency, less load. |
| `serial_reconnect_interval` | `2.0` | Time between reconnect attempts. | Higher means less spam but slower recovery. | Lower means faster recovery but more log noise. |
| `serial_rx_timeout` | `3.0` | No valid RX for this long triggers reconnect. | Slower failure detection. | Faster failure detection; too low can false-reconnect. |
| `cmd_timeout` | `0.3` | If turret command is stale, shoot is forced to 0. | Tolerates longer ROS gaps but can keep old commands longer. | Safer, but can drop fire on brief scheduling jitter. |
| `use_framed_protocol` | `false` | Enables header + CRC8 framed packets. Requires matching micro firmware. | Better resync and corruption rejection once firmware supports it. | Raw packets are compatible with old firmware but cannot self-resync after byte loss. |
| `low_latency` | code default `false` | Tries Linux `ASYNC_LOW_LATENCY`. | Can reduce serial latency on supported devices. | Safer/default; many CDC-ACM devices do not support the flag. |
| `thread_priority` | code default `0` | Attempts SCHED_FIFO priority if > 0. | Lower scheduling jitter if permissions allow. | Default normal scheduling. |

### Topic and pipeline switches

| Parameter | Current values | Effect | If changed |
|---|---:|---|---|
| `turret_cmd_topic` | standard/hero `/cmd_vel_AI`, sentry `/turret/cmd` | Topic used for turret yaw/pitch/shoot input. | Wrong topic means the bridge does not receive the intended turret command. |
| `nav_cmd_topic` | `/cmd_vel_NAV` | Topic used for navigation velocity TX. | Wrong topic means nav commands are zero/stale. |
| `micro_status_topic` | `/micro_status` | Published status topic consumed by autoaim/viewer/mux. | Changing it requires changing all downstream subscribers. |
| `enable_nav_pipeline` | standard/hero `false`, sentry `true` | If false, nav TX fields are forced to zero. | Enable only when navigation pipeline is valid. |
| `enable_turret_pipeline` | `true` | If false, shoot is forced to 0 and yaw/pitch are held or zeroed. | Useful for disabling turret without stopping serial. |
| `hold_last_turret_when_disabled` | `true` | When turret pipeline is disabled, keeps last yaw/pitch instead of zeroing. | False sends yaw/pitch zero when disabled. |

### Lab override parameters

These are declared by the C++ serial bridge. They affect only the published `/micro_status`
fields used by ROS-side logic, not the raw serial data sent to the microcontroller.

| Parameter | Default | Effect |
|---|---:|---|
| `lab_override_micro_status` | `true` | Allows overriding status fields for lab testing. |
| `lab_override_team` | `1.0` | Overrides team/color field. `0` red, `1` blue. |
| `lab_override_game_progress` | `4.0` | Overrides match state. `4` means in match. |
| `lab_override_health` | `100.0` | Overrides HP field. |
| `lab_override_resupply_status` | `0.0` | Overrides resupply status. |
| `lab_override_center_status` | `0.0` | Overrides center status. `0` free, `1` ours, `2` enemy. |

---

## 5. Turret yaw mux parameters

Current production note: `sentry.launch.py` passes mux parameters inline. The standalone
`turret_mux.yaml` contains older/legacy parameter names that do not match the current
`turret_yaw_mux.cpp` implementation. Do not assume `turret_mux.yaml` is active unless a
specific launch file loads it.

Current C++ mux parameters:

| Parameter | Current/default | Effect | If increased/enabled | If decreased/disabled |
|---|---:|---|---|---|
| `cv_cmd_topic` | `/cmd_vel_AI` | Input autoaim command topic. | Topic change only; must match publisher. | Wrong topic disables CV command input. |
| `detection_topic` | `/detector/armors` | Detection topic used to decide whether CV is active. | Topic change only. | Wrong topic makes mux think there is no valid detection. |
| `micro_status_topic` | `/micro_status` | Reads current micro yaw/pitch. | Topic change only. | Wrong topic breaks hold/fallback behavior. |
| `output_topic` | `/turret/cmd` | Mux output consumed by serial bridge on sentry. | Topic change only. | Must match `serial_bridge.turret_cmd_topic`. |
| `publish_rate_hz` | default `50.0` | Mux output frequency. | More frequent output, slightly lower mux latency. | Lower load but slower updates. |
| `detection_timeout` | sentry `0.50` | Detection freshness window. | Holds CV mode longer after detection loss. | Exits CV mode faster; can flicker if too low. |
| `cv_cmd_timeout` | sentry `0.50` | Autoaim command freshness window. | Tolerates longer autoaim gaps. | Safer but can drop CV mode on jitter. |
| `min_detection_score` | sentry `0.12` | Minimum detection score for mux-valid detection. | Cleaner CV activation, but can drop weak detections. | More permissive, but can accept false positives. |
| `valid_class_ids` | `["0", "2"]` | Detection classes that count as valid for mux mode. | Include only classes that should trigger CV mode. | Do not include grey class `1` for match logic. |
| `freeze_when_no_detection` | `true` | Holds/freezes turret behavior when detection is lost. | More stable during short detection loss. | If false, output can fall back/zero faster depending on code path. |

Legacy parameters currently present in `turret_mux.yaml` but not used by the current
`turret_yaw_mux.cpp` include:

```text
cv_detection_topic, micro_yaw_index, micro_pitch_index, idle_target_topic,
map_frame, base_frame, turret_cmd_topic, cv_timeout, publish_rate, tf_timeout,
yaw_sign, yaw_zero_offset, idle_yaw_rate_limit, idle_pitch_mode,
idle_pitch_static, use_default_idle_target, default_idle_target_x,
default_idle_target_y
```

If you want those features back, either update `turret_yaw_mux.cpp` to declare and use them
or remove them from the YAML to avoid misleading future debugging.

---

## 6. Viewer parameters

Viewer parameters do not affect the robot command path. They affect only `/tracker/debug_image`
and the HUD interpretation of lock/error.

| Parameter | Current/default | Effect | If changed |
|---|---:|---|---|
| `micro_pitch_feedback_opposite_sign` | launch viewer `false`, code default `true` | Viewer-only correction for pitch error display. | If wrong, the HUD pitch error/lock display lies. It does not change robot firing. |
| `aim_topic_timeout` | `0.30` | Timeout for `/tracker/aim_pixels` marker freshness. | Too low marks fresh aim as stale; too high hides stale aim. |
| `cmd_topic_timeout` | `0.50` | Timeout for `/cmd_vel_AI` command freshness in HUD. | Too low creates false CMD STALE; too high hides dead commands. |
| `fire_lock_yaw` | launch standard/hero `0.05`, viewer default `0.05` | HUD lock threshold. | Should match autoaim for accurate display. |
| `fire_lock_pitch` | launch standard/hero `0.04`, viewer default `0.04` | HUD lock threshold. | Should match autoaim for accurate display. |
| `debug_image_topic` | `/tracker/debug_image` | Viewer output topic. | Change only if multiple debug views are intentionally needed. |
| `exclusive_output` | `true` | Prevents duplicate viewer publishers on the same debug topic. | Keep true to avoid flickering HUD from duplicate launches. |

---

# Current robot-specific differences

## Standard and Hero

```text
DEFAULT_CAMERA = realsense
use turret_yaw_mux = no
serial turret_cmd_topic = /cmd_vel_AI
enable_nav_pipeline = false
bullet_speed = 25.0
barrel_offset_y = 0.02
barrel_offset_z = -0.05
fire_lock_yaw = 0.05
fire_lock_pitch = 0.04
q_yaw = 20.0
min_keypoint_score = 0.30
```

## Sentry

```text
DEFAULT_CAMERA = zed
use turret_yaw_mux = yes
serial turret_cmd_topic = /turret/cmd
enable_nav_pipeline = true
target_classes_from_micro_status = true
bullet_speed = 20.0
barrel_offset_y = 0.08
barrel_offset_z = -0.15
fire_lock_yaw = 0.5
fire_lock_pitch = 0.5
q_yaw = 10.0
min_keypoint_score = 0.15
```

`fire_lock_yaw = 0.5` and `fire_lock_pitch = 0.5` are extremely loose. Treat them as
lab/tuning values, not precision match values.

---

# Calibration order

Do not tune EKF gains before the static geometry is correct.

1. Confirm the selected detector, serial bridge, autoaim node, and viewer all start.
2. Confirm that exactly one camera detector is running.
3. Determine the active camera lens by covering one lens and watching `/yolo/debug_image`.
4. Measure `barrel_offset_x/y/z` from the active lens, not the housing center.
5. Measure real `bullet_speed` before tuning vertical aim.
6. Verify pitch sign with `/micro_status`: compare raw pitch feedback and command echo.
7. Verify `gimbal.yaw_sign` and `gimbal.pitch_sign` with slow physical gimbal motion.
8. Measure detector rate with:

   ```bash
   ros2 topic hz /detector/armors_keypoints
   ```

   Then set `ref_freq` to the real rate.

9. Keep `use_measured_latency=true` and tune `actuation_latency` on a moving target in
   5 ms steps.
10. Only after static aim and latency are correct, tune `q_pos`, `q_yaw`, `r_pos_*`,
    `r_yaw_*`, `angular_window`, and `fire_lock_*`.

---

# Symptom-based tuning guide

| Symptom | First parameters to check | Notes |
|---|---|---|
| Shoots consistently low/high while target is static | `bullet_speed`, `barrel_offset_z`, `gimbal_height`, pitch signs | Do not fix this with `q_pos` or `actuation_latency`. |
| Shoots consistently left/right while target is static | `barrel_offset_y`, `yaw_offset_deg`, active lens selection, `gimbal.yaw_sign` | Re-measure from active lens, especially after switching ZED/RealSense. |
| Hits static targets but trails moving targets | `actuation_latency`, `ref_freq`, `alpha_pos`, `q_pos` | Increase `actuation_latency` in 0.005 s steps only after static calibration. |
| Leads moving targets too much | `actuation_latency`, `time_bias` if measured latency disabled | Reduce prediction horizon. |
| Tracker flickers TRACKING/TEMP_LOST/LOST | `min_keypoint_score`, `max_reproj_error`, `maha_threshold`, `max_match_dist`, exposure/gain | Fix image quality before loosening gates too much. |
| Robot detects but stays in HOLD | `fire_lock_yaw`, `fire_lock_pitch`, `micro_pitch_lock_opposite_sign`, `angular_window`, `min_fire_dist`, `max_fire_dist` | Check HUD and `/cmd_vel_AI.angular.x`. |
| Pitch error appears around 2x expected value | `micro_pitch_lock_opposite_sign` | Lock comparison is probably using the wrong sign. |
| Vertical geometry appears mirrored in RViz/debug | `micro_pitch_feedback_opposite_sign`, `gimbal.pitch_sign` | Feedback sign affects geometry; lock sign affects final fire permission. |
| Detection works but no real shooting | `shooting_active`, `cmd_timeout`, serial connection | `shooting_active=false` forces TX shoot to zero. |
| Serial connects sometimes but data is garbage | `serial_baudrate`, `serial_parity`, `use_framed_protocol`, firmware packet layout | Framed protocol requires matching firmware. |
| Viewer HUD flickers | duplicate `autoaim_viewer` nodes, `exclusive_output` | Check `ros2 topic info /tracker/debug_image --verbose`. |
| Sentry/Nav output does not reach micro | mux `output_topic`, bridge `turret_cmd_topic`, `enable_nav_pipeline` | On sentry, mux must publish `/turret/cmd` and bridge must subscribe to it. |
| Ego compensation makes aim worse | `ego_velocity_available`, `ego_velocity_scale_*`, `chassis_heading_index` | Disable ego velocity until micro vx/vy and chassis yaw are validated. |

---

# Serial notes

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

---

# Git notes

Useful Git commands from the repo workflow notes:

```bash
git log --oneline --graph --decorate --all
```

Temporarily inspect an older commit:

```bash
git switch --detached <commit>
```

Revert the latest commit without rewriting history:

```bash
git revert HEAD
git push origin main
```

---

# Final tuning rule

Static error is calibration. Moving-target error is latency/prediction. Random error is noise/gating.
Do not solve one category by blindly tuning another.