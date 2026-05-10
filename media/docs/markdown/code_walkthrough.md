# Code Walkthrough

## End-To-End Pipeline

The current debug/match runtime path starts when the YOLOv26-pose detector
publishes `auto_aim/ArmorKeypointArray` on `/detector/armors_keypoints`.
The bbox-only `vision_msgs/Detection2DArray` path on `/detector/armors` still
exists as a fallback when `use_keypoints=false`. With the keypoint path, the
node does the following work in one detection callback:

1. Confirm camera intrinsics and IMU yaw/pitch have been received.
2. Filter detections by target class and reject low-score/non-finite corners.
3. Run keypoint PnP for each candidate.
4. Transform each successful PnP pose from camera frame to odom frame.
5. Reject impossible positions by height and distance.
6. Update the tracker with the remaining armor detections.
7. Compute an aim result from the tracker state.
8. Evaluate the fire gate.
9. Publish the gimbal command, legacy command, HUD pixels, RViz markers, and
   structured debug message.

The key design decision is that the node publishes absolute yaw and pitch in
the microcontroller reference frame. It does not publish relative increments.

## `src/src/auto_aim_node.cpp`

### Responsibility

`auto_aim_node.cpp` is the ROS-facing coordinator. It owns:

- all runtime parameters
- subscribers for detections, camera info, and IMU yaw/pitch
- publishers for commands, HUD pixels, markers, and debug data
- the `Tracker`, `PnPSolver`, `FrameTransformer`, `FireGate`, and
  `DebugPublisher` objects
- command smoothing and final sign conversion

It should stay mostly orchestration code. Heavy math belongs in `tracker.cpp`,
`pnp_solver.cpp`, or `frame_transformer.cpp`.

### Constructor

The constructor declares parameters and creates all ROS interfaces. Parameter
declaration has two purposes:

1. It gives every setting a default value.
2. It allows a YAML launch file to override that value.

The first large block fills `TrackerConfig`. That config is passed into the
tracker constructor:

```cpp
tracker_ = std::make_unique<Tracker>(cfg);
cfg_ = cfg;
```

Then the constructor builds a matching `FireGate::Config`. Fire-related
parameters are split because the tracker computes planner feasibility while
the fire gate owns the final fire/hold decision.

The target class list is converted into a `std::set<std::string>` for fast
lookup. Class `"1"` is erased because it represents grey/dead detections and
should not be tracked.

Before starting subscriptions, the constructor validates configuration with
`ConfigValidator::validate`. Errors abort startup. Warnings are printed but
allow the node to run. This is deliberate: a wrong sign, impossible bullet
speed, or empty target list is more dangerous than a startup failure.

### Camera Info Subscription

The `/camera_info` callback initializes the PnP solver once:

```cpp
std::array<double, 9> K;
std::copy(msg->k.begin(), msg->k.end(), K.begin());
pnp_.init(K, msg->d);
```

The matrix `K` contains focal lengths and principal point. The distortion
vector `d` corrects lens distortion. The same callback stores `cam_fx_`,
`cam_fy_`, `cam_cx_`, and `cam_cy_` for HUD pixel projection later.

### Micro IMU Callback

`microImuCallback` receives `/micro_imu`. The message must contain:

```text
data[0] = yaw
data[1] = pitch
```

Both are in radians and in the microcontroller convention. The node multiplies
them by the configured signs:

```cpp
imu_yaw_   = yaw_sign_ * micro_yaw_rad;
imu_pitch_ = pitch_sign_ * micro_pitch_rad;
```

Then it updates the frame transformer. This creates the cached rotation and
translation used by detection callbacks. If no IMU message has arrived, the
detection callback refuses to run because camera points cannot be transformed
into the correct frame.

### Detection Callback

`keypointCallback` is the main hot path for `debug_targeting.launch.py`
because that launch file sets `use_keypoints=true`. `detectionCallback` is the
legacy bbox fallback. Both callbacks start with two guards:

- PnP must be initialized from camera info.
- IMU yaw/pitch must be valid.

It creates a `DebugFrame` at the start and fills it stage by stage. This is why
the debug topic can show where a frame failed.

Frame time `dt` is computed from message timestamps and clamped to `0.01` to
`0.10` seconds. The clamp prevents one bad timestamp from exploding the EKF
prediction.

In the keypoint path, the callback filters `ArmorKeypointArray` detections by
`target_classes`, checks the TL/TR/BR/BL corner scores, and calls
`PnPSolver::solveKeypoints`.

In the fallback bbox path, the detection adapter filters raw detections:

```cpp
auto candidates = DetectionAdapter::convert(*msg, target_classes_);
```

For each candidate, the node runs PnP:

```cpp
auto pnp = pnp_.solve(cand.cx, cand.cy, cand.w, cand.h,
                      cfg_.light_ratio, 10.0, enable_pnp_refine_);
```

Failed PnP results are still written into the debug frame when possible. That
helps diagnose whether a detection was dropped because of PnP quality rather
than tracker logic.

Successful PnP results go through the frame transformer:

```cpp
auto td = frame_transformer_.apply(pnp.rvec, pnp.tvec, max_oblique_rad);
```

Then the node rejects positions with excessive height or range:

- `max_armor_z` limits vertical position.
- `max_armor_distance` limits horizontal range.

The remaining detections are converted into `ArmorDetection` values and passed
to the tracker:

```cpp
tracker_->update(armors, dt);
```

After the tracker update, the node copies EKF state, innovation, Mahalanobis
distance, and target id into the debug frame.

### Aim Computation And Latency

The node calls:

```cpp
auto aim = tracker_->computeAim(imu_yaw_, imu_pitch_, override_pred_lead);
```

Normally `override_pred_lead` is `-1.0`, which tells the tracker to use the
legacy `time_bias` calculation. If `use_measured_latency` is enabled and
initialized, the node passes an EMA of measured pipeline latency plus
`gimbal_response_delay_s`.

The callback also projects the selected impact point back into camera-relative
yaw and pitch. The fire gate uses this camera-frame total angle as the
`OFF_AXIS` check.

### Command Publishing

`publishCommand` is where the final command is formed. The order matters:

1. Start from `aim.abs_yaw` and `aim.abs_pitch`.
2. Subtract bore-sight offsets.
3. Store pre-smoothing values for debug.
4. Apply EMA smoothing if tracking.
5. Compute smoothing lag.
6. Call `FireGate::evaluate`.
7. Convert yaw and pitch back to the microcontroller convention using
   `gimbal.yaw_sign` and `gimbal.pitch_sign`.
8. Publish `geometry_msgs::Twist`.

The primary command contract is:

| Field | Meaning |
| --- | --- |
| `angular.z` | absolute yaw target in radians |
| `angular.y` | absolute pitch target in radians |
| `angular.x` | fire flag, `1.0` means fire and `0.0` means hold |
| `linear.x` | range to selected armor face in meters |

The same yaw, pitch, and fire values are also published on the legacy topic
`/cmd_vel_AI`. The legacy topic does not include distance.

### Markers

`publishMarkers` publishes RViz markers:

- enemy center sphere
- four armor cubes
- velocity arrow
- approximate aim point
- selected impact point

When the tracker is not tracking, the function publishes delete actions for
old markers so RViz does not show stale geometry.

## `src/include/auto_aim/debug_frame.hpp`

`DebugFrame` is an internal plain data struct. The detection callback fills it
during processing. `DebugPublisher` copies it into the ROS message at the end.

This two-step design keeps the hot callback readable. Each stage can write its
own fields without constructing a ROS message directly.

## `src/src/detection_adapter.cpp`

### Responsibility

`DetectionAdapter::convert` turns a `vision_msgs::msg::Detection2DArray` into
a vector of `DetectionCandidate` structs.

This is the fallback bbox path. The current `debug_targeting.launch.py`
runtime path uses `/detector/armors_keypoints` and `keypointCallback` instead.

It performs three checks:

1. Ignore detections with no classification result.
2. Ignore detections whose class id is not in `target_classes`.
3. Ignore boxes smaller than `min_pixel_size`.

The adapter does not know about PnP, tracking, or firing. That separation is
useful: detector-message parsing can be tested and changed without touching
the EKF.

## `src/src/pnp_solver.cpp`

### Initialization

`PnPSolver::init` stores camera intrinsics and distortion. It also builds two
3D armor models:

- small armor
- large armor

The object points lie on a flat plane with `x = 0` and dimensions converted
from millimeters to meters.

### Solving

The current keypoint path calls `PnPSolver::solveKeypoints` with four image
points ordered TL, TR, BR, BL from `ArmorKeypointArray`. It tests the small and
large armor models and keeps the lower reprojection error.

The fallback bbox path calls `PnPSolver::solve`. It first rejects invalid
input:

- non-finite center or size
- boxes below minimum size
- invalid `light_ratio`

It then builds four synthetic image points from the bounding box. The helper
`try_model` runs OpenCV `solvePnP` with `SOLVEPNP_IPPE`, which is suited for
planar targets. If `enable_pnp_refine` is true, it runs a
Levenberg-Marquardt refinement step.

Both small and large armor models are tested. The lower reprojection error
wins. The output `PnPResult` contains:

- `ok`: true only if solve succeeded and error is below threshold
- `solver_ok`: true if PnP itself produced a finite result
- `is_large`: which model won
- `rvec` and `tvec`: OpenCV pose output
- `reproj_err` and `reproj_err_norm`: quality metrics
- `image_points`: direct YOLOv26 corners in the keypoint path, synthetic
  corners in the fallback bbox path

## `src/src/frame_transformer.cpp`

### `updateFromImu`

`updateFromImu` converts current yaw and pitch into a camera-to-odom rotation.

```cpp
q_gimbal.setRPY(0.0, imu_pitch, imu_yaw);
q_conv.setRPY(-M_PI / 2.0, 0.0, -M_PI / 2.0);
rotation_ = q_gimbal * q_conv;
translation_ = tf2::Vector3(0, 0, gimbal_height_);
```

`q_conv` is the fixed rotation from gimbal body axes to camera optical axes.
`q_gimbal` changes with the IMU. The translation lifts the camera to gimbal
height.

### `apply`

`apply` receives OpenCV `rvec` and `tvec`. It:

1. converts `tvec` into a camera-frame point
2. converts `rvec` into a quaternion
3. rotates and translates position into odom
4. rotates orientation into odom
5. extracts yaw
6. corrects or clamps impossible armor yaw
7. returns `TransformedDetection`

If orientation extraction fails, `valid` remains false and the detection is
ignored.

## `src/src/tracker.cpp`

### Responsibility

`Tracker` is the largest math module. It owns:

- EKF state `x_`
- covariance `P_`
- target state machine
- target association
- armor face jump handling
- radius and alternate-radius tracking
- ballistic aim computation
- optional adaptive process noise and anti-gyro timing

### Constructor

The constructor initializes a 9-element zero state and a diagonal initial
covariance `P0_`. Larger covariance means less certainty. For example, yaw
rate starts with relatively large uncertainty because it is hard to know from
one detection.

### `ekfPredict`

`ekfPredict` runs before measurement association unless the tracker is `LOST`.
It applies damped constant velocity to position, height, and yaw.

When state is `TEMP_LOST`, it uses `alpha_coast` to decay velocity faster
because no measurement is currently confirming the motion.

If `enable_adaptive_q` is true, process noise is changed dynamically:

- stationary targets get reduced process noise to suppress jitter
- high yaw-rate targets get increased yaw process noise for response

After prediction, covariance is symmetrized and bounded. Bounding prevents
coasting uncertainty from growing without limit.

### `ekfUpdate`

`ekfUpdate` applies a measurement:

```text
z = [xa, ya, za, yaw]^T
```

It builds the Jacobian `H`, the range- and obliquity-dependent measurement
noise `R`, then computes innovation, Kalman gain, state update, and
Joseph-form covariance update.

Two debug values are stored:

- `last_innovation_norm_`
- `last_mahalanobis_`

These are published so operators can tell whether the tracker agrees with
incoming detections.

### `ekfMahalanobis`

This function computes the association distance for a candidate detection. A
candidate must have distance below `maha_threshold` to be associated with the
active target.

If the math solve fails, it returns a very large number so the detection is
rejected.

### `initFromDetection`

When the tracker is `LOST`, the closest valid detection initializes the state.
The detected armor is converted into an estimated enemy center by adding radius
in the face-yaw direction:

```text
xc = xa + r cos(yaw)
yc = ya + r sin(yaw)
```

Velocities start at zero. The target id becomes the detection's class id.

### `shouldSwitch`

Target switching is conservative. The tracker does not switch while
`DETECTING` or `TRACKING`. In `TEMP_LOST`, it can switch after a cooldown if a
new target is clearly closer according to `switch_range_ratio`.

This avoids bouncing between two similar targets.

### `handleArmorJump`

When the target rotates and another face becomes visible, measured yaw can
jump. `handleArmorJump` snaps yaw, handles possible spin reversal, updates
alternate face radius/height information, and either resets or inflates
covariance depending on whether the inferred armor position is plausible.

Then it runs a normal EKF update using the jump measurement.

### `update`

`update` is the main per-frame tracker entry point. It has these stages:

1. Decrement target-switch cooldown.
2. Possibly switch targets if state allows it.
3. Initialize from closest detection if state is `LOST`.
4. Predict EKF state.
5. Associate detections using class id and Mahalanobis gate.
6. Run normal update, face-jump handling, or miss handling.
7. Clamp radius and yaw rate.
8. Advance the state machine.

The state machine logic at the end is important:

- `DETECTING` needs `confirm_frames` consecutive matches.
- `TRACKING` becomes `TEMP_LOST` after one miss.
- `TEMP_LOST` becomes `LOST` after `lost_timeout`.

### `solveBallistic`

`solveBallistic` returns pitch and flight time for a target at horizontal
distance `gd` and vertical difference `dz`. It iterates five times to
compensate gravity. It rejects too-close, too-slow-horizontal, or extreme
pitch solutions.

### `computeAim`

`computeAim` turns the EKF state into a gimbal command. It returns an empty
`AimResult` unless the tracker is `TRACKING` or `TEMP_LOST`.

The function computes the barrel position from current yaw and configured
barrel offset. It then evaluates four future faces over two passes. Each face
candidate stores:

- predicted 3D face position
- range and bearing from barrel
- ballistic pitch and flight time
- fire margin
- visibility score
- anti-gyro residual when applicable

The best face is the one with highest margin, with visibility as a tie-break.
In anti-gyro mode, the best viable residual can override the margin pick.

The final `AimResult` contains:

- absolute yaw and pitch
- relative yaw and pitch for debug/fire only
- distance
- selected target point
- tracking and fire booleans
- selected face index
- flight time and prediction time
- anti-gyro residual

## `src/src/fire_gate.cpp`

### Responsibility

`FireGate` is the single source of truth for fire/hold. It receives
`FireGate::Inputs` from the command publisher and returns a `Decision` with:

- `fire`: true or false
- `blocker`: enum reason
- `reason`: human-readable debug string

### Decision Order

The order is deliberate:

1. Not tracking blocks immediately.
2. Invalid target blocks immediately.
3. Invalid ballistic solution blocks immediately.
4. Out-of-range blocks immediately.
5. Optional stale measurement blocks.
6. Optional anti-gyro timing blocks.
7. Margin, off-axis, and smoothing checks decide normal fire.
8. Hysteresis may keep fire true across small angular drift.
9. The most informative remaining blocker is returned.

Hard failures reset hysteresis. Hysteresis only helps small geometry-like
drifts after fire was already allowed.

## `src/src/debug_publisher.cpp`

`DebugPublisher` creates the `/auto_aim/debug` publisher. Its `publish`
function copies every `DebugFrame` field into the generated ROS message type.

It also maintains a fire blocker histogram. Every `fire_log_period_s` seconds
it logs counts and percentages for recent blocker reasons. This is useful when
fire is not allowed and individual messages are too noisy to inspect by eye.

## `src/src/config_validator.cpp`

`ConfigValidator` checks parameter ranges before the node fully starts. It
separates warnings from errors:

- Errors mean the node should not run. Examples: empty target classes,
  impossible sign values, or max fire distance less than min fire distance.
- Warnings mean the value is unusual but not necessarily impossible. Examples:
  nonzero bore-sight offsets or suspicious physical dimensions.

Use the validator as the first place to add checks when adding new parameters.
Bad parameter values should fail loudly at startup, not silently mis-aim on the
field.

## Headers In `src/include/auto_aim`

The headers expose the package's internal module boundaries:

| Header | What to read there |
| --- | --- |
| `tracker.hpp` | Main data structs, tracker config, public tracker API, state enum, and private helper declarations. |
| `pnp_solver.hpp` | PnP result fields, armor dimensions, solve signature. |
| `frame_transformer.hpp` | Frame definitions and transformed detection output. |
| `fire_gate.hpp` | Fire blocker enum, input fields, config fields, and decision output. |
| `detection_adapter.hpp` | Detection candidate representation and convert function. |
| `debug_frame.hpp` | Internal debug fields populated across the callback. |
| `debug_publisher.hpp` | Debug publisher interface. |
| `config_validator.hpp` | Validation result and validator interface. |

## Messages

### `src/msg/AutoAimDebug.msg`

This is the most important message for debugging. It groups fields by pipeline
stage:

- detection
- PnP
- frame transform
- EKF
- aim planner
- command publisher
- fire gate
- latency

When a bug appears, inspect these groups in order. For example, if
`pnp_tvec_z` is unstable on a static target, do not tune the EKF first.

### `src/msg/GimbalCmd.msg`

This message is not used at runtime. The actual command topic is
`/tracker/cmd_gimbal` with `geometry_msgs/Twist`. Keep `GimbalCmd.msg` only as
a placeholder unless the team intentionally migrates the firmware contract.

## Configuration Files

`params_realsense_16.yaml` and `params_zed_64.yaml` provide robot-specific
runtime values. The two files are similar, but physical values such as
`gimbal_height`, `barrel_offset_x`, and `barrel_offset_z` should be measured
per robot.

Important categories:

- detection/tracking: target classes, light ratio, max armor range, state
  machine settings
- EKF: process noise, measurement noise, damping
- physical: bullet speed, gravity, gimbal height, barrel offset
- fire gate: angular window, range limits
- timing: prediction bias, reference frequency
- command shaping: smoothing alpha and sign convention

If the code has a default value and the YAML has a different value, the YAML
wins at launch.

## Launch Files

`auto_aim_realsense_16.launch.py` and `auto_aim_zed_64.launch.py` load the
matching YAML file and start `auto_aim_node`. They assume upstream nodes
provide:

- `/detector/armors`
- `/camera_info`
- `/micro_imu`

`auto_aim_launch.py` declares parameters inline. It is useful as an older
single-file example, but the robot-specific YAML launch files are clearer for
real deployments.

## Build Files

### `src/CMakeLists.txt`

CMake does the following:

1. sets C++17
2. finds ROS 2, OpenCV, Eigen, tf2, and message-generation dependencies
3. generates message code from `msg/GimbalCmd.msg` and `msg/AutoAimDebug.msg`
4. builds `libauto_aim_component.so` from all C++ source files
5. links generated message typesupport
6. registers `auto_aim::AutoAimNode` as a composable node and standalone
   executable
7. installs headers, launch files, config files, and docs

If a new C++ source file is added, it must be listed in `add_library(...)` or
it will not be compiled.

### `src/package.xml`

`package.xml` is ROS package metadata. It names dependencies so ROS tools know
what must be present to build and run the package.

## Debugging Recipes By Symptom

### Node Starts But Publishes No Commands

Check:

1. Is `/camera_info` publishing?
2. Is `/micro_imu` publishing at least two floats?
3. Is `/detector/armors` publishing?
4. Does `/auto_aim/debug` show `detection_present=true`?
5. Is tracker state stuck in `LOST` or `DETECTING`?

### PnP Fails Often

Check:

1. Bounding boxes are not too small.
2. `light_ratio` is reasonable.
3. Camera intrinsics are correct.
4. `pnp_reproj_err_norm` is not large on clean frames.

### Target Position Jitters While Robot Is Still

Check in order:

1. bbox center and size jitter
2. PnP `tvec`
3. `/micro_imu` yaw and pitch noise
4. odom position
5. EKF innovation and covariance behavior

### Pitch Is Consistently Wrong

Check:

1. measured `bullet_speed`
2. `gimbal_height`
3. `barrel_offset_z`
4. pitch sign
5. whether `pitch_offset_deg` was used to hide a calibration issue

### Fire Is Never Allowed

Check `fire_blocker` and `fire_blocker_reason`. Common causes:

- `NOT_TRACKING`: tracker has not reached `TRACKING`
- `OUT_OF_RANGE`: distance outside configured range
- `MARGIN_NEGATIVE`: selected face is not aligned
- `OFF_AXIS`: target is outside camera angular window
- `SMOOTHING_LAG`: command smoothing has not caught up

## Good First Code Tasks

1. Add a new field to `DebugFrame` and `AutoAimDebug.msg`, then publish it
   through `DebugPublisher`. This teaches the full debug-message path.
2. Add a validator warning for a new parameter. This teaches safe startup
   checks.
3. Add a console log for one state transition in `Tracker::update`. This
   teaches tracker flow without changing math.
4. Add a small offline unit test for `FireGate::evaluate`. This is a contained
   logic module with no ROS subscriptions.

## Rules For Future Maintainers

- Keep units visible in comments and parameter names.
- Keep frame conversions inside `FrameTransformer` or very close to where data
  enters/leaves a frame.
- Keep fire decision logic centralized in `FireGate`.
- Do not change the `/tracker/cmd_gimbal` contract without firmware
  coordination.
- Prefer adding debug fields over guessing from terminal logs.
- When fixing a bug, identify the earliest pipeline stage where the data
  becomes wrong.
