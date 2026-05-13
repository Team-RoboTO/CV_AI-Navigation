# Auto Aim Targeting Audit And Refactor Changes

Date: 2026-05-13

Scope:

- Active runtime package: `src/auto_aim`
- Launch package: `src/launch_pkg`
- Main launch file audited: `src/launch_pkg/launch/debug_targeting.launch.py`
- Reference tree compared: `auto_aim_complete_refactor`

Note: the requested `auto_aim_targeting` source package is not present under
`src/`. Only stale `build/`, `install/`, and log artifacts exist. The active
targeting path is `src/auto_aim`. `launch_pkg/package.xml` still referenced the
removed package, so that stale dependency was removed.

## Log Evidence Used

The files present in the workspace were `last_logs.txt` and `last_logs2.txt`,
not `logs.txt` / `logs2.txt`.

From `last_logs.txt`:

- `Keypoint PnP OK`: 52 frames
- PnP reprojection error:
  - min: about `0.03 px`
  - mean: about `0.54 px`
  - max: about `1.09 px`
- PnP size selection:
  - small: 42
  - large: 10
- PnP camera depth range: about `1.11 m` to `7.20 m`
- Pose rejects: 7
- Frames passing zero armors to tracker: 47
- Tracker state log counts:
  - `LOST`: 46
  - `DETECTING`: 10
  - `TRACKING`: 21
  - `TEMP_LOST`: 17
- Fire blocker histograms were dominated by:
  - `NOT_TRACKING`
  - `OFF_AXIS`

From `last_logs2.txt`:

- `detection_present: false`
- `tracker_state: 0`
- `cmd_yaw_published: 0.0`
- `cmd_pitch_published: 0.0`
- `fire_blocker: 1`
- `fire_blocker_reason: tracker is not in TRACKING state`

## Root Causes Found

### Aim leaving the detection

Confirmed code causes:

- The no-target path published absolute yaw and pitch zero.
- The command contract says yaw/pitch are absolute destinations, not deltas.
- Therefore `0,0` is not a harmless hold command; it can command the gimbal
  away from the current target whenever tracking drops.
- The old `auto_aim_complete_refactor` code had the same bug:
  `cmd.angular.z = aim.tracking ? yaw_target_micro : 0.0`.

Confirmed launch/debug causes:

- `debug_targeting.launch.py` had an old fake IMU mode coupled to video motion.
- That made the camera-to-odom transform depend on video-frame optical flow.
- This is debug-hostile because detections, PnP, and tracker output can look
  wrong even when the detector itself is correct.
- The debug launch also overrode tracker settings with aggressive values:
  - broad `target_classes`
  - `confirm_frames: 1`
  - low switch cooldown
  - tracking switch enabled
  - high command smoothing
- Those settings made target jumping and unstable association more likely.

Log-confirmed contributing causes:

- PnP reprojection was usually good, so the main issue was not simple PnP
  pixel error.
- PnP size selection flipped between small and large in the logs. That can
  still move the 3D pose even when reprojection remains low.
- Detections were intermittent; many frames passed zero armors to the tracker.

### Robot never firing

Confirmed from logs:

- Fire was mostly blocked by `NOT_TRACKING` and `OFF_AXIS`.
- Later logs showed the tracker in `LOST`, with no active detection.

Confirmed code causes:

- Fire had no explicit pose freshness blocker, even though `/micro_imu` defines
  the camera-to-odom transform.
- `TEMP_LOST` was not distinguishable from other non-fire states in the fire
  diagnostics.
- No-target zero commands made `OFF_AXIS` and tracker instability harder to
  diagnose because the command path itself could move away from the target.

Clarification:

- With static fake IMU, `OFF_AXIS` can be a correct blocker when the target is
  not centered. Static fake IMU does not represent a real gimbal following the
  command. It is a stable test pose source, not proof that the physical gimbal
  has converged.

### Tracker stuck in TEMP_LOST or unstable

Confirmed causes:

- The debug launch was forcing aggressive target switching and one-frame
  confirmation.
- Association Mahalanobis gating used a covariance model that did not match
  the EKF update covariance for oblique armor views.
- That can reject measurements in the association stage that the EKF update
  would otherwise down-weight safely.
- Initial/switch matches did not update some match-quality bookkeeping, making
  debug age and match counters less useful.

Not confirmed for active code:

- The active node does not reconstruct armor corners from axis-aligned YOLO
  bounding boxes. It uses YOLO-pose keypoints from
  `/detector/armors_keypoints`.
- Therefore bbox-derived PnP corners were not the active cause in this code
  path.

Still possible:

- PnP small/large armor ambiguity.
- Wrong enemy `target_classes`.
- Gimbal sign mismatch.
- Camera mount pitch or transform mismatch.
- Firmware interpreting absolute commands differently than expected.

## Files Changed

### `src/auto_aim/scripts/fake_micro_imu_node.py`

Replaced the old fake IMU implementation with a static-only testing publisher.

New behavior:

- Publishes `std_msgs/Float32MultiArray` on `/micro_imu`.
- Default message:
  - `data[0] = 0.0` yaw rad
  - `data[1] = 0.0` pitch rad
- Parameters:
  - `mode`
  - `imu_topic`
  - `rate_hz`
  - `yaw_rad`
  - `pitch_rad`
  - `publish_log_period_s`
- Only `mode="static"` is accepted.
- Any video-derived mode now throws a clear runtime error.

Reason:

- Video-derived fake IMU creates hidden, time-varying TF drift.
- For debugging targeting, a fake pose source must be simple, still, and
  observable.

### `src/launch_pkg/launch/debug_targeting.launch.py`

Simplified targeting debug launch.

Removed:

- `fake_imu_mode`
- `fake_imu_yaw_rad`
- `fake_imu_pitch_rad`
- `fake_imu_video_hfov_deg`
- `fake_imu_video_vfov_deg`
- `fake_imu_video_smoothing_alpha`
- `fake_imu_video_process_every_n`
- Debug override of `target_classes`
- Debug override of `confirm_frames`
- Debug override of `switch_cooldown`
- Debug override of `enable_tracking_switch`
- Debug override of `tracking_switch_range_ratio`
- Debug override of `cmd_smooth_alpha`

Kept:

- Video/camera pipeline.
- YOLO keypoint detector path.
- Optional fake `/micro_imu`.

New behavior:

- Fake IMU launch always uses:
  - `mode: static`
  - `yaw_rad: 0.0`
  - `pitch_rad: 0.0`
- Tracker state-machine values come from the YAML config.

Reason:

- The old launch was hiding real association and transform problems by forcing
  unsafe debug-only behavior.

### `src/launch_pkg/package.xml`

Removed:

- `<exec_depend>auto_aim_targeting</exec_depend>`

Reason:

- `auto_aim_targeting` source package is no longer present.

### `src/auto_aim/src/auto_aim_node.cpp`

Main runtime changes.

Added pose freshness handling:

- Added parameter:
  - `micro_imu.stale_threshold_s`
- Tracks:
  - latest raw micro yaw/pitch
  - latest converted internal yaw/pitch
  - last `/micro_imu` receive time
- Populates debug fields:
  - `pose_source`
  - `pose_present`
  - `pose_fresh`
  - `pose_age_s`
  - `imu_yaw`
  - `imu_pitch`

Changed stale/no-pose behavior:

- If `/micro_imu` is missing or stale:
  - tracker is updated with no detections
  - markers are cleared
  - debug frame is still published
  - fire gate reports the exact pose blocker

Added helper functions:

- `fillPoseDebug`
- `populateTrackerDebug`
- `finishDebugFrame`

Reason:

- Pose freshness is part of the targeting math. If it is stale, transformed
  armor pose and aim command are not trustworthy.

Changed command hold behavior:

- Before:
  - no target published yaw `0`, pitch `0`
- After:
  - valid target publishes target yaw/pitch
  - no valid target publishes current pose hold if pose is fresh
  - if pose is stale but a previous command exists, it holds the previous
    command
  - if no safe absolute hold exists yet, it suppresses the gimbal command

Added command debug:

- `cmd_hold_active`
- `cmd_published`
- `cmd_yaw_pre_smooth`
- `cmd_pitch_pre_smooth`
- `cmd_yaw_published`
- `cmd_pitch_published`
- `aim_rel_yaw`
- `aim_rel_pitch`

Reason:

- Absolute zero is a real destination. It must not be used as a generic hold.

Added aim/fire diagnostics:

- `aim_fire_margin`
- `aim_anti_gyro_active`
- `aim_anti_gyro_residual`
- `aim_cam_yaw`
- `aim_cam_pitch`
- `aim_cam_total_angle`

Reason:

- These fields make it possible to tell whether fire is blocked by tracking,
  pose freshness, ballistic planning, planner margin, or off-axis camera angle.

### `src/auto_aim/include/auto_aim/fire_gate.hpp`

Added fire blockers:

- `TEMP_LOST`
- `STALE_POSE`
- `NO_POSE_SOURCE`

Added fire inputs:

- `temp_lost`
- `pose_available`
- `pose_fresh`

Added config:

- `enable_pose_gate`

Reason:

- Fire gating needs to report pose failures explicitly. `/micro_imu` is not
  optional for this transform path.

### `src/auto_aim/src/fire_gate.cpp`

Changed evaluation order:

1. Pose source available.
2. Pose source fresh.
3. Tracker is in a firing-capable state.
4. Not `TEMP_LOST`.
5. Target valid.
6. Ballistics valid.
7. Range valid.
8. Optional stale measurement gate.
9. Optional anti-gyro timing gate.
10. Planner margin / off-axis / smoothing gates.

Reason:

- The first hard blocker should identify the real unsafe condition.

### `src/auto_aim/msg/AutoAimDebug.msg`

Added fields:

- Pose:
  - `pose_source`
  - `pose_present`
  - `pose_fresh`
  - `pose_age_s`
  - `imu_yaw`
  - `imu_pitch`
- Aim:
  - `aim_rel_yaw`
  - `aim_rel_pitch`
  - `aim_fire_margin`
  - `aim_anti_gyro_active`
  - `aim_anti_gyro_residual`
  - `aim_cam_yaw`
  - `aim_cam_pitch`
  - `aim_cam_total_angle`
- Command:
  - `cmd_published`
  - `cmd_hold_active`
- Fire blocker enum:
  - `FIRE_TEMP_LOST`
  - `FIRE_STALE_POSE`
  - `FIRE_NO_POSE_SOURCE`

Reason:

- Real robot testing needs per-frame structured diagnostics, not only logs.

### `src/auto_aim/include/auto_aim/debug_frame.hpp`

Added matching internal debug fields for the new `AutoAimDebug.msg` fields.

Reason:

- `DebugFrame` is the node-local staging struct used before publishing the ROS
  debug message.

### `src/auto_aim/src/debug_publisher.cpp`

Added publishing support for all new debug fields.

Updated blocker names:

- `TEMP_LOST`
- `STALE_POSE`
- `NO_POSE_SOURCE`

Reason:

- The fire blocker histogram must report the same names as the message enum.

### `src/auto_aim/include/auto_aim/tracker.hpp`

Added:

- `AimResult::fire_margin`

Reason:

- The planner margin was already used internally to decide fire eligibility,
  but it was not visible in debug output.

### `src/auto_aim/src/tracker.cpp`

Changed `ekfMahalanobis`:

- Association gating now uses the same obliquity-aware covariance inflation
  strategy as the EKF update path.

Reason:

- The old association gate could reject oblique measurements even though the
  update stage would have handled them by increasing measurement covariance.

Changed initial/switch bookkeeping:

- On accepted initialization or switch:
  - `last_assigned_count = 1`
  - `last_meas_quality = MQ_ACCEPTED`
  - `match_count = 1`
  - `miss_count = 0`
  - `last_match_time_s` is updated when wall time is available

Reason:

- Debug fields should reflect accepted measurements immediately.

Changed unmatched Mahalanobis reporting:

- `last_mahalanobis_` now records the best finite seen value when unmatched,
  instead of falling back to zero.

Reason:

- A zero Mahalanobis on rejection is misleading.

Changed `computeAim`:

- Fills `aim.fire_margin`.

Reason:

- Fire blocker `MARGIN_NEGATIVE` can now be diagnosed numerically.

### `src/auto_aim/config/params_realsense_16.yaml`

Added:

- `micro_imu.stale_threshold_s: 0.25`
- `fire.enable_pose_gate: true`

Reason:

- RealSense targeting uses `/micro_imu` as the pose source for frame transform.
  Fire should not be allowed if that pose source is stale or absent.

### `src/auto_aim/config/params_zed_64.yaml`

Added:

- `micro_imu.stale_threshold_s: 0.25`
- `fire.enable_pose_gate: true`

Reason:

- Same as RealSense config.

### `media/docs/gimbal_command_contract.md`

Updated command contract:

- No-target behavior now publishes a current-pose hold destination when safe.
- It does not publish yaw/pitch zero as a generic hold.
- If no pose sample and no previous command exist, command publishing is
  suppressed.

Reason:

- This is a firmware-facing contract. The hold behavior must be explicit.

### `media/docs/auto_aim_debugging_guide.md`

Added fire blocker documentation for:

- `TEMP_LOST`
- `STALE_POSE`
- `NO_POSE_SOURCE`

Reason:

- These blockers now appear in `/auto_aim/debug` and fire histograms.

## Behavior After Refactor

### Fake IMU

The only remaining fake IMU mode is static.

Expected fake IMU message:

```text
/micro_imu std_msgs/msg/Float32MultiArray
data[0] = 0.0  # yaw rad
data[1] = 0.0  # pitch rad
```

It is for bench/video tests only. It must not be treated as real gimbal
feedback.

### Command Publishing

Valid target:

- Publishes absolute target yaw/pitch in microcontroller convention.
- Fire flag is set only when FireGate allows fire.

No valid target:

- Holds latest fresh `/micro_imu` yaw/pitch.
- If pose is stale but a previous command exists, holds previous command.
- If no pose and no previous command exist, suppresses command.

### Fire Gate

Fire can now be blocked explicitly by:

- `NO_POSE_SOURCE`
- `STALE_POSE`
- `NOT_TRACKING`
- `TEMP_LOST`
- `INVALID_TARGET`
- `INVALID_BALLISTIC`
- `OUT_OF_RANGE`
- `STALE_MEASUREMENT`
- `ANTI_GYRO_TIMING`
- `OFF_AXIS`
- `SMOOTHING_LAG`
- `MARGIN_NEGATIVE`

## Validation Performed

Commands that passed:

```bash
python3 -m py_compile \
  src/auto_aim/scripts/fake_micro_imu_node.py \
  src/launch_pkg/launch/debug_targeting.launch.py
```

```bash
/opt/ros/humble/bin/ament_flake8 \
  src/launch_pkg/launch/debug_targeting.launch.py
```

```bash
colcon build --packages-select auto_aim launch_pkg \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

```bash
colcon test --packages-select auto_aim --event-handlers console_direct+
```

Known unrelated test failure:

```bash
colcon test --packages-select launch_pkg --event-handlers console_direct+
```

This still fails `flake8` because of pre-existing lint errors in other launch
files, including:

- `realsense_only_det_image_only.launch.py`
- `realsense_only_det_rtdetr.launch.py`
- `realsense_only_det_flip_adam.launch.py`
- `debug.launch.py`
- `test_realsense_optimized_launcher.launch.py`
- `realsense_only_det.launch.py`

The edited `debug_targeting.launch.py` passes focused flake8.

## How To Run

Bench/video test with static fake IMU:

```bash
source install/setup.bash
ros2 launch launch_pkg debug_targeting.launch.py use_fake_micro_imu:=true
```

Robot test with real microcontroller feedback:

```bash
source install/setup.bash
ros2 launch launch_pkg debug_targeting.launch.py use_fake_micro_imu:=false
```

## Topics To Inspect

Primary debug:

```bash
ros2 topic echo /auto_aim/debug
```

Gimbal command:

```bash
ros2 topic echo /tracker/cmd_gimbal
```

Aim overlay:

```bash
ros2 topic echo /tracker/aim_pixels
```

Fake or real micro IMU:

```bash
ros2 topic echo /micro_imu
```

Detector keypoints:

```bash
ros2 topic echo /detector/armors_keypoints
```

## Expected Values For A Still Target

Pose:

- `pose_present: true`
- `pose_fresh: true`
- `pose_age_s < micro_imu.stale_threshold_s`

Detection/PnP:

- `raw_kp_detection_count > 0`
- `detection_present: true`
- `kp_reject_reason: KP_OK`
- `pnp_ok: true`
- `pnp_reproj_err` roughly under `1-2 px` for good detections
- `armors_passed_to_tracker_count > 0`

Tracker:

- `tracker_state` should progress from `DETECTING` to `TRACKING`
- `ekf_match_count` should grow
- `ekf_miss_count` should stay near zero
- `match_reject_reason` should be `accepted` on stable frames
- `best_match_mahalanobis` should remain below `maha_threshold`

Command:

- `cmd_published: true`
- `cmd_hold_active: false` while target is valid
- `cmd_yaw_published` and `cmd_pitch_published` should not jump to zero unless
  zero is the actual target or hold pose.

Fire:

- `fire_blocker` should become `FIRE_ALLOWED` only when:
  - tracker is `TRACKING`
  - pose is fresh
  - target is valid
  - range is valid
  - `aim_cam_total_angle < angular_window`
  - `aim_fire_margin >= 0`

With static fake IMU:

- `OFF_AXIS` is expected if the target is not already centered in the camera.
- Static fake IMU does not prove real gimbal convergence.

## Remaining Risks And Tuning Items

Still needs real robot validation:

- `gimbal.yaw_sign`
- `gimbal.pitch_sign`
- firmware interpretation of absolute yaw/pitch commands
- camera mounting pitch and frame convention
- `bullet_speed`
- barrel offsets
- `angular_window`
- `target_classes`

Still possible failure modes:

- PnP small/large size ambiguity.
- Wrong target class or multiple enemy-like detections.
- Detector intermittently publishing zero valid keypoints.
- Real `/micro_imu` dropping or becoming stale.
- Ballistic model mismatch at close range.
- Camera intrinsics mismatch with actual image stream.
- Gimbal controller lag larger than prediction assumptions.

Useful next plots for persistent instability:

- `tracker_state`
- `armors_passed_to_tracker_count`
- `pnp_reproj_err`
- `pnp_is_large`
- `pnp_size_margin`
- `odom_x`, `odom_y`, `odom_z`, `odom_yaw`
- `best_match_mahalanobis`
- `best_match_position_diff`
- `best_match_yaw_diff`
- `match_reject_reason`
- `aim_cam_total_angle`
- `aim_fire_margin`
- `cmd_yaw_published`
- `cmd_pitch_published`
- `cmd_hold_active`
- `cmd_published`
- `fire_blocker`
- `pose_age_s`

## Files Not Touched By This Refactor

There were pre-existing unrelated workspace changes:

- deleted files under `media/`
- untracked `media/old/`

Those were left alone.

## Second Critical Patch: 2026-05-13

This follow-up patch kept the first refactor intact and targeted remaining
high-impact causes of unstable aim, tracker oscillation, false `OFF_AXIS`
blocking, unsafe `TEMP_LOST` commands, and low-reprojection PnP pose jumps.

### Fire alignment source

Added:

- `fire.alignment_source: "camera_angle" | "relative_error" | "disabled"`
- `fire_alignment_source` in `/auto_aim/debug`
- `fire_alignment_error` in `/auto_aim/debug`

Behavior:

- `camera_angle` gates fire on `aim_cam_total_angle`.
- `relative_error` gates fire on `sqrt(aim.rel_yaw^2 + aim.rel_pitch^2)`.
- `disabled` disables only the alignment gate; all debug angles still publish.

Real robot YAML keeps the safe default:

```yaml
"fire.alignment_source": "camera_angle"
```

`debug_targeting.launch.py` defaults to:

```text
fire_alignment_source:=disabled
```

Reason:

- Static fake IMU cannot prove physical gimbal convergence, so using camera
  centering as a hard fire gate in video/bench mode can permanently block fire.

### TEMP_LOST command policy

Added:

- `cmd.temp_lost_coast_max_s`
- `cmd_coast_active`
- `tracker_fresh_enough_for_command`
- `coast_age_s`

Behavior:

- `TRACKING`: publish normal target command.
- `TEMP_LOST`: fire is blocked, but command may coast for a short bounded
  window.
- After the coast window, command switches to hold.
- `DETECTING`, `LOST`, invalid target: hold.
- Hold still never means absolute zero unless zero is the actual hold pose.

Default:

```yaml
"cmd.temp_lost_coast_max_s": 0.08
```

### Yaw unwrap side effects

Changed tracker yaw handling:

- Replaced side-effecting unwrap during candidate evaluation with
  `previewUnwrapYaw`.
- Added `commitYaw`.
- Rejected measurements no longer mutate yaw history.
- `last_yaw_` changes only on accepted measurements or tracker initialization.

Reason:

- A rejected candidate must not corrupt yaw continuity for future frames.

### Face-jump association fallback

Added tracker parameters:

```yaml
face_jump_max_match_dist_ratio: 1.5
face_jump_min_yaw: 0.78539816339
face_jump_max_yaw: 2.35619449019
```

Behavior:

- Normal Mahalanobis association still runs first.
- If no normal candidate passes, the tracker checks for a plausible face jump:
  same class, close position, and yaw jump roughly in the 45-135 degree range.
- A valid face-jump candidate is routed to `handleArmorJump()` instead of being
  counted as a miss.

New debug reasons include:

- `rejected_maha_but_face_jump_candidate`
- `accepted_face_jump_after_maha`

Reason:

- A different visible armor face can produce a large yaw jump while the
  position remains physically consistent. This is an association-model issue,
  not a reason to loosen `maha_threshold` globally.

### PnP size hysteresis

Added:

- `pnp.size_switch_margin_px`
- `pnp_size_hysteresis_kept`
- launch arguments:
  - `pnp_force_armor_size`
  - `pnp_size_switch_margin_px`

Behavior:

- If `pnp.force_armor_size == "auto"` and small/large reprojection errors are
  too close, the node keeps the previous armor size choice.
- Operators can still force `small` or `large` from launch for diagnosis.

Reason:

- Small armor width and large armor width imply different 3D scales. Planar PnP
  can give low reprojection error for both, so lower reprojection error alone is
  not always enough to prevent pose jumps.

### Multi-blocker fire debug

Added:

- `fire_blocker_mask`
- `fire_blockers_active`

Behavior:

- `fire_blocker` remains the primary blocker.
- The mask/string show every active blocker in the frame, so `NOT_TRACKING` or
  `TEMP_LOST` no longer hide secondary causes such as `OFF_AXIS`,
  `MARGIN_NEGATIVE`, `OUT_OF_RANGE`, `STALE_POSE`, or smoothing lag.

### Static fake IMU arguments restored

`debug_targeting.launch.py` remains static-only, but now exposes:

- `fake_imu_yaw_rad`
- `fake_imu_pitch_rad`
- `fake_imu_rate_hz`

Video-motion fake IMU remains removed.
