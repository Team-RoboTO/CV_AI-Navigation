# Auto-aim Debugging Guide

How to diagnose static jitter, pitch bias, target switching, and fire-gate
behaviour. Use this guide before tuning any parameter.

## Setup

Park the robot, point the gimbal at a fixed armor target at a known
distance (1 m, 3 m, 5 m). The robot must be still — no motion at the base.

The current RealSense + YOLOv26-pose + auto-aim bring-up is:

```
source install/setup.bash
ros2 launch launch_pkg debug_targeting.launch.py
```

`debug_targeting.launch.py` publishes a fake `/micro_imu` by default for
bench Jetsons that do not have the gimbal microcontroller attached:

```
/micro_imu = std_msgs/Float32MultiArray {data: [0.0, 0.0]}
```

This lets PnP, tracking, debug overlays, and command publishing run with a
fixed gimbal pose. On the real robot, disable it so the firmware is the only
publisher:

```
ros2 launch launch_pkg debug_targeting.launch.py use_fake_micro_imu:=false
```

That launch file expects the TensorRT engine here:

```
src/launch_pkg/resources/yolov26_keypoints.engine
```

After rebuilding `launch_pkg`, the default launch argument resolves through
the installed package share directory:

```
install/launch_pkg/share/launch_pkg/resources/yolov26_keypoints.engine
```

The detector executable is the Python script installed by `auto_aim`:

```
auto_aim yolo26_pose_realsense_node.py
```

It requires TensorRT Python and CuPy at runtime. A fast launch-side sanity
check is:

```
ros2 pkg executables auto_aim
ros2 launch launch_pkg debug_targeting.launch.py --show-args
```

Record at least 30 seconds:

```
ros2 bag record \
  /camera/camera/color/image_raw \
  /camera/camera/color/camera_info \
  /detector/armors \
  /detector/armors_keypoints \
  /yolo/debug_image \
  /camera_info \
  /micro_imu \
  /tracker/cmd_gimbal \
  /cmd_vel_AI \
  /tracker/aim_pixels \
  /tracker/marker \
  /auto_aim/debug
```

If `/auto_aim/debug` is not yet wired (pre-P1), use console logs:

```
ros2 launch auto_aim auto_aim_realsense_16.launch.py 2>&1 | tee aim.log
```

## Startup Failures

Check these before tuning:

| Symptom | Likely cause | Fix |
|---------|--------------|-----|
| `package 'auto_aim' not found` | Workspace was not sourced or `auto_aim` was not built | `colcon build --symlink-install --packages-select auto_aim launch_pkg`, then `source install/setup.bash` |
| Detector exits before ready log | Missing `cupy`, TensorRT Python, or engine file | Install the Python dependency and place `yolov26_keypoints.engine` in `launch_pkg/resources` |
| `Waiting for /micro_imu` | Firmware yaw/pitch topic is absent and fake micro is off | For bench launch, set `use_fake_micro_imu:=true`; on robot, publish `std_msgs/Float32MultiArray [yaw_rad, pitch_rad]` from firmware |
| No PnP output | Camera info has not arrived | Confirm `/camera/camera/color/camera_info` is remapped into `/camera_info` |
| Detections exist but no tracking | Class filter does not include the detected class | Check `target_classes`; debug launch currently keeps `["0", "2", "3"]` live |

## What to inspect (in order)

The pipeline has six stages where jitter or bias can enter. Inspect them
in order — do not jump to the EKF if PnP is already noisy.

### 1. Detection

Current `debug_targeting.launch.py` uses the YOLOv26-pose detector in
`auto_aim/scripts/yolo26_pose_realsense_node.py`.

* Primary topic: `/detector/armors_keypoints`
  (`auto_aim/ArmorKeypointArray`).
* Compatibility topic: `/detector/armors`
  (`vision_msgs/Detection2DArray`).
* Debug image: `/yolo/debug_image` when `publish_debug_every > 0`.
* Field: `Detection: cx=... cy=... w=... h=...` console line, plus
  `/auto_aim/debug` fields `bbox_*`, `detect_confidence`, and `class_id`.
* Bbox center should not drift >2 px frame-to-frame on a static target.
* Bbox size should not breathe more than ~5%.
* Keypoint corners should stay ordered TL, TR, BR, BL and should not jump
  between armor lights.
* If bbox or keypoints jitter, the detector is upstream of auto-aim and must
  be fixed there. Check the engine, confidence threshold, NMS, keypoint score
  threshold, lighting, exposure, and model weights.

### 2. PnP

After P1: `/auto_aim/debug` includes `pnp_reproj_err`, `pnp_image_points`,
and `pnp_tvec_*`.

* Reproj err normalized by bbox diagonal should be `< 0.05` for a clean
  static frame. Higher usually means the four YOLOv26 corner keypoints are
  noisy, out of order, or inconsistent with the real armor geometry.
* `pnp.tvec.z` (depth in camera frame) should be steady within ~1% on a
  static target.
* If reproj err is OK but tvec jitters, inspect the raw keypoints and camera
  intrinsics before touching EKF parameters.

### 3. Frame transform

After P1: `/auto_aim/debug` includes `transform.p_odom`.

* `p_odom` should be steady within ~1 cm on a static target.
* If `p_odom` jitters more than `tvec`, then `imu_rotation_` is changing,
  which means `/micro_imu` is reporting a moving gimbal (real or noise).
* Confirm `/micro_imu` is not noisy by plotting `data[0]` and `data[1]`
  over a static interval.

### 4. EKF

After P1: `/auto_aim/debug` includes `ekf.state`, `ekf.innovation`,
`ekf.maha`.

* Innovation norm should be `< 2 sigma_pos` most frames.
* Mahalanobis distance should be `< maha_threshold` (default 13.3).
* If innovation is large but maha is small, the measurement noise model
  (`r_pos_*`, `r_yaw_*`) may be over-confident: increase the slope.
* If maha is large persistently, the tracker is drifting from the target
  and may be locked onto a different object — check `target_id`.

### 5. Aim planner

After P1: `/auto_aim/debug` includes `aim.face_index`, `aim.target_xyz`,
`aim.abs_yaw`, `aim.abs_pitch`, `aim.distance`.

* On a static target, `aim.abs_yaw` and `aim.abs_pitch` should match the
  robot's actual yaw/pitch within calibration tolerance.
* If `abs_pitch` is consistently biased, the calibration is wrong (see
  `docs/calibration_guide.md`) — do NOT cover this with `pitch_offset_deg`.
* `face_index` should be stable (e.g. always `0`) on a static target.
  If it flips between adjacent indices, the angular-margin tie-break is
  too tight — file a calibration issue.

### 6. Command

After P1: `/auto_aim/debug` includes `cmd.yaw_pre_smooth`,
`cmd.pitch_pre_smooth`, `cmd.yaw_published`, `cmd.pitch_published`.

* The difference between `cmd.yaw_pre_smooth` and `cmd.yaw_published` is
  the EMA filter lag. A large persistent gap means smoothing alpha is too
  low for the target's motion or the rate is too low.
* On a static target, `cmd.yaw_published` should converge to
  `cmd.yaw_pre_smooth` within a few frames.

## Fire blocker reasons (P6+)

When `/auto_aim/debug.fire_allowed == false`, `fire_blocker` is one of:

| Reason             | Meaning                                                  |
|--------------------|----------------------------------------------------------|
| `NOT_TRACKING`     | Tracker not in `TRACKING` state                          |
| `OUT_OF_RANGE`     | Range is below `min_fire_dist` or above `max_fire_dist`  |
| `MARGIN_NEGATIVE`  | Selected face is outside the angular window              |
| `OFF_AXIS`         | Camera-frame angle to face exceeds `angular_window`      |
| `INVALID_TARGET`   | Aim planner returned `target_valid == false`             |
| `INVALID_BALLISTIC`| Ballistic solver failed (range too small or bad geometry)|
| `SMOOTHING_LAG`    | Smoothed cmd not yet close to ballistic ray (diagnostic, off by default) |
| `STALE_MEASUREMENT`| Last detection too old                                   |
| `ANTI_GYRO_TIMING` | Anti-gyro impact-time residual outside tolerance (P9+)   |

A blocker histogram printed every 10 s gives an at-a-glance view of why
fire is being denied. If `MARGIN_NEGATIVE` dominates, the issue is
geometry or planning.

### When (not) to enable `SMOOTHING_LAG`

`fire.enable_smoothing_gate` is **off by default** and should stay off in
competition. EMA smoothing of the absolute command introduces a phase lag
that is roughly constant against a steady-moving target — order
`v_target * (1 - alpha) / (alpha * f)` radians, e.g. ≈ 90 mrad against a
10°/s yaw at α=0.4 and 30 Hz. The gate would therefore block fire for as
long as the target keeps moving, not because aim is bad but because the
filter has not caught up.

Enable the gate only when:

* You are doing acquisition tuning on a stationary rig where the lag
  *should* converge to zero, and you want the gate to expose any
  remaining steady-state error.
* You are running static QA (the standard deviation tests in this guide).

In a moving-target scenario rely instead on the geometric margin
(`MARGIN_NEGATIVE` / `OFF_AXIS`) and ballistic validity. The
`smoothing_lag_rad` value continues to be published in `/auto_aim/debug`
regardless, so you can plot it without enabling the gate.

## Diagnosing pitch bias

1. Park the robot at 3 m from a static armor.
2. Set bullet speed in launch to a measured value (NOT the default).
3. Capture commanded pitch from `/tracker/cmd_gimbal angular.y`.
4. Fire one shot. Measure vertical error of the impact relative to the
   armor center.
5. If the impact is high, the commanded pitch was too high: either the
   ballistic model is under-compensating gravity (drag effect, slower
   actual bullet) or the barrel offset z is wrong.
6. Fix at the source: measure bullet speed and barrel offset z. Do not
   compensate with `pitch_offset_deg`.

## Diagnosing target-switching issues

* Field: `tracker.state`, `tracker.target_id` (P1+).
* If state oscillates between `TRACKING` and `DETECTING`, the
  `confirm_frames` is too high or detections are intermittent.
* If `target_id` flips between two robots, raise `switch_range_ratio`
  (closer to 1.0 means more aggressive switching) or lengthen
  `switch_cooldown`.
* If the tracker stays on a far target while a near one appears, lower
  `switch_range_ratio` (e.g. 0.7) so a closer target wins.

## New debug fields (P3+ / P5+ / P7+ / P8+)

Beyond the original `bbox_*`, `pnp_*`, `ekf_state`, `aim_*`, `cmd_*`,
`fire_*` fields, `/auto_aim/debug` now carries:

| Field                      | What it tells you                                                |
|----------------------------|------------------------------------------------------------------|
| `pnp_reject_reason`        | Why a PnP attempt failed (see `PNP_*` enum). 0 = OK              |
| `ekf_innovation_pos_norm`  | sqrt(yx²+yy²+yz²) [m]. Use this instead of the legacy mixed norm |
| `ekf_innovation_yaw_abs`   | \|y_yaw\| [rad]                                                    |
| `ekf_q_pos_eff`            | Effective `q_pos` at predict time (after adaptive Q schedule)    |
| `ekf_q_yaw_eff`            | Effective `q_yaw` at predict time                                |
| `ekf_r_pos_eff`            | Effective sqrt(R_pos) at the most recent update [m]              |
| `ekf_r_yaw_eff`            | Effective sqrt(R_yaw) at the most recent update [rad]            |
| `ekf_pos_sigma`            | sqrt(P00+P22+P44), a quick "tracker uncertainty" proxy [m]       |
| `ekf_yaw_sigma`            | sqrt(P66) [rad]                                                  |
| `ekf_match_count`          | Consecutive matched frames                                       |
| `ekf_miss_count`           | Consecutive missed frames                                        |
| `ekf_measurement_age_s`    | Wall-clock age of the last accepted measurement                  |
| `ekf_measurement_quality`  | NONE / ACCEPTED / DEGRADED / REJECTED on the last frame          |
| `latency_estimate_ema_s`   | EMA of `latency_total_s`                                         |
| `pred_lead_measured_s`     | Latency contribution to `pred_t` (use_measured_latency)          |
| `pred_lead_gimbal_s`       | Fixed actuator delay (use_measured_latency)                      |
| `pred_lead_time_bias_s`    | Legacy `cfg.time_bias` contribution                              |
| `pred_lead_ema_s`          | EMA-smoother delay compensation contribution                     |
| `pred_flight_time_s`       | Ballistic flight time                                            |
| `pred_t_total_s`           | Final lead committed to the planner                              |
| `smoothing_lag_rad`        | sqrt(yaw_lag² + pitch_lag²); always logged                       |

### Recommended enable order (do not turn everything on at once)

The flags below default OFF. Enable one at a time, validate on a static
rig and a moving target, and only then proceed to the next. Track
`/auto_aim/debug` while you flip flags.

1. `enable_pnp_refine` — adds an LM polish to the IPPE solution.
   Validate `pnp_reproj_err_norm` drops on average and that `pnp_ok`
   stays at 100% on the static rig.
2. `enable_pnp_health_gate` — turns on the normalized reproj +
   depth-band rejection. Watch `pnp_reject_reason` histograms; if
   `REPROJ_NORM` dominates on real targets, raise
   `pnp_max_reproj_err_norm`. Conservative defaults: 0.10, depth in
   [0.10, 12.0] m.
3. `enable_adaptive_r` — makes R inflate when reprojection looks bad
   or detector confidence is low. Watch `ekf_r_pos_eff` against
   `pnp_reproj_err_norm`. The scaler should rise smoothly, not
   spike.
4. `enable_adaptive_q` — reduces Q in the stationary regime, boosts
   yaw Q under spin. Watch `ekf_q_pos_eff` and `ekf_q_yaw_eff`.
   A clean static target should converge `q_pos_eff` toward
   `q_pos * (1 - q_reduction_max)` clamped to `q_pos_eff_min`.
5. `use_measured_latency` — replaces `time_bias` with the live latency
   EMA. Verify `latency_estimate_ema_s` is stable for a few seconds
   first.
6. `enable_ema_delay_compensation` — adds the steady-state EMA
   smoother delay to the prediction lead. Only useful when
   `cmd_smooth_alpha < 1.0`. Confirm `pred_lead_ema_s` matches the
   theoretical `dt * (1 - alpha) / alpha`.
7. `enable_anti_gyro` — only when you have spinning-target bag data.
8. `fire.enable_smoothing_gate` — diagnostic only, keep off in
   competition (see "When (not) to enable SMOOTHING_LAG" above).

## Diagnosing static jitter

A clean static rig should produce:

* `/tracker/cmd_gimbal angular.z` standard deviation ≤ 0.005 rad
  (≈ 0.3°) at 3 m on a static target.
* `/tracker/cmd_gimbal angular.y` standard deviation in the same range.

To measure:

```
ros2 topic echo --field angular.z /tracker/cmd_gimbal | head -1000 > yaw.csv
python -c "import numpy, sys; a=numpy.loadtxt('yaw.csv'); print(a.std())"
```

If standard deviation is significantly higher, walk back through the six
stages above. Do not raise smoothing alpha to hide jitter — find the source.
