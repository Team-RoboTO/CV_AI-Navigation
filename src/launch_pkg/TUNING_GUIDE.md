# Auto-Aim Stack Parameter & Tuning Guide

This guide is aligned to the current `debug.launch.py` / `debug_targeting.launch.py` wiring and to the targeting behavior visible in the source files.

It focuses on two goals:
1. **Understanding what each parameter really does in this stack**
2. **Giving a practical tuning workflow on the physical robot**

---

## Table of Contents
1. [Debug Tools & Topics](#1-debug-tools--topics)
2. [RealSense Driver, IMU, and Vision Pipeline](#2-realsense-driver-imu-and-vision-pipeline)
3. [Armor Tracker (PnP + EKF) Tuning](#3-armor-tracker-pnp--ekf-tuning)
4. [Trajectory Solver (Ballistics + Fire Gate) Tuning](#4-trajectory-solver-ballistics--fire-gate-tuning)
5. [Recommended Tuning Workflow](#5-recommended-tuning-workflow)
6. [Common Problems & Solutions](#6-common-problems--solutions)
7. [Important Notes / Mismatches](#7-important-notes--mismatches)

---

## 1. Debug Tools & Topics

Before changing any parameter, make sure you can observe the full pipeline.

### Tracker / Detection Topics

| Topic | Type | Why it matters |
|---|---|---|
| `/tracker/info` | `TrackerInfo` | Best quick EKF debug topic. Contains `position_diff`, `yaw_diff`, `face_angle`, and the effective adaptive damping values. |
| `/tracker/target` | `Target` | Legacy single-target output. Good for quickly checking the currently selected best target. |
| `/tracker/targets` | `Targets` | The real multi-target output used by the trajectory solver. Use this when debugging target selection and anti-spin logic. |
| `/tracker/marker` | `MarkerArray` | RViz visualization of tracked center, velocities, and armor geometry. |
| `/detections_output` | `Detection2DArray` | Raw YOLO detections after decoder. Useful to separate detector issues from tracker issues. |
| `/detections_output/optimal_target` | `Detection2D` | The bbox corresponding to the currently selected best target. Useful when checking whether target selection matches what you expect visually. |

### Command / Trajectory Topics

| Topic | Type | Why it matters |
|---|---|---|
| `/tracker/cmd_gimbal` | `GimbalCmd` | Main command output: `pitch`, `yaw`, `distance`, `fire_cmd`. |
| `/cmd_vel` | `Twist` | Deprecated compatibility output. It no longer carries fire state; use `/tracker/cmd_gimbal` for control. |
| `/trajectory/marker` | `Marker` | Shows the predicted impact / aim point in RViz. |

### Useful terminal commands

```bash
# Best quick look at the final selected target
ros2 topic echo /tracker/target

# Full list of tracked targets
ros2 topic echo /tracker/targets

# Final gimbal command actually published by the solver
ros2 topic echo /tracker/cmd_gimbal

# Tracker debug residuals and damping
ros2 topic echo /tracker/info

# Check the bbox selected as the current best target
ros2 topic echo /detections_output/optimal_target

# Example runtime tuning
ros2 param set /auto_aim_targeting ekf.sigma2_q_xyz 2.0
```

---

## 2. RealSense Driver, IMU, and Vision Pipeline

## 2.1 RealSense driver

### `json_file_path`
Path to the RealSense Viewer preset file. Use it to lock camera ISP behavior (exposure, white balance, etc.) so your detector sees a more repeatable image.

### `publish_tf`
Set to `False` in this launch.

In this stack, the camera TF is not meant to be driven by the RealSense driver. The tracker builds and broadcasts the effective camera pose itself from the chosen `pose_source` (`micro_pose`, `camera_imu`, or `none`). If the camera driver also publishes TF, you risk competing transforms.

---

## 2.2 Madgwick IMU filter

### `use_mag`
Set to `False` here.

That means the filter relies on gyro + accel only, not on a magnetometer. This avoids magnetic interference from motors and wiring, but yaw can drift over time.

### `world_frame`
Set to `enu`.

This gives a standard sign convention for the fused IMU orientation:
- **E**ast
- **N**orth
- **U**p

This is useful because yaw sign mistakes often come from inconsistent frame conventions.

---

## 2.3 Image encoder

### `keep_aspect_ratio`
Set to `True`.

This is extremely important.

The input image is `640x480`, while the network input is `640x640`. With `keep_aspect_ratio=True`, the encoder **letterboxes** the image instead of stretching it. That preserves the shape of the armor, which is much better for both detection and downstream PnP.

The side effect is that the detector output lives in a padded coordinate system. That is exactly why the tracker has `bbox_padding_y`.

---

## 2.4 YOLO decoder

### `confidence_threshold`
Current launch value: **0.10**

This is intentionally permissive.
- Lower values detect more difficult / blurry targets but increase false positives.
- Higher values reduce ghosts but can lose weak detections during motion blur or long range.

### `nms_threshold`
Current launch value: **0.45**

This controls how aggressively overlapping detections are merged.
- Lower = stricter merging
- Higher = keeps more close detections alive

If two robots pass close to each other and detections collapse too often, this is one parameter worth checking.

---

## 2.5 Pose source modes

### `pose_source = micro_pose`
Uses `/micro_pose` as the orientation source.

In this stack, that means the node reads yaw and pitch from a `PoseStamped` message and then builds / broadcasts the camera pose from those values.

### `pose_source = camera_imu`
Uses the filtered IMU topic (`/camera/filtered_imu` in this launch) as the orientation source.

This is the current default in the launch file.

### `pose_source = none`
Software-only mode.

Use this only for debugging the pure vision pipeline. It is useful for bag replay or dry software tests, but not for evaluating true world-frame behavior on the real robot.

---

## 3. Armor Tracker (PnP + EKF) Tuning

## 3.1 Target filtering and measurement construction

### `target_classes`
Current launch value: `['3']`

This tells the tracker which detector classes are allowed to become targets.

Use this first when the system is tracking the wrong team / wrong color.

### `max_armor_distance`
Current launch value: **10.0 m**

Hard range cutoff for detections before they are accepted into tracking.

If you see far ghost detections, reduce this.

---

## 3.2 PnP-related parameters

### `light_ratio`
Current launch value: **0.85**

This is **not** a physical width/height validation threshold.

In this codebase, `light_ratio` is the factor used to **shrink the YOLO bbox inward** before constructing the four synthetic 2D points for PnP. Since the detector gives an axis-aligned bbox, the tracker approximates the armor light-bar corners by pulling the bbox edges inward.

Practical meaning:
- Lower `light_ratio` → tighter synthetic corner placement
- Higher `light_ratio` → corners closer to the full bbox edges

### Tuning hint
If the target distance is roughly correct but the yaw shows a consistent bias, `light_ratio` is one of the first parameters to test.

Move it in small steps, for example:
- `0.83`
- `0.85`
- `0.87`

Do not jump wildly.

### `bbox_padding_y`
Current launch value: **80.0 px**

This is a **letterbox compensation term**, not a generic “extra padding for blur”.

Because the encoder maps `640x480` into `640x640` with aspect ratio preserved, the image gets vertical black padding. The detector outputs bbox coordinates in that padded space, so the tracker subtracts `bbox_padding_y` to recover the original image coordinates before building the PnP measurement.

For the current encoder settings, `80` is exactly the expected value:

\[
(640 - 480) / 2 = 80
\]

Only change this if you change image or network dimensions, or disable aspect-ratio preservation.

### `pnp.max_reprojection_error`
Current launch value: **10.0 px**

This is the quality gate for PnP.

After estimating pose, the solver checks how well the chosen 3D model reprojects back onto the observed 2D points. If the average reprojection error is too high, the measurement is rejected.

Practical meaning:
- Lower value = stricter geometry filter, cleaner but more fragile
- Higher value = accepts noisier poses, more robust but can let bad geometry through

Useful range to test:
- `6.0` to `8.0` for stricter filtering
- `10.0` to `12.0` for more tolerance

If detections are clearly present but PnP-based armors disappear too often, try relaxing this slightly.

---

## 3.3 Tracker lifecycle and association gates

### `tracker.max_match_distance`
Current launch value: **0.30 m**

Main spatial gate for matching a prediction to a new armor measurement.

- Increase it if the tracker drops targets during fast motion or during abrupt target changes.
- Decrease it if the tracker starts stealing nearby robots or swapping too easily.

### `tracker.max_track_range`
Current launch value: **6.0 m**

Once the tracked target center goes beyond this range, the tracker is dropped.

Use this as the “real engagement tracking range”, not just raw detector range.

### `tracker.tracking_thres`
Current launch value: **4 frames**

Number of consecutive successful updates required before a tracker becomes fully confirmed.

Important nuance:
- `DETECTING` does **not** publish a shootable target (`tracking=false`)
- `TRACKING` does

So lowering this makes the system more responsive, but it also increases the chance of confirming noise.

### `tracker.lost_time_thres`
Current launch value: **0.50 s**

How long the tracker is allowed to coast without a valid measurement before it is considered lost.

Increase it for brief occlusions.
Decrease it if ghost tracks live too long.

### `tracker.max_trackers`
Current launch value: **5**

Hard cap on simultaneous live trackers.

Mainly useful for CPU / complexity control.

### `tracker.new_tracker_min_dist`
Current launch value: **0.45 m**

Minimum spatial gap required before spawning a new tracker near existing live ones.

Increase it if duplicate trackers spawn too easily.
Decrease it if nearby enemies fail to get separate tracks.

### `tracker.maha_match_threshold`
Current launch value: **13.3**

Mahalanobis gate for normal measurement fusion.

Lower = stricter
Higher = more tolerant

### `tracker.maha_jump_threshold`
Current launch value: **20.0**

More permissive Mahalanobis gate used when classifying / handling armor jumps.

If face switches are ignored too often during spin, this is one of the parameters to inspect.

---

## 3.4 Scalar radius / geometry adaptation parameters

These parameters control the parts of the tracker that estimate the robot’s orbit geometry.

### `tracker.initial_r1`, `tracker.initial_r2`
Current launch values:
- `initial_r1 = 0.22`
- `initial_r2 = 0.30`

These are the initial guesses for the two radius estimates used by the tracker.

If these are badly wrong, the tracker can still converge, but the early frames may behave worse.

### `tracker.r_kf_Q`, `tracker.r_kf_R`, `tracker.r_kf_P_init`
Current launch values:
- `Q = 3.3e-8`
- `R = 4.0e-4`
- `P_init = 6.4e-3`

These tune the scalar Kalman filters used for radius estimation.

Use them only after the main tracking is already healthy. They are second-order tuning knobs, not the first place to look.

### `tracker.r_adapt_max_dist`
Current launch value: **4.0 m**

Beyond this distance, radius adaptation is limited / paused because the measurement gets too noisy.

### `tracker.dz_adapt_alpha`
Current launch value: **0.05**

EMA factor for adapting the vertical offset `dz` between armor pairs.

Higher = adapts faster but noisier
Lower = steadier but slower

### `tracker.r_yaw_uncertainty_scale`
Current launch value: **50.0**

Inflates radius uncertainty when yaw is uncertain.

This is a guard against overtrusting geometry extracted from poor angular measurements.

### `tracker.v_yaw_max`
Current launch value: **5.0 rad/s**

Hard clamp on estimated spin rate.

If the tracker occasionally explodes to absurd spin values, this protects the rest of the stack.

---

## 3.5 EKF process model (Q and damping)

## `ekf.sigma2_q_xyz`
Current launch value: **1.0**

Translational process noise.

Lower value:
- trusts the motion model more
- gives smoother motion
- reacts less aggressively to measurements

Higher value:
- trusts measurements more
- reacts faster
- can become noisy / twitchy

### Tuning intuition
- If the filter lags behind real target motion, try increasing it.
- If the estimate chatters, try decreasing it.

## `ekf.sigma2_q_yaw`
Current launch value: **3.0**

Rotational process noise.

This is one of the most important parameters for spinning targets.

- Increase it if the filter is too slow to adapt to real spin changes.
- Decrease it if yaw becomes nervous or overreactive.

## `ekf.sigma2_q_r`
Current launch value: **1.0e-6**

Process noise for the radius state.

Usually leave this alone unless you are deeply reworking the radius model.

## `ekf.xyz_damping_alpha`, `ekf.yaw_damping_alpha`
Current launch values:
- `xyz_damping_alpha = 0.85`
- `yaw_damping_alpha = 0.85`

These are damping factors, not “responsiveness multipliers”.

Important direction:
- **Lower alpha = more damping = velocity dies faster**
- **Higher alpha = less damping = velocity persists longer**

So if yaw velocity does **not** die fast enough when the robot stops spinning, you generally want to **decrease** `yaw_damping_alpha`, not increase it.

## `ekf.coast_damping_factor`
Current launch value: **0.60**

Extra damping used during coasting / temporary loss.

Lower = more aggressive braking during blindness.

## `ekf.damping_innov_threshold`, `ekf.yaw_innov_threshold`
Current launch values:
- `0.05`
- `0.15`

These thresholds decide when the innovation is large enough to trigger stronger adaptive damping.

### `ekf.ref_frequency`
Current launch value: **30.0 Hz**

Reference frequency used to normalize damping with respect to frame rate.

### `ekf.accel_ema_alpha`
Current launch value: **0.30**

EMA smoothing factor for the acceleration estimate sent downstream in `Target`.

Higher = more responsive, noisier
Lower = smoother, laggier

---

## 3.6 EKF measurement model (R)

The measurement noise grows with distance and also with viewing geometry.

## `ekf.r_xyz_base`, `ekf.r_xyz_slope`
Current launch values:
- `r_xyz_base = 0.04`
- `r_xyz_slope = 0.03`

These are the base and distance-dependent terms for position measurement noise.

The key idea is:
- farther targets are measured less accurately
- therefore the filter should trust long-range PnP less

### Tuning intuition
- If long-range PnP is unstable, increase `r_xyz_slope`
- If the tracker ignores good short-range measurements too much, slightly reduce `r_xyz_base`

Do **not** make `r_xyz_base` unrealistically tiny. Values like `0.002` are generally too optimistic for this bbox-based PnP pipeline.

## `ekf.r_yaw_base`, `ekf.r_yaw_slope`
Current launch values:
- `r_yaw_base = 0.05`
- `r_yaw_slope = 0.002`

Equivalent yaw-noise model.

If yaw becomes too unstable at distance, these are good knobs to inspect.

## `ekf.r_yaw_angle_power`
Current launch value: **4.0**

Controls how strongly yaw distrust increases when the armor is seen obliquely.

Higher = more severe punishment for side views.

## `ekf.max_yaw_oblique_deg`
Current launch value: **65 deg**

Beyond this obliqueness threshold, yaw is considered too unreliable and is effectively de-weighted / bypassed.

This is very important for edge-on armor views.

## `ekf.secondary_face_fusion`
Current launch value: **True**

Enables the use of a second simultaneously visible armor face to improve state estimation.

Leave this on unless you have a concrete reason to disable it.

## `ekf.secondary_r_inflation`
Current launch value: **2.0**

Inflates the uncertainty of the secondary face update so it helps without dominating the primary face.

## `ekf.secondary_maha_threshold`
Current launch value: **13.3**

Mahalanobis gate for the secondary face fusion step.

---

## 4. Trajectory Solver (Ballistics + Fire Gate) Tuning

## 4.1 Ballistic core

### `bullet_speed`
Current launch value: **25.0 m/s**

This is one of the most important ballistic parameters.

It directly affects time of flight and vertical drop compensation.

### Tuning direction
- If shots land **below** the target, the solver is under-compensating the drop. Try **increasing** `bullet_speed`, or decreasing gravity / drag slightly.
- If shots land **above** the target, the solver is over-compensating the drop. Try **decreasing** `bullet_speed`, or increasing gravity / drag slightly.

Always prefer measuring the real muzzle speed with a chronograph instead of guessing.

### `gravity`
Current launch value: **9.8**

Normally this should stay physical.

Changing it is a last-resort compensation trick, not the first tuning choice.

### `linear_drag_coeff`, `quadratic_drag_coeff`, `use_quadratic_drag`
Current launch values:
- `linear_drag_coeff = 0.01`
- `quadratic_drag_coeff = 0.01`
- `use_quadratic_drag = False`

With the current launch, the system uses the **linear** drag path.

Practical guidance:
- if short range is fine but long range falls short, slightly increase drag or reduce bullet speed if the measured speed is lower than expected
- do not enable quadratic drag casually; only do it if you have data showing it improves your projectile model

### `gimbal_pitch_min`, `gimbal_pitch_max`
Current launch values:
- `-0.524 rad`
- `+0.524 rad`

These are hard mechanical pitch limits used by the ballistic solver.

If the real hardware cannot reach these angles, commands may look mathematically valid but still be physically wrong for the robot.

---

## 4.2 Latency and timing

### `time_bias`
Current launch value: **0.05 s**

This is the explicit forward prediction offset used to compensate system latency.

If the robot consistently shoots **behind** a laterally moving target, increase `time_bias`.
If it consistently shoots too far **in front**, decrease it.

### `time_bias_alpha`
Current launch value: **0.35**

EMA adaptation rate for measured latency.

Higher = adapts faster but more nervous
Lower = steadier but slower

### `gimbal_response_delay`
Current launch value: **0.0 s**

Extra delay term for actuator response.

If your gimbal is mechanically slow even when the perception pipeline is fast, this parameter can matter.

### `latency_gate_sigma`
Current launch value: **2.5**

Outlier rejection threshold for latency adaptation.

Larger = accepts more latency fluctuation samples
Smaller = more conservative

### `latency_warmup_samples`
Current launch value: **5**

Number of initial latency samples accepted before the outlier gate becomes more meaningful.

---

## 4.3 Fire gate and reachability

### `min_fire_dist`, `max_fire_dist`
Current launch values:
- `0.5 m`
- `10.0 m`

Hard engagement distance bounds.

### `angular_window`
Current launch value: **0.09 rad**

This is the base angular tolerance used by the fire gate.

Practical meaning:
- Larger = more willing to fire
- Smaller = stricter alignment requirement

### `angular_window_ref_dist`
Current launch value: **3.0 m**

Distance scaling reference for the angular firing window.

This helps avoid using the exact same tolerance at every range.

### `max_measurement_age`
Current launch value: **0.10 s**

If a target measurement is too old, the solver marks it stale and suppresses firing.

### `detector_stall_timeout`
Current launch value: **0.20 s**

If detector frames stop arriving, the targeting node watchdog publishes an explicit hold command instead of leaving the last command alive.

### `max_gimbal_yaw_rate`, `max_gimbal_pitch_rate`
Current launch values:
- `6.0 rad/s`
- `4.0 rad/s`

These limit whether the solver believes the gimbal can physically reach the required aim in time.

### `fire_yaw_tolerance`, `fire_pitch_tolerance`
Current launch values:
- `0.03 rad`
- `0.03 rad`

These are the final present-alignment gates. They answer a different question from slew reachability: is the gimbal aligned closely enough **right now** to allow `fire_cmd=true`?

### `max_cmd_angle`
Current launch value: **30.0 deg**

Hard clamp on the command sent out in one solver cycle.

This protects the lower layer from absurd spikes.

### `cmd_smooth_alpha`
Current launch value: **0.40**

EMA smoothing for the outgoing relative pitch / yaw command.

Higher = snappier, shakier
Lower = smoother, laggier

---

## 4.4 High-spin / indirect mode

### `indirect_vyaw_threshold`
Current launch value: **3.0 rad/s**

If the target spin rate exceeds this threshold, the solver can switch from direct tracking to indirect / predictive face timing.

### `indirect_timing_tolerance`
Current launch value: **0.02 s**

Allowed timing mismatch when trying to align bullet arrival with face arrival.

### `indirect_max_candidates`
Current launch value: **8**

Search depth for indirect candidate solutions.

### `oblique_exponent`
Current launch value: **2.0**

Penalty exponent used in the engagement scoring for low visibility / oblique views.

---

## 4.5 Engagement scoring weights

These parameters do not change the tracker itself. They change **which target / shot plan is preferred**.

Current launch values:
- `cost.range = 0.30`
- `cost.flight_time = 0.15`
- `cost.uncertainty = 0.35`
- `cost.slew = 0.25`
- `cost.switch_target = 0.35`
- `cost.staleness = 0.25`
- `cost.temp_lost = 0.70`
- `cost.low_visibility = 0.35`
- `cost.negative_margin = 1.00`

Practical meaning:
- increase `cost.uncertainty` if you want the solver to avoid risky, noisy tracks
- increase `cost.slew` if the gimbal spends too much time whipping across the field
- increase `cost.switch_target` if the solver target-switches too often
- increase `cost.temp_lost` if you want stronger punishment for coasting tracks
- increase `cost.low_visibility` if you want to avoid heavily oblique faces

---

## 5. Recommended Tuning Workflow

## Phase 1 — Static camera / PnP sanity check

1. Put a static armor at a known distance (for example 3 m).
2. Check `/tracker/target` and `/tracker/marker`.
3. Verify:
   - range is reasonable
   - yaw is not obviously biased
   - the target does not flicker in and out from pure geometry rejection
4. If geometry is bad:
   - check `light_ratio`
   - check `bbox_padding_y`
   - check `pnp.max_reprojection_error`
5. Repeat at multiple distances.

## Phase 2 — Basic tracker stability

1. Move the target slowly.
2. Watch `/tracker/info`:
   - `position_diff`
   - `yaw_diff`
   - damping values
3. If the tracker lags too much:
   - increase `ekf.sigma2_q_xyz`
   - possibly reduce `ekf.r_xyz_base` slightly
4. If it chatters too much:
   - decrease `ekf.sigma2_q_xyz`
   - increase `ekf.r_xyz_base`

## Phase 3 — Spin behavior

1. Spin the target.
2. Inspect:
   - `v_yaw`
   - `yaw_diff`
   - whether jumps are recognized cleanly
3. If spin adaptation is too slow:
   - increase `ekf.sigma2_q_yaw`
4. If yaw oscillates too much:
   - reduce `ekf.yaw_damping_alpha`
   - or increase `ekf.r_yaw_base` / `r_yaw_slope`
5. If face switches are missed:
   - inspect `tracker.maha_jump_threshold`
   - inspect `ekf.max_yaw_oblique_deg`

## Phase 4 — Static ballistic tuning

1. Shoot at a static target at medium range.
2. If impacts are **below** target:
   - increase `bullet_speed`
   - or reduce gravity / drag slightly
3. If impacts are **above** target:
   - decrease `bullet_speed`
   - or increase gravity / drag slightly
4. Repeat at longer range.

## Phase 5 — Dynamic latency tuning

1. Track a target moving laterally.
2. If shots lag behind the target, increase `time_bias`.
3. If shots lead too much, decrease `time_bias`.
4. Only after that, fine-tune `time_bias_alpha` and `gimbal_response_delay`.

## Phase 6 — Fire gate tuning on fast spinners

1. Test high-spin mode.
2. If the system almost never fires:
   - increase `angular_window`
   - or reduce `indirect_vyaw_threshold`
3. If it fires too loosely / too often on weak opportunities:
   - reduce `angular_window`
   - tighten `indirect_timing_tolerance`
   - increase uncertainty / visibility penalties

---

## 6. Common Problems & Solutions

### Tracker loses target during fast spins
- Increase `tracker.max_match_distance`
- Check `ekf.sigma2_q_yaw`
- Check `tracker.maha_jump_threshold`
- Check whether the face is often near the obliqueness cutoff

### Tracker keeps ghost tracks too long
- Decrease `tracker.lost_time_thres`
- Lower `tracker.max_track_range`
- Make `pnp.max_reprojection_error` stricter

### Spatial estimate chatters / jitters
- Decrease `ekf.sigma2_q_xyz`
- Increase `ekf.r_xyz_base`
- Check IMU / TF input stability
- Check detector quality before touching the EKF too much

### Yaw estimate is unstable on side views
- Increase `ekf.r_yaw_base` or `ekf.r_yaw_slope`
- Reduce `ekf.max_yaw_oblique_deg` if needed
- Verify `light_ratio` is not producing poor synthetic corners

### Trigger never fires
- Check `/tracker/cmd_gimbal`
- Verify distance is within `[min_fire_dist, max_fire_dist]`
- Verify the target is not stale (`max_measurement_age`)
- Verify the target is not in `TEMP_LOST`
- Consider relaxing `angular_window`
- Check whether gimbal rate limits are rejecting the shot opportunity

### Shots are consistently behind moving targets
- Increase `time_bias`
- If the gimbal is physically slow, also test `gimbal_response_delay`

### Shots are consistently in front of moving targets
- Decrease `time_bias`

### Long-range shots are bad but short-range shots are fine
- Re-check `bullet_speed`
- Then inspect `linear_drag_coeff`
- Then inspect `r_xyz_slope` if the target estimate itself is already unstable

---

## 7. Important Notes / Mismatches

### 1. `tracker.new_tracker_assumed_radius`
This parameter is active in `auto_aim_targeting`. It is used only for the new-tracker spawn guard, where an unmatched armor pose is converted into a provisional robot-center estimate before comparing it to existing tracker centers.

### 2. Some comments inside the current `debug.launch.py`
The launch wiring itself is understandable, but several inline comments in the current file are noisy or partially corrupted. The guide above reflects the **actual functional meaning** of the parameters, not those broken inline comments.

---

## Quick starting values from the current launch

If you want a compact summary of the active defaults:

- `pose_source = camera_imu`
- `light_ratio = 0.85`
- `bbox_padding_y = 80.0`
- `pnp.max_reprojection_error = 10.0`
- `tracker.max_match_distance = 0.30`
- `tracker.tracking_thres = 4`
- `tracker.lost_time_thres = 0.50`
- `ekf.sigma2_q_xyz = 1.0`
- `ekf.sigma2_q_yaw = 3.0`
- `ekf.xyz_damping_alpha = 0.85`
- `ekf.yaw_damping_alpha = 0.85`
- `bullet_speed = 25.0`
- `linear_drag_coeff = 0.01`
- `time_bias = 0.05`
- `angular_window = 0.09`
- `indirect_vyaw_threshold = 3.0`
- `cmd_smooth_alpha = 0.40`
