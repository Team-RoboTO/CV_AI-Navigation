# Calibration Guide

How to calibrate the auto-aim pipeline so the robot actually hits what it
is aiming at. Follow these procedures in order: a downstream parameter
that fights an upstream miscalibration is the fastest route to a system
nobody can debug.

## Order of operations

1. Camera intrinsics (`/camera_info`).
2. **YOLO-pose keypoint ordering check** (the new measurement model — silent
   if wrong, see step 2 below).
3. Gimbal sign convention (`gimbal.yaw_sign`, `gimbal.pitch_sign`).
4. Bullet speed (`bullet_speed`).
5. Gimbal height (`gimbal_height`).
6. Barrel offset (`barrel_offset_x/y/z`).
7. Static pitch test at known distances (verify the chain).
8. Optional: bore-sight residual measurement (do NOT compensate with
   `pitch_offset_deg`/`yaw_offset_deg`; instead, fix the underlying input).

## 1. Camera intrinsics

The PnP solver uses K and D from `/camera_info`. Wrong intrinsics
produce a depth bias that grows with the inverse of the focal length
error.

* RealSense D435/D435i: factory-calibrated. Verify by running a
  checkerboard recording and confirming the published K matrix matches
  the value reported by `rs-enumerate-devices`.
* Zed: factory-calibrated. Use the ZED Calibration tool only if the lens
  has been mechanically disturbed.

A 1% focal-length error at 5 m of range is ~5 cm of position error.

## 2. YOLO-pose keypoint ordering (CRITICAL, silent if wrong)

The `auto_aim` PnP path indexes each detection's four keypoints as
TL, TR, BR, BL in image pixels and feeds them to a fixed 3D armor model
in that exact order. **There is no runtime reordering.** If the trained
model emits keypoints in a different convention (a real risk: many
RoboMaster light-bar models annotate the four light-bar endpoints in
some other order), PnP will still solve and `pnp_reproj_err_norm` may
even look acceptable, but the recovered yaw and tvec will be silently
wrong by up to 90°. The robot will then aim at a plausible but incorrect
location.

This must be verified once per model export, on a known frame. The
verification procedure:

1. Park the robot in front of a static armor at ~3 m.
2. Run the bench launch:
   ```
   ros2 launch launch_pkg debug_targeting.launch.py publish_debug_every:=1
   ```
3. Record one rosbag covering at least 5 s of stable detections:
   ```
   ros2 bag record /detector/armors_keypoints /yolo/debug_image /auto_aim/debug
   ```
4. Open one frame of `/yolo/debug_image` in any image viewer.
5. From the matching `/auto_aim/debug` message, read `kp_image_points` —
   the eight floats are `(TLx, TLy, TRx, TRy, BRx, BRy, BLx, BLy)` in
   the model's index order.
6. Plot the four points on the frame, labelled `0/1/2/3`. Confirm
   physically:
   - Point `0` is the upper-left armor corner.
   - Point `1` is the upper-right armor corner.
   - Point `2` is the lower-right armor corner.
   - Point `3` is the lower-left armor corner.
   - Walking `0→1→2→3` is clockwise in the image (image y grows down).
7. Confirm `/auto_aim/debug.kp_geometry_valid == true` for the same frame.
   If it is `false`, the convex+winding sanity check rejected the quad —
   either the keypoints are bow-tied (likely a label permutation in the
   trained weights) or the armor is at an extreme oblique angle. Re-do
   on a face-on frame; if `kp_geometry_valid` stays `false`, the model
   is mis-labeled and **must be retrained or the indices remapped before
   the keypoint path is safe to use in competition**.

The convex/winding check does not catch a uniform index rotation such as
TR,BR,BL,TL. That case is still convex and clockwise, but PnP correspondence
is wrong. The visual `0/1/2/3` check above is mandatory.

This is a one-time per-model check. It does **not** need to be redone
between bag captures or between robots, only between model exports.

## 3. Gimbal sign convention

With the robot on blocks (no firing), set the target yaw to a known
value via the operator UI and confirm the gimbal moves the expected
direction:

* If commanding `+0.2 rad` produces a counter-clockwise yaw (viewed from
  above), `gimbal.yaw_sign = +1.0`.
* If it produces a clockwise yaw, `gimbal.yaw_sign = -1.0`.

Same procedure for pitch with the convention "positive = nose up".

The validator emits a FATAL error if either sign is not exactly +/- 1.

## 4. Bullet speed

Ground truth: chronograph the actual bullet at the barrel.

If a chronograph is unavailable:

1. Set up two horizontal beams at known horizontal distance (e.g. 1.0 m
   apart), each with a microphone or photogate.
2. Fire one shot through both. Measure the time-of-flight `dt` between
   the two events.
3. `bullet_speed = distance / dt`.

For RoboMaster 17mm referees, expected range is 15..30 m/s. Defaults of
25 m/s are placeholder values; **measure on every robot**.

If the actual bullet speed is below 5 m/s or above 50 m/s, the
`ConfigValidator` will emit an error at startup.

## 5. Gimbal height

Definition: the vertical distance from the floor to the camera optical
center, with the gimbal at zero pitch.

Procedure:

1. Place a level on the chassis.
2. Set the gimbal pitch to zero (operator UI or by hand if powered down).
3. Measure with a tape from the floor to the lens center.
4. Round to the nearest centimeter and put the value in
   `gimbal_height` in the YAML.

Typical values: 0.25–0.40 m. Outside that range the validator warns.

## 6. Barrel offset

Definition: the offset from the camera optical center to the barrel
exit, in the **gimbal body frame**:

* `barrel_offset_x` — forward (positive = barrel ahead of camera lens)
* `barrel_offset_y` — left    (positive = barrel left of camera lens)
* `barrel_offset_z` — up      (positive = barrel above camera lens)

Procedure:

1. With gimbal at zero yaw and zero pitch, measure with a tape from the
   camera lens center to the barrel exit.
2. Decompose the vector into the forward / left / up components.
3. Put the values in the YAML.

Most RoboMaster gimbals have the barrel directly below the camera by
~5–15 cm: `barrel_offset_z` is negative.

The validator warns if any component exceeds 30 cm magnitude.

## 7. Static pitch test

Goal: verify the chain camera → PnP → transform → ballistic produces a
hit at a known distance.

Setup:

* Park the robot on the firing line, facing a static armor target.
* Choose three distances: 1.5 m, 3.0 m, 5.0 m.
* Aim the gimbal at the armor center using the auto-aim system.
* Wait for `/auto_aim/debug.fire_blocker == ALLOWED`.
* Fire one shot per distance. Repeat 3 times and average the impact.

Record per shot:

| Distance [m] | cmd pitch [rad] | impact y [m] | comment        |
|--------------|-----------------|--------------|----------------|
| 1.5          | …               | …            | Aim center     |
| 3.0          | …               | …            | Aim center     |
| 5.0          | …               | …            | Aim center     |

Use `config/calibration_table_template.csv` as a starting template.

Interpretation:

* If the impact is consistently high at **all** distances by the same
  angular offset, the gimbal sign or boresight is off.
* If the impact is high more at **long** distance, the bullet speed is
  too low (gravity drop is bigger than the model expects).
* If the impact is low more at **short** distance, the barrel offset z
  is wrong (parallax not corrected).
* If the **horizontal** impact drifts more at short distance, the
  barrel offset x or y is wrong.

Fix the root cause that the data points to. Do not paper over it with
`pitch_offset_deg` or `yaw_offset_deg`. Those parameters exist for
backwards compatibility and the validator warns on any non-zero value.

## 8. Bore-sight residual (last resort)

If after steps 1–7 there is still a small constant angular offset that
cannot be explained, you can set `pitch_offset_deg` / `yaw_offset_deg`
as a final compensation. The recommended workflow is:

1. Fire 10 shots at 3 m on a static target with the system running.
2. Compute the mean horizontal and vertical impact errors in radians.
3. Convert to degrees and put the value in the YAML.

Validate that this offset stays within ~0.5° on each axis. A larger
value indicates one of steps 1–7 is wrong.

## Reading `/auto_aim/debug` during calibration

After P1, the structured debug topic exposes:

* `bbox_*`: detector output stability (step 1 sanity check).
* `pnp_reproj_err`, `pnp_reproj_err_norm`: PnP solver quality.
* `pnp_tvec_*`: raw camera-frame depth (independent of transform).
* `odom_*`: world-frame position after the IMU rotation.
* `ekf_state`: smoothed state from the tracker.
* `aim_abs_yaw`, `aim_abs_pitch`: commanded angles before smoothing.
* `cmd_yaw_published`, `cmd_pitch_published`: what the gimbal actually
  receives.

If the gap between `aim_abs_*` and `cmd_*_published` is large for
several frames, smoothing is the culprit, not the calibration.
