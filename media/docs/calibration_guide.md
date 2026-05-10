# Calibration Guide

How to calibrate the auto-aim pipeline so the robot actually hits what it
is aiming at. Follow these procedures in order: a downstream parameter
that fights an upstream miscalibration is the fastest route to a system
nobody can debug.

## Order of operations

1. Camera intrinsics (`/camera_info`).
2. Gimbal sign convention (`gimbal.yaw_sign`, `gimbal.pitch_sign`).
3. Bullet speed (`bullet_speed`).
4. Gimbal height (`gimbal_height`).
5. Barrel offset (`barrel_offset_x/y/z`).
6. Static pitch test at known distances (verify the chain).
7. Optional: bore-sight residual measurement (do NOT compensate with
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

## 2. Gimbal sign convention

With the robot on blocks (no firing), set the target yaw to a known
value via the operator UI and confirm the gimbal moves the expected
direction:

* If commanding `+0.2 rad` produces a counter-clockwise yaw (viewed from
  above), `gimbal.yaw_sign = +1.0`.
* If it produces a clockwise yaw, `gimbal.yaw_sign = -1.0`.

Same procedure for pitch with the convention "positive = nose up".

The validator emits a FATAL error if either sign is not exactly +/- 1.

## 3. Bullet speed

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

## 4. Gimbal height

Definition: the vertical distance from the floor to the camera optical
center, with the gimbal at zero pitch.

Procedure:

1. Place a level on the chassis.
2. Set the gimbal pitch to zero (operator UI or by hand if powered down).
3. Measure with a tape from the floor to the lens center.
4. Round to the nearest centimeter and put the value in
   `gimbal_height` in the YAML.

Typical values: 0.25–0.40 m. Outside that range the validator warns.

## 5. Barrel offset

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

## 6. Static pitch test

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

## 7. Bore-sight residual (last resort)

If after steps 1–6 there is still a small constant angular offset that
cannot be explained, you can set `pitch_offset_deg` / `yaw_offset_deg`
as a final compensation. The recommended workflow is:

1. Fire 10 shots at 3 m on a static target with the system running.
2. Compute the mean horizontal and vertical impact errors in radians.
3. Convert to degrees and put the value in the YAML.

Validate that this offset stays within ~0.5° on each axis. A larger
value indicates one of steps 1–6 is wrong.

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
