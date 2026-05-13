# Gimbal Command Contract

This document is the authoritative description of the runtime command interface
between the `auto_aim` node and the gimbal microcontroller. It MUST NOT be
changed without coordinated firmware updates.

The `msg/GimbalCmd.msg` file in this package is **not used at runtime**. The
runtime contract is `geometry_msgs/Twist`, described below. `GimbalCmd.msg` is
preserved as a placeholder for a possible future migration but is not produced
or consumed by the current node.

## Primary command topic

| Property        | Value                                              |
|-----------------|----------------------------------------------------|
| Topic           | `/tracker/cmd_gimbal`                              |
| Message type    | `geometry_msgs/Twist`                              |
| QoS             | `rclcpp::SensorDataQoS()` (best effort, depth 10)  |
| Publish rate    | One message per detection callback (~30 Hz nominal) |
| Reference frame | Microcontroller absolute frame (the IMU startup frame as published on `/micro_imu`) |

### Field mapping

| Twist field   | Meaning                                                 | Unit  |
|---------------|---------------------------------------------------------|-------|
| `angular.z`   | Absolute target yaw, after `gimbal.yaw_sign` flip       | rad   |
| `angular.y`   | Absolute target pitch, after `gimbal.pitch_sign` flip   | rad   |
| `angular.x`   | Fire flag: `1.0` to fire, `0.0` to hold                 | flag  |
| `linear.x`    | Range to selected armor face                            | m     |
| `linear.y`    | Reserved (always zero)                                  | --    |
| `linear.z`    | Reserved (always zero)                                  | --    |

These are **absolute destinations**, not deltas. The microcontroller is
expected to drive its gimbal to the commanded yaw/pitch in its own frame.
Command policy:

* `TRACKING`: publish the normal absolute target yaw/pitch.
* `TEMP_LOST`: fire is blocked. The node may publish a short bounded coast
  command for at most `cmd.temp_lost_coast_max_s`, then it falls back to hold.
* `DETECTING`, `LOST`, invalid target, or expired coast: publish a hold
  destination when one is safe.

Hold means the latest fresh `/micro_imu` yaw/pitch, or the previous command if
pose feedback has gone stale. It must not publish yaw/pitch zero as a generic
hold command, because zero is a real absolute gimbal destination. If no pose
sample and no previous command exist yet, the node suppresses the gimbal command
until a safe absolute hold destination is known. `/auto_aim/debug` reports this
with `cmd_published`, `cmd_hold_active`, and `cmd_coast_active`.

### Sign convention

Two parameters flip the published sign so the C++ code can use a consistent
"left-handed yaw, nose-up pitch" convention internally regardless of how the
microcontroller is wired:

* `gimbal.yaw_sign`: `+1.0` if the microcontroller's yaw is positive in the
  same direction as the C++ yaw (counter-clockwise viewed from above).
  `-1.0` to invert.
* `gimbal.pitch_sign`: `+1.0` if a positive pitch from the microcontroller
  corresponds to nose-up. `-1.0` to invert.

The published `angular.z` and `angular.y` are pre-multiplied by these signs,
so the microcontroller does not need to flip again.

### Fire semantics

* `angular.x == 1.0` indicates the fire gate granted permission **for this
  callback**. A fire gate hysteresis keeps fire enabled across small drifts
  of the angular margin to avoid flicker. See `auto_aim_debugging_guide.md`
  for the list of fire blocker reasons.
* `angular.x == 0.0` means **do not fire**. The microcontroller must treat
  the absence of a `1.0` as "no fire", not as "keep last".

The fire alignment gate is configured by `fire.alignment_source`:

* `camera_angle`: gate on the camera-frame selected-target angle. This is the
  safe real-robot default when camera pose follows fresh gimbal feedback.
* `relative_error`: gate on `sqrt(aim.rel_yaw^2 + aim.rel_pitch^2)`, which
  still depends on fresh pose feedback.
* `disabled`: do not use an alignment gate. This is useful for static fake-IMU
  video tests only; it does not prove physical gimbal convergence.

## Legacy / compatibility command topic

| Property        | Value                                              |
|-----------------|----------------------------------------------------|
| Topic           | `/cmd_vel_AI`                                      |
| Message type    | `geometry_msgs/Twist`                              |
| QoS             | `depth=10` reliable                                |
| Field mapping   | `angular.x` = fire, `angular.y` = pitch, `angular.z` = yaw |
| Distance field  | NOT published on this topic                        |
| Purpose         | Backwards compatibility with earlier consumers     |

This publisher is kept active so that any subscriber still listening on
`/cmd_vel_AI` continues to see the same yaw/pitch/fire stream as
`/tracker/cmd_gimbal`. Do not remove this publisher without confirming no
other component subscribes.

## Pixel overlay topic

| Property        | Value                                              |
|-----------------|----------------------------------------------------|
| Topic           | `/tracker/aim_pixels`                              |
| Message type    | `geometry_msgs/Twist`                              |
| Purpose         | 2D image-plane overlay for the operator HUD        |

Field mapping (debug only, not consumed by firmware):

| Field           | Meaning                                            |
|-----------------|----------------------------------------------------|
| `linear.x`      | Aim pixel x (commanded ballistic ray)              |
| `linear.y`      | Aim pixel y (commanded ballistic ray)              |
| `angular.x`     | Impact pixel x (selected armor face center)        |
| `angular.y`     | Impact pixel y (selected armor face center)        |
| `angular.z`     | Fire flag (mirrors `/tracker/cmd_gimbal angular.x`)|

## Marker topic

* Topic: `/tracker/marker`
* Type: `visualization_msgs/MarkerArray`
* Purpose: RViz visualization of robot center, four armor faces, velocity
  arrow, aim sphere, impact sphere. Debug-only.

## Inputs consumed by the node

| Topic                | Type                                  | QoS               |
|----------------------|---------------------------------------|-------------------|
| `/detector/armors_keypoints` | `auto_aim/ArmorKeypointArray` | `SensorDataQoS()` |
| `/camera_info`       | `sensor_msgs/CameraInfo`              | `SensorDataQoS()` |
| `/micro_imu`         | `std_msgs/Float32MultiArray`          | `SensorDataQoS()` |

`auto_aim_node` consumes YOLO-pose keypoints exclusively. The detector
still publishes `/detector/armors` (`vision_msgs/Detection2DArray`) for
legacy debug consumers (e.g. RViz overlays), but `auto_aim` does not
subscribe to it. There is no bbox fallback path.

`/micro_imu` is a `Float32MultiArray` with at least two elements:

| Index | Meaning                                                  | Unit |
|-------|----------------------------------------------------------|------|
| `[0]` | Current absolute gimbal yaw, microcontroller convention  | rad  |
| `[1]` | Current absolute gimbal pitch, microcontroller convention| rad  |

The C++ side multiplies these by `yaw_sign`/`pitch_sign` before using them.
This keeps the published commands in the same reference as the
microcontroller's own measurements.

## What MUST NOT change without firmware coordination

* The topic name `/tracker/cmd_gimbal`.
* The Twist message type.
* The field mapping above (yaw on `angular.z`, pitch on `angular.y`,
  fire on `angular.x`, distance on `linear.x`).
* The unit (radians, not degrees).
* The "absolute target, not delta" semantics.
* The legacy `/cmd_vel_AI` publisher.
* The `/micro_imu` Float32MultiArray layout.

Any change to any of the items above is a breaking change and requires a
matching firmware update.

## Bore-sight offsets (calibration footgun)

The node accepts `pitch_offset_deg` and `yaw_offset_deg` parameters. These
subtract a constant from the published yaw/pitch and exist for legacy
compatibility. **They should be left at zero in any new deployment.** Use
`docs/calibration_guide.md` to measure the actual barrel offsets and
ballistic bias, which is more diagnosable than a global angular constant.
