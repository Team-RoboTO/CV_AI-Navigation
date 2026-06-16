# Head-referenced mode

This package uses a deliberate convention:

```text
map -> odom -> base_link -> livox_frame
                     -> base_scan
                     -> base_footprint
```

`base_link` is the ROS-compatible navigation frame fixed to the **LiDAR/IMU head rigid body**. It is not the chassis center.

Why this mode exists:

- FAST_LIO estimates the pose of the rigid body carrying the Livox + IMU.
- On this robot that rigid body is the head, not the chassis.
- Therefore the cleanest way to avoid TF contradictions while the head rotates is to let `base_link` follow the head.

Important TF rules:

- `odom -> base_link` comes from FAST_LIO through `tf_frame_relay.py`.
- `base_link -> livox_frame` has zero translation and only the fixed physical mounting rotation.
- In the default upside-down Livox configuration, `base_link -> livox_frame` is 180 deg about X.
- `base_link -> base_scan` is identity. It is just the 2D slicing frame for `/scan`.
- Do **not** put the 0.8 m chassis-to-LiDAR height in `base_link -> livox_frame` in this mode.

Command convention:

- Navigation may produce `linear.x` and `linear.y`.
- `cmd_vel_xy_only.py` forces `angular.z = 0` on the final command topic so Nav2 does not command yaw.
- Yaw can be controlled by the combat/CV stack separately.

Main limitation:

This mode localizes the head-referenced navigation frame. It is suitable if your control architecture treats the head direction as the operational forward direction. It is not a physically exact chassis localization unless the chassis follows the head tightly enough for your application.

## Planar relay fix

`tf_frame_relay.py` now publishes `odom -> base_link` as a **2D planar transform**:

- x and y come from FAST_LIO;
- z is forced to 0;
- roll and pitch are removed;
- only yaw is kept.

This is required because `slam_toolbox` is a 2D SLAM system. Feeding it a
`base_link` with roll about 180 degrees, pitch changes, or z drift causes scan
projection errors and map overwriting.

The Livox upside-down mounting is still represented by the static TF:

```text
base_link -> livox_frame = 180 deg about X
```

The dynamic odometry transform must stay planar:

```text
odom -> base_link = x, y, yaw only
```
