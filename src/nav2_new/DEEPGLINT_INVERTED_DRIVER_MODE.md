# DeepGlint-style inverted Mid-360 mode

Use this package variant when your Livox driver already applies the upside-down correction to BOTH the point cloud and the IMU.

Driver config example in livox_ros_driver2/config/MID360_config.json:

```json
"extrinsic_parameter" : {
  "roll": 180.0,
  "pitch": 0.0,
  "yaw": 0.0,
  "x": 0,
  "y": 0,
  "z": 0
}
```

Rules:

1. Use the modified driver that rotates both cloud and IMU, not only PointCloud2.
2. Keep FAST-LIO `extrinsic_R` as identity unless you have measured the real internal LiDAR-IMU calibration.
3. Launch sensors with `mount:=normal`. Do not add another 180 degree static TF.
4. Do not use `invert_x`, `invert_y`, or `invert_yaw` sign hacks.
5. Record a bag with the modified driver; bags recorded with the unmodified driver are not equivalent.
