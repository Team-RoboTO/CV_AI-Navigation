# Topics

## Detector output

```text
/detector/armors              vision_msgs/Detection2DArray
/detector/armors_keypoints    autoaim/ArmorKeypointArray
/detector/armors_keypoints_json std_msgs/String
/yolo/debug_image             sensor_msgs/Image
/zed/imu_data                 sensor_msgs/Imu
/camera_info                  sensor_msgs/CameraInfo
```

`/detector/armors_keypoints` is the tracking input. The bbox topic is kept for legacy visualization and quick checks.

## Auto aim output

```text
/cmd_vel_AI          geometry_msgs/Twist
/tracker/aim_pixels  geometry_msgs/Twist
```

`/cmd_vel_AI` layout:

```text
linear.x   target distance [m]
angular.x  fire flag, 0.0 or 1.0
angular.y  absolute pitch target [rad]
angular.z  absolute yaw target [rad]
```

`/tracker/aim_pixels` is for overlay/debug only.

## Micro feedback

```text
/micro_status std_msgs/Float32MultiArray
```

Expected values:

```text
data[0] yaw [rad]
data[1] pitch [rad]
data[2] chassis vx [m/s]
data[3] chassis vy [m/s]
```

The serial bridge may append transmit echo values after the first four fields.
