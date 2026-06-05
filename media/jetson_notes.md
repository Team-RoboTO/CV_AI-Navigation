# Jetson notes

## Active pipeline

Build from the workspace root:

```bash
colcon build --packages-select autoaim --symlink-install
source install/setup.bash
```

Standard:

```bash
ros2 launch autoaim standard.launch.py
```

Hero:

```bash
ros2 launch autoaim hero.launch.py
```

Sentry:

```bash
ros2 launch autoaim sentry.launch.py
```

All three launch files start the same runtime graph:

- `zed_detector`: ZED X Mini frames, TensorRT inference, keypoints, camera info, IMU.
- `autoaim`: keypoint PnP, target tracking, ballistic lead, command generation.
- `serial_bridge.py`: microcontroller link on `/dev/ttyACM0`.
- `autoaim_viewer`: debug image on `/tracker/debug_image`.

## Armor class

`target_classes` is set to `["0"]` in all launch profiles. With the current model labels:

- `0`: blue armor
- `1`: grey armor
- `2`: red armor

Change the launch parameter before running against a different team color.

## Main topics

```text
/detector/armors              vision_msgs/Detection2DArray
/detector/armors_keypoints    autoaim/ArmorKeypointArray
/yolo/debug_image             sensor_msgs/Image
/zed/imu_data                 sensor_msgs/Imu
/camera_info                  sensor_msgs/CameraInfo
/micro_status                 std_msgs/Float32MultiArray
/cmd_vel_AI                   geometry_msgs/Twist
/tracker/aim_pixels           geometry_msgs/Twist
/tracker/debug_image          sensor_msgs/Image
```

## Manual commands

Run only the C++ node:

```bash
ros2 run autoaim autoaim_node --ros-args \
  -p target_classes:="['0']" \
  -p angular_window:=0.3 \
  -p bullet_speed:=25.0
```

Override the serial port:

```bash
ros2 launch autoaim sentry.launch.py serial_port:=/dev/ttyACM1
```

Override the TensorRT engine:

```bash
ros2 launch autoaim sentry.launch.py \
  engine_path:=/absolute/path/to/yolov26_keypoints.engine
```
