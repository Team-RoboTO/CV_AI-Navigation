# CV_AI-Navigation

ROS 2 workspace for the RoboMaster auto-aim pipeline.

## Workspace

```text
src/autoaim
media/output2.mp4
media/docs/
media/jetson_notes.md
video_publisher.py
```

`src/autoaim` is the only ROS package in `src/`. It contains the ZED detector, auto-aim C++ node, serial bridge, viewer, launch files, and messages. TensorRT and source model assets live in the workspace-level `models/` directory.

## Build

```bash
cd /home/andreas/ros_ws
colcon build --packages-select autoaim --symlink-install
source install/setup.bash
```

## Launch

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

## Armor Classes

Current YOLO26 keypoint labels:

- `0`: blue armor
- `1`: grey armor, ignored by the tracker
- `2`: red armor

`target_classes` defaults to `["0"]`. Use `["2"]` for red enemies.

Override the serial port:

```bash
ros2 launch autoaim sentry.launch.py serial_port:=/dev/ttyACM1
```

Override the TensorRT engine:

```bash
ros2 launch autoaim sentry.launch.py \
  engine_path:=/absolute/path/to/yolov26_keypoints.engine
```

## Topics

```text
/detector/armors_keypoints  autoaim/ArmorKeypointArray
/cmd_vel_AI                 geometry_msgs/Twist
/micro_status               std_msgs/Float32MultiArray
/tracker/debug_image        sensor_msgs/Image
```

## Contribution workflow

```bash
git checkout main
git pull origin main
git checkout -b feature/descriptive-name
git add <files>
git commit -m "Clear description"
git fetch origin main
git rebase origin/main
git push origin feature/descriptive-name
```

Open a PR and wait for review before merging into `main`.
