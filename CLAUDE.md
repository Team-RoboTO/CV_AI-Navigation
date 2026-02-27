# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Context

This is a ROS 2 (Humble) robotics project for the RoboMaster competition. It implements an auto-aim system running on a Jetson Orin inside an Isaac ROS Docker container. The full pipeline detects enemy armors via YOLO, tracks them with an EKF, solves ballistic trajectories, and commands the gimbal over serial.

## Build System

All C++ packages use `colcon` inside the Isaac ROS Docker environment:

```bash
# Start the Isaac ROS dev container (done automatically on boot via systemd)
docker start isaac_ros_dev-aarch64-container
docker exec -it isaac_ros_dev-aarch64-container bash

# Inside the container — build all packages
cd /workspaces/isaac_ros-dev
colcon build --symlink-install

# Build a single package
colcon build --symlink-install --packages-select <package_name>

# Source the workspace
source install/setup.bash
```

Python packages (`filter`, `cmd_vel_serial`) are also built via colcon (they use `setup.py`/`setup.cfg`).

## Running the System

```bash
# Standard production launch (RealSense + YOLOv8 + detection pipeline)
ros2 launch launch_pkg realsense_only_det.launch.py

# Optimized launch for Jetson (uses isaac_ros_realsense + zero-copy NITROS pipeline)
ros2 launch launch_pkg test_realsense_optimized_launcher.launch.py

# Filter + shooting prediction (choose the launch variant matching robot type and team color)
ros2 launch filter __sentry_red.launch.py   # or __standard_blue.launch.py etc.

# Serial command publisher (gimbal/UART bridge)
ros2 run cmd_vel_serial cmd_vel_subscriber
```

The startup script `src/bbox_filter_node/start_docker_and_nodes.sh` automates all three above in tmux sessions and is registered in crontab for boot.

## Testing

Python packages have lint tests only:
```bash
# Run lint tests for a Python package
colcon test --packages-select filter
colcon test-result --all
```

C++ packages have lint checks (copyright and uncrustify are excluded). GTest infrastructure exists in `armor_tracker` but tests are commented out.

## Architecture

The auto-aim pipeline flows in this order:

```
Camera (RealSense / ZED)
  └─> [NITROS composable container]
        ├─ isaac_ros_realsense / realsense2_camera  → /image
        ├─ DnnImageEncoderNode                      → /tensor_pub
        ├─ TensorRTNode (YOLOv8/YOLO11 .plan)       → /output_tensor
        ├─ YoloV8DecoderNode                        → /detections_output (Detection2DArray)
        └─ BboxXyzNode (pointcloud_consumer)        → /detections_output/with_pose

  └─> filter (Python, ROS 2 node "talker")
        Scores and selects the best bounding box     → /detections_output/optimal_target

  └─> armor_tracker (C++, composable node)
        PnP solver → 3D pose; EKF "Spinning Top" model → /tracker/target (Target msg)

  └─> rm_trajectory (C++, composable node)
        Ballistic solver (iterative, drag + gravity)  → /tracker/cmd_gimbal (GimbalCmd msg)

  └─> cmd_vel_subscriber (Python)
        Subscribes /cmd_vel and sends UART bytes to the lower computer
```

### Key Packages

| Package | Language | Role |
|---|---|---|
| `launch_pkg` | Python (launch) | Composable node containers, sensor configs, ONNX/plan model files |
| `filter` | Python | BBox scoring/selection, shoot prediction without drag (`PredictionWithout`) |
| `armor_tracker` | C++ | PnP solve + 9-state EKF tracker (`rm_auto_aim::ArmorTrackerNode`) |
| `rm_trajectory` | C++ | Iterative ballistic solver (`rm_auto_aim::TrajectorySolverNode`) |
| `auto_aim_interfaces` | ROS 2 msg | Custom messages: `Armor`, `Armors`, `Target`, `GimbalCmd`, `TrackerInfo`, debug msgs |
| `cmd_vel_subscriber` | Python | Serial/UART bridge to lower computer |

### Custom Messages (`auto_aim_interfaces`)

- **`Target`**: EKF output — center position/velocity, yaw, yaw-rate, two radii, dz (for 4-armor robots)
- **`GimbalCmd`**: Trajectory solver output — pitch, yaw, distance, fire_cmd (bool)
- **`Armors`/`Armor`**: 3D pose of detected armors after PnP

### Models

ONNX and TensorRT `.plan` files live in `src/launch_pkg/resources/`. The `.plan` file **must be generated on the target Jetson** using `trtexec` (GPU-specific optimization):
```bash
trtexec --onnx=yolov8_op.onnx --saveEngine=yolov8_op.plan --fp16
```

### armor_tracker EKF State

9D state: `[xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r]` — models robot center, not just the visible armor plate. Handles armor-jump events when the robot rotates and a different armor face becomes visible. C++ standard is **C++14**.

## Contributing

- **Never push to `main`** — always open a PR.
- **PR title format**: `[vX.Y.Z] SubDivision_Specification Description`
  - SubDivision: `CV`, `AI`, or `Navigation`
  - Specification: `Bbox_Shooting`, `Uart_protocol`, `SLAM`, `Sensors`, `AI_Model`, `Camera`
- Rebase onto main before pushing: `git fetch origin main && git rebase origin/main`
- Merging a PR automatically creates a GitHub release tag via `.github/workflows/release-on-merge.yml`.
- Each version tag must be unique — never reuse a version number.
