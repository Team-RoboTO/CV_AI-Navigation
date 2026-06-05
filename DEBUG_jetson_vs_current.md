# !!!! AI WARNING !!!!
# Jetson rollback diff, captured before changes

Captured from `/home/andreas/ros_ws` before the rollback/reorganization.

## Git state

- `git status --short` showed only one untracked tree: `jetson/`.
- Tracked workspace files were clean before this debug file was created.
- `.gitignore` was read only and must stay unchanged.

## Package layout difference

Current tracked `src/`:

- `src/auto_aim`
- `src/bbox_filter_node`
- `src/cmd_vel_subscriber`
- `src/launch_pkg`

Jetson `src/`:

- `jetson/src/auto_aim_2`
- `jetson/src/auto_aim_interfaces`
- `jetson/src/armor_tracker`
- `jetson/src/rm_trajectory`
- `jetson/src/cmd_vel_subscriber`
- `jetson/src/zed-ros2-interfaces`
- `jetson/src/zed-ros2-wrapper`
- `jetson/src/isaac_ros_common`

`diff -qr src jetson/src`:

```text
Only in jetson/src: armor_tracker
Only in src: auto_aim
Only in jetson/src: auto_aim_2
Only in jetson/src: auto_aim_interfaces
Only in src: bbox_filter_node
Only in jetson/src/cmd_vel_subscriber/cmd_vel_serial: serial_new_communication_USB_C_final.py
Only in jetson/src/cmd_vel_subscriber/cmd_vel_serial: serial_new_communication_USB_C_final_old.py
Only in jetson/src/cmd_vel_subscriber/cmd_vel_serial: serial_new_communication_USB_C_for_test.py
Files src/cmd_vel_subscriber/setup.py and jetson/src/cmd_vel_subscriber/setup.py differ
Only in jetson/src: isaac_ros_common
Only in src: launch_pkg
Only in jetson/src: rm_trajectory
Only in jetson/src: zed-ros2-interfaces
Only in jetson/src: zed-ros2-wrapper
```

## Current-only code that the rollback removes

`src/auto_aim`:

- `CMakeLists.txt`, `package.xml`
- `config/calibration_table_template.csv`
- `config/params_realsense_16.yaml`
- `config/params_zed_64.yaml`
- `include/auto_aim/config_validator.hpp`
- `include/auto_aim/debug_frame.hpp`
- `include/auto_aim/debug_publisher.hpp`
- `include/auto_aim/fire_gate.hpp`
- `include/auto_aim/frame_transformer.hpp`
- `include/auto_aim/pnp_solver.hpp`
- `include/auto_aim/tracker.hpp`
- `msg/ArmorKeypoint.msg`
- `msg/ArmorKeypointArray.msg`
- `msg/AutoAimDebug.msg`
- `msg/GimbalCmd.msg`
- `scripts/fake_micro_imu_node.py`
- `scripts/yolo26_pose_realsense_node.py`
- `src/auto_aim_node.cpp`
- `src/config_validator.cpp`
- `src/debug_publisher.cpp`
- `src/fire_gate.cpp`
- `src/frame_transformer.cpp`
- `src/pnp_solver.cpp`
- `src/tracker.cpp`

`src/bbox_filter_node`:

- Python bbox/depth/IMU filter nodes, shooting helpers, old viewer nodes, and nine `filter/launch/__*.launch.py` presets.
- Removed by request.

`src/launch_pkg`:

- Realsense, Isaac ROS, RT-DETR, YOLOv8, RViz, URDF, mesh, and long-form launch assets.
- Removed by request. Its model resources are superseded by the new model layout.

`src/cmd_vel_subscriber`:

- Existing standalone serial package.
- Removed by request because the Jetson rollback package carries the serial bridge inside the auto-aim package.

Root files:

- `last_logs.txt`
- `last_logs2.txt`
- `viewer.py`

## Jetson package imported as runtime base

`jetson/src/auto_aim_2` is the rollback target and becomes `src/autoaim`.

Files:

- `CMakeLists.txt`
- `package.xml`
- `include/auto_aim/pnp_solver.hpp`
- `include/auto_aim/tracker.hpp`
- `launch/auto_aim_launch.py`
- `msg/ArmorKeypoint.msg`
- `msg/ArmorKeypointArray.msg`
- `msg/GimbalCmd.msg`
- `serial_new_communication_USB_C_micro_imu_v9.py`
- `src/auto_aim_node.cpp`
- `src/pnp_solver.cpp`
- `src/tracker.cpp`
- `viewer_node.py`
- `zed_yolo26_pose_keypoints_node_auto_aim2.py`
- `MODIFICHE_AUTO_AIM.md`

Code-level behavior in `auto_aim_2`:

- Builds one C++ component and standalone executable from `pnp_solver.cpp`, `tracker.cpp`, and `auto_aim_node.cpp`.
- Generates only `GimbalCmd`, `ArmorKeypoint`, and `ArmorKeypointArray`.
- Uses `auto_aim_2::msg::ArmorKeypointArray` in C++ and Python.
- Subscribes `/detector/armors_keypoints`.
- Publishes `/cmd_vel_AI`.
- Publishes `/tracker/aim_pixels`.
- Serial bridge publishes `/micro_status` and subscribes `/cmd_vel_AI`.
- ZED/TensorRT detector publishes `/detector/armors`, `/detector/armors_keypoints`, `/detector/armors_keypoints_json`, `/yolo/debug_image`, `/zed/imu_data`, and `/camera_info`.
- Viewer subscribes the detector, `/cmd_vel_AI`, `/tracker/aim_pixels`, `/micro_status`, and `/camera_info`, then publishes `/tracker/debug_image`.
- Launch file starts four nodes: serial bridge, C++ auto aim, viewer, ZED detector.

## Main code difference: current `auto_aim` vs Jetson `auto_aim_2`

Current `src/auto_aim` has the newer refactor:

- Extra modules: `config_validator`, `debug_publisher`, `fire_gate`, `frame_transformer`.
- Extra message: `AutoAimDebug`.
- Extra config files for Realsense 16 and ZED 64.
- Extra detector script for Realsense and fake micro IMU.
- Structured `/auto_aim/debug` output and fire/debug diagnostics.
- More defensive config validation and parameter YAML usage.

Jetson `auto_aim_2` is the older competition pipeline:

- Smaller C++ surface: PnP, tracker, auto aim node.
- Inline launch parameters instead of config YAML.
- No `AutoAimDebug` message and no `/auto_aim/debug` publisher.
- No Realsense detector path.
- ZED X Mini + TensorRT detector is part of the package.
- Serial bridge and viewer are part of the package.
- Uses workspace-level model paths under `models/` after reorganization.

## Overlapping `cmd_vel_subscriber` difference

The only shared package with changed content is `cmd_vel_subscriber`.

Jetson adds console scripts:

```diff
 serial_try = cmd_vel_serial.serial_try:main
+usb_c_test = cmd_vel_serial.serial_new_communication_USB_C_for_test:main
+usb_c_final = cmd_vel_serial.serial_new_communication_USB_C_final:main
```

Jetson also adds these serial files:

- `serial_new_communication_USB_C_final.py`
- `serial_new_communication_USB_C_final_old.py`
- `serial_new_communication_USB_C_for_test.py`

These are not imported because the requested rollback removes the standalone `cmd_vel_subscriber` package and uses the serial bridge embedded in `autoaim`.

## Jetson-only packages not imported

Not imported into `src/` because the requested final workspace should use the rollback auto-aim package and remove old tracker/filter/serial helper packages:

- `armor_tracker`
- `auto_aim_interfaces`
- `rm_trajectory`
- `cmd_vel_subscriber`
- `isaac_ros_common`
- `zed-ros2-interfaces`
- `zed-ros2-wrapper`

The ZED wrapper and Isaac ROS trees in `jetson/src` contain nested `.git` directories, build/install/log artifacts, wrapper config build output, and third-party package code. They are better kept as external dependencies instead of committed inside this rollback.

## Models

Current workspace model location:

- `models/source/`
- `models/jetson16/standard/`
- `models/jetson16/hero/`
- `models/jetson64/`

Jetson model location:

- `jetson/AI-models/`

Jetson model files:

- `rt-detr.onnx`
- `rt-detr.pt`
- `rtdetr-l.engine`
- `yolo26_bbox.engine`
- `yolo26_bbox.onnx`
- `yolo26_bbox.pt`
- `yolov11_blur_optimization.engine`
- `yolov11_blur_optimization.onnx`
- `yolov11_blur_optimization.pt`
- `yolov11n.engine`
- `yolov11n.onnx`
- `yolov11n.pt`
- `yolov26_keypoints.engine`
- `yolov26_keypoints.onnx`
- `yolov26_keypoints.pt`
- `yolov8_op.engine`
- `yolov8_op.onnx`
- `yolov8n_p2.engine`
- `yolov8n_p2.onnx`
- `yolov8n_p2.pt`

Important compatibility note:

- TensorRT `.engine` files are target-specific. The Jetson folder is treated as Jetson 64 output.
- Jetson 16 launch files should point at a separate Jetson 16 engine path.
- `.onnx` and `.pt` sources are portable inputs for rebuilding the Jetson 16 engines on that device.

## Docs/media difference

Current `media/` contains:

- Old Markdown docs for the newer `auto_aim` refactor.
- LaTeX/Markdown course material.
- Old targeting notes.
- `media/output2.mp4`.

Jetson docs available:

- `jetson/Readme.md`
- `jetson/src/auto_aim_2/MODIFICHE_AUTO_AIM.md`
- README files under `armor_tracker`, `rm_trajectory`, `zed-ros2-interfaces`, `zed-ros2-wrapper`, and `isaac_ros_common`.

The requested rollback keeps the video, removes old current docs, moves the Jetson README into `media/`, and rewrites docs around the new `autoaim` package and launch/model layout.

## Launch difference

Current launch files:

- `src/launch_pkg/launch/debug.launch.py`
- `src/launch_pkg/launch/debug_targeting.launch.py`
- `src/launch_pkg/launch/fake_launch_file.launch.py`
- `src/launch_pkg/launch/realsense_only_det.launch.py`
- `src/launch_pkg/launch/realsense_only_det_adam.launch.py`
- `src/launch_pkg/launch/realsense_only_det_flip.launch.py`
- `src/launch_pkg/launch/realsense_only_det_flip_adam.launch.py`
- `src/launch_pkg/launch/realsense_only_det_image_only.launch.py`
- `src/launch_pkg/launch/realsense_only_det_rtdetr.launch.py`
- `src/launch_pkg/launch/test_realsense_optimized_launcher.launch.py`
- `src/bbox_filter_node/filter/launch/__*.launch.py`

Jetson launch file:

- `jetson/src/auto_aim_2/launch/auto_aim_launch.py`

Requested final launch layout:

- Short, reachable launch names.
- Separate Jetson 64 and Jetson 16 folders.
- Same node set and parameters for both platforms.
- Platform-specific model path only.
