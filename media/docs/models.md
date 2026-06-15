# Models

Model files live at the workspace root, outside `src/`:

```text
models/source/
models/jetson64/
models/jetson16/standard/
models/jetson16/hero/
```

`source/` keeps portable `.pt` and `.onnx` files. The Jetson/profile folders keep TensorRT `.engine` files used by launch defaults.

The RT-DETR `.pt` and `.onnx` source files from the Jetson archive are larger than 100 MB, so they stay out of the package tree unless the repository is moved to Git LFS.

Runtime default:

```text
/workspaces/isaac_ros-dev/AI-models/yolov26_keypoints.engine
```

All three launch profiles use this external path by default. The package does
not install TensorRT engines, so `colcon build` does not copy large binaries.
Workspace-level model files remain source/archive assets and explicit override
options.

Both detector paths (RealSense C++ and ZED Python) load the **same** keypoint
engine via the same `engine_path` resolution order, so only one engine is
needed per device.

TensorRT engines can be rejected when JetPack, TensorRT, CUDA, or GPU
architecture differs from the device that built them. Rebuild on the target
Jetson if TensorRT fails to deserialize an engine. (Observed during this work:
the device runs TensorRT 10.7; an engine built with a newer TensorRT fails to
deserialize with "old deserialization call on a newer plan file" — rebuild it
on the target.)

Launch override:

```bash
ros2 launch autoaim standard.launch.py \
  engine_path:=/home/robomaster/models/yolov26_keypoints.engine
```

Environment override for running `zed_detector.py` directly:

```bash
AUTOAIM_ENGINE_PATH=/home/robomaster/models/yolov26_keypoints.engine \
ros2 run autoaim zed_detector.py
```
