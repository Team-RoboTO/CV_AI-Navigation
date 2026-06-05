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

Default engines:

```text
standard.launch.py -> models/jetson16/standard/yolov26_keypoints.engine
hero.launch.py     -> models/jetson16/hero/yolov26_keypoints.engine
sentry.launch.py   -> models/jetson64/yolov26_keypoints.engine
```

TensorRT engines can be rejected when JetPack, TensorRT, CUDA, or GPU architecture differs from the device that built them. Rebuild on the target Jetson if TensorRT fails to deserialize an engine.

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
