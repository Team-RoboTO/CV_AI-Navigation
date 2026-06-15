# Package layout

```text
src/autoaim/
  CMakeLists.txt
  package.xml
  include/autoaim/
    realsense_preprocess.h     # CUDA preprocess wrapper decl
  src/
    serial_bridge.cpp
    realsense_detector.cpp     # C++ RealSense + TensorRT detector
    realsense_preprocess.cu    # CUDA letterbox/normalize kernel
  msg/
  launch/
  config/sensors/
    zed.yaml                   # ZED camera/detector params
    realsense.yaml             # RealSense camera/detector params
  viewer_node.py
  zed_detector.py
models/
  source/
  jetson16/
    standard/
    hero/
  jetson64/
```

Removed workspace packages:

- `bbox_filter_node`
- `cmd_vel_subscriber`
- `launch_pkg`
- previous C++ refactor package

Runtime entrypoints:

```text
autoaim_node       C++ tracker and command node
realsense_detector C++ RealSense + TensorRT detector (default competition path)
zed_detector.py    ZED X Mini + TensorRT detector (camera:=zed)
serial_bridge      default C++ microcontroller serial bridge
viewer_node.py     debug image overlay
```

The only ROS package in `src/` is `autoaim`. Internal node names and runtime
topics retain their existing `autoaim` naming for compatibility.
