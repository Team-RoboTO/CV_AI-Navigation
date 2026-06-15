# Package layout

```text
src/autoaim/
  CMakeLists.txt
  package.xml
  include/autoaim/
  src/
    serial_bridge.cpp
  msg/
  launch/
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
autoaim_node   C++ tracker and command node
zed_detector.py   ZED X Mini + TensorRT detector
serial_bridge     default C++ microcontroller serial bridge
viewer_node.py    debug image overlay
```

The only ROS package in `src/` is `autoaim`. Internal node names and runtime
topics retain their existing `autoaim` naming for compatibility.
