# Runtime

## Build

```bash
cd /home/andreas/ros_ws
colcon build --packages-select autoaim --symlink-install
source install/setup.bash
```

## Launch

```bash
ros2 launch autoaim standard.launch.py
ros2 launch autoaim hero.launch.py
ros2 launch autoaim sentry.launch.py
```

The package intentionally exposes only these three launch files:

```text
src/autoaim/launch/standard.launch.py
src/autoaim/launch/hero.launch.py
src/autoaim/launch/sentry.launch.py
```

## Camera selection

All three profiles default to the **RealSense** C++ detector. Override with
`camera:=zed` to use the ZED detector instead:

```bash
ros2 launch autoaim standard.launch.py                 # RealSense (default)
ros2 launch autoaim hero.launch.py                     # RealSense (default)
ros2 launch autoaim standard.launch.py camera:=zed     # ZED path
```

Per-camera parameters live in YAML (camera/detector only — robot calibration
stays in the autoaim node params inside the launch files):

```text
src/autoaim/config/sensors/realsense.yaml
src/autoaim/config/sensors/zed.yaml
```

## Overrides

```bash
ros2 launch autoaim sentry.launch.py serial_port:=/dev/ttyACM1
ros2 launch autoaim sentry.launch.py engine_path:=/absolute/path/to/model.engine
ros2 launch autoaim sentry.launch.py camera:=zed
```

All launch profiles use the C++ `serial_bridge` executable. It is the only
installed serial transport.

The prediction horizon uses measured capture-to-aim latency plus
`actuation_latency`. Set `use_measured_latency:=false` in a launch profile only
for fixed-`time_bias` comparison tests.

## Sanity checks

```bash
ros2 topic hz /detector/armors_keypoints
ros2 topic echo /micro_status --once
ros2 topic echo /cmd_vel_AI --once
ros2 topic hz /tracker/debug_image
```

RViz image topic:

```text
/tracker/debug_image
```
