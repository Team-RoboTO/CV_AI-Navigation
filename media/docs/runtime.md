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

## Overrides

```bash
ros2 launch autoaim sentry.launch.py serial_port:=/dev/ttyACM1
ros2 launch autoaim sentry.launch.py engine_path:=/absolute/path/to/model.engine
```

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
