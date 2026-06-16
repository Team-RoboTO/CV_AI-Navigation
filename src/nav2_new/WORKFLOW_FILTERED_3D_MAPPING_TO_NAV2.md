# Filtered 3D mapping → 2D Nav2 workflow

## 0. Build

```bash
cd ~/nav2_ws
colcon build --symlink-install
source /opt/ros/humble/setup.bash
source ~/nav2_ws/install/setup.bash
```

## 1. Start filtered FAST-LIO mapping

The package now filters `/livox/lidar` into `/livox/lidar_filtered` before FAST-LIO.
The raw Livox topic remains available for debugging.

```bash
ros2 launch nav2_new sensors.launch.py mount:=normal rviz:=true
```

Default filter:

```text
range: 0.45 m to 6.0 m
z:    -0.55 m to 1.50 m
```

For a narrow corridor, try shorter range:

```bash
ros2 launch nav2_new sensors.launch.py mount:=normal rviz:=true \
  filter_max_range:=4.0 filter_min_z:=-0.50 filter_max_z:=1.20
```

RViz:

```text
Fixed Frame: odom
Show: /Laser_map, /cloud_registered, /path, TF
```

Wait 5-10 seconds still before moving.

## 2. Save 3D PCD map

While FAST-LIO is still running:

```bash
mkdir -p /root/nav2_ws/maps
ros2 run nav2_new save_pcd_once --ros-args \
  -p topic:=/Laser_map \
  -p out_dir:=/root/nav2_ws/maps \
  -p prefix:=fastlio_map
```

Check:

```bash
ls -lh /root/nav2_ws/maps
```

## 3. Convert PCD to 2D Nav2 map

```bash
ros2 run nav2_new pcd_to_2d_map /root/nav2_ws/maps/YOUR_FASTLIO_MAP.pcd \
  --out /root/nav2_ws/maps/robomaster_map \
  --resolution 0.05 \
  --z-min -0.10 \
  --z-max 0.30 \
  --min-hits 3 \
  --inflate 1
```

If noisy:

```bash
ros2 run nav2_new pcd_to_2d_map /root/nav2_ws/maps/YOUR_FASTLIO_MAP.pcd \
  --out /root/nav2_ws/maps/robomaster_map \
  --resolution 0.05 \
  --z-min 0.00 \
  --z-max 0.35 \
  --min-hits 5 \
  --inflate 1
```

Outputs:

```text
/root/nav2_ws/maps/robomaster_map.yaml
/root/nav2_ws/maps/robomaster_map.pgm
```

## 4. Run localization/navigation

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source ~/nav2_ws/install/setup.bash
ros2 launch nav2_new sensors.launch.py mount:=normal rviz:=false
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source ~/nav2_ws/install/setup.bash
ros2 launch nav2_new navigation.launch.py \
  map:=/root/nav2_ws/maps/robomaster_map.yaml \
  use_sim_time:=false
```

## 5. RViz from laptop

On Jetson and laptop, use the same ROS settings:

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

On laptop:

```bash
rviz2
```

RViz:

```text
Fixed Frame: map
Add Map /map
Add LaserScan /scan
Add TF
Add Path /plan
Add Path /local_plan
Add Global Costmap
Add Local Costmap
```

Set pose:

```text
2D Pose Estimate
```

Adjust until `/scan` overlaps the map walls.

Send goal:

```text
2D Goal Pose
```

Start with a 30-50 cm goal in clear free space.

## 6. Command topic chain

Expected:

```text
controller_server -> /cmd_vel_nav
velocity_smoother -> /cmd_vel_nav_raw
cmd_vel_xy_only   -> /cmd_vel_NAV
```

Check:

```bash
ros2 topic info /cmd_vel_nav -v
ros2 topic info /cmd_vel_nav_raw -v
ros2 topic info /cmd_vel_NAV -v
ros2 topic echo /cmd_vel_NAV
```

If robot moves sideways/opposite, relaunch navigation with:

```bash
ros2 launch nav2_new navigation.launch.py \
  map:=/root/nav2_ws/maps/robomaster_map.yaml \
  cmd_vel_rotate_yaw_deg:=-90.0
```
