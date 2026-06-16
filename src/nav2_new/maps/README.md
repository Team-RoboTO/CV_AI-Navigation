# Maps

This folder stores maps of the competition arena and the lab.

## Format

Each map is **two files with the same base name**:

| File | Content |
|---|---|
| `my_map.pgm` | 2D occupancy grid as a grayscale PGM (0 = occupied, 255 = free) |
| `my_map.yaml` | Metadata: resolution, origin, thresholds |

Nav2's `map_server` loads the `.yaml`, which references the `.pgm`.

## Typical workflow

```bash
# 1. Drive around to build the map
ros2 launch nav2_new slam.launch.py

# 2. Save the map (from another terminal)
ros2 run nav2_new save_map --ros-args -p name:=lab_map

#    → creates lab_map.pgm + lab_map.yaml in ~/roboto_maps/

# 3. Copy into the package (or load from home directly)
cp ~/roboto_maps/lab_map.* ~/nav2_ws/src/nav2_new/maps/

# 4. Rebuild and use
colcon build --packages-select nav2
ros2 launch nav2_new lab_test.launch.py \
    map:=$(ros2 pkg prefix nav2)/share/nav2_new_new/maps/lab_map.yaml
```

## Included maps

- `arena_map.{pgm,yaml}` — placeholder, replace with the official RoboMaster arena map before competition.

## Tips

- Keep maps under ~500×500 px — SLAM Toolbox loop-closure slows quadratically.
- If the PGM looks rotated/mirrored, your TF tree is wrong — don't try to fix it by rotating the PGM, fix the LiDAR TF in `launch/sensors.launch.py`.
