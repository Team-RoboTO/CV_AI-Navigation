# Stable head-motion navigation patch

Files modified:
- `config/nav2_params.yaml`
- `config/mid360.yaml`
- `launch/sensors.launch.py`
- `nav2_new/scan_memory_filter.py`
- `nav2_new/set_initial_pose.py`

Main behavior:
1. Global costmap is static map only; no obstacle layer from the head-mounted Livox.
2. Local costmap and AMCL use `/scan_nav`.
3. `/scan_nav` has very short memory and is gated by `/turret/cmd`.
   When CV is active (`linear.z >= 0.5`) or turret yaw command changes, the node publishes an all-inf clearing scan and resets memory.
4. Initial pose uses timestamp zero to avoid AMCL future extrapolation at startup.
5. FAST-LIO is slightly less aggressive with sparse Livox points: `point_filter_num: 1`, `blind: 0.10`, `det_range: 30.0`.

Install by copying these files over the same paths in `/root/nav2_ws/src/nav2_new`, then rebuild:

```bash
cd /root/nav2_ws
colcon build --packages-select nav2_new --symlink-install
source install/setup.bash
```
