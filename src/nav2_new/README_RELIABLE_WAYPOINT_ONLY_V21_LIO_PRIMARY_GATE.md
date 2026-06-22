# V21 – FAST-LIO primary + micro stationary gate

This version intentionally restores the localization behaviour of the original package:

- `odom -> base_link` x/y/yaw are primarily relayed from FAST-LIO `camera_init -> body`, planarized.
- `/micro_status` is **not** integrated to create position.
- `/micro_status[2]` and `/micro_status[3]` are used only to decide if the chassis is translating.
- If `hypot(vx, vy)` is below `stationary_vxy_threshold`, x/y are frozen exactly while yaw still follows FAST-LIO.
- When translation starts again, the accumulated stationary fake drift is kept as a correction offset, so there is no jump.

Why this version exists:
The later experimental versions integrated `vx/vy` directly. On this robot those velocities are not a sufficiently calibrated global pose source, so any yaw/scale/reference mismatch produced diagonal drift at high speed. The original FAST-LIO pose followed the robot motion better; it only needed the head-rotation fake x/y removed.

Expected behavior:

- stopped + rotate head/reference: x/y fixed, yaw changes;
- moving: x/y/yaw follow FAST-LIO like the original package;
- no raw FAST-LIO ghost TF in public `/tf` by default.

Useful launch overrides:

```bash
stationary_vxy_threshold:=0.06
micro_vy_sign:=-1.0
publish_fastlio_debug_tf:=false
```

If right/left become inverted again, set:

```bash
micro_vy_sign:=1.0
```
