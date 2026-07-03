# autoaim_v2 — deployment on the robot (Jetson AGX Orin + STM32H723)

Supports both **ZED X Mini** and **RealSense D455** cameras. The serial protocol
is byte-identical to the old bridge: **no firmware change is required** to
deploy. The old `autoaim` pipeline stays installed as a fallback
(`ros2 launch autoaim standard.launch.py`).

## 0. Prerequisites on the Jetson

- JetPack 6.x (CUDA 12, TensorRT 10) or JetPack 5.1 (CUDA 11, TensorRT 8.5+)
- **ZED X Mini**: ZED SDK **≥ 4.0** for the matching JetPack (SDK 4+ uses the
  CUDA primary context, which this node relies on)
- **RealSense D455**: librealsense2 dev (`apt install librealsense2-dev` or
  build from source for aarch64 — the Intel apt repo ships x86 only)
- ROS 2 Humble (bare-metal or the existing Isaac ROS container — same as the
  old pipeline)
- The TensorRT engine `yolov26_keypoints.engine` built **on the Jetson**
  (TensorRT engines are not portable across GPU/TRT versions):

```bash
# on the Jetson, from the training .pt:
pip install ultralytics
yolo export model=AI-models/yolov26_keypoints.pt format=engine imgsz=640 \
     device=0 half=True nms=False dynamic=False
# -> produces yolov26_keypoints.engine; put its path in config/standard_1v1.yaml
```

`half=True` (fp16 compute) is fine — IO stays fp32, which is what the node
requires. Both raw (`nms=False`) and end2end (`nms=True`) exports work; raw is
recommended (the node's own NMS is faster than the plugin at 3 classes).

## 1. Build

```bash
cd ~/ros_ws_vinceremo            # or /workspaces/isaac_ros-dev inside the container
source /opt/ros/humble/setup.bash
colcon build --packages-up-to autoaim_v2 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

The CMake auto-detects CUDA + TensorRT + camera SDKs. On the Jetson you must
see at least one of:

```
autoaim_v2: ZED + TensorRT found (zed_trt mode available)
autoaim_v2: RealSense + TensorRT found (rs_trt mode available)
```

Both can be available simultaneously — the `input_mode` parameter selects which
camera is used at runtime. If it says `ros_topics input mode only`, the camera
SDK or TensorRT dev headers are missing — fix that first
(`ls /usr/local/zed`, `dpkg -l | grep nvinfer`, `dpkg -l | grep librealsense`).

Run the unit tests once per machine: `colcon test --packages-select autoaim_v2
&& colcon test-result` → `25 tests, 0 errors, 0 failures`.

### Full-pipeline bench check (no hardware needed)

Validates ingestion → PnP + yaw search → EKF → regimes end-to-end with a
synthetic enemy. Terminal 1:

```bash
ros2 run autoaim_v2 aim_node --ros-args -p input_mode:=ros_topics \
    -p sim_gimbal:=true -p fixed_target_class:=0 \
    -p debug_enable:=true -p debug_every:=1
```

Terminal 2 — all three must print `PASS`:

```bash
python3 src/autoaim_v2/tools/replay_synthetic.py --omega 25 --duration 5  # TIMED
python3 src/autoaim_v2/tools/replay_synthetic.py --omega 0 --vy 1.0 --duration 5  # TRACK
python3 src/autoaim_v2/tools/replay_synthetic.py --omega 6 --duration 5   # SWEEP
```

## 2. Serial / permissions / performance

```bash
sudo usermod -aG dialout $USER        # serial access without sudo
sudo systemctl enable jetson-fan-max  # keep using the existing fan service

# Max clocks (do this in the startup script, it resets on boot):
sudo nvpmodel -m 0 && sudo jetson_clocks
```

Give the process realtime priority + timer precision (recommended — the timed
fire regime sleeps to sub-ms deadlines). Add to `/etc/security/limits.d/99-aim.conf`:

```
<your-user>  -  rtprio  90
```

## 3. First power-on checklist (safety order)

1. **Feeder empty, flywheels off**, robot on blocks.
2. Launch with your camera:
   - ZED: `ros2 launch autoaim_v2 standard_1v1.launch.py`
   - RealSense: `ros2 launch autoaim_v2 standard_rs.launch.py`
   (defaults: `shooting_active:=false` — the node never sends shoot=1).
3. Check the micro link: gimbal should hold still; move the gimbal by hand in
   manual mode and confirm `/aimv2/debug` (launch with `debug:=true`) updates.
4. Show it an armor plate (powered lightbars or a printed target at ~2 m).
   In `debug:=true`, `/aimv2/debug` keeps the compact legacy float array
   `[regime, tracker_state, face, p_hit, omega, dist, ...]`, and
   `/aimv2/debug_frame` publishes the full typed debug frame for rosbag. The
   `tracker_state` field must go 1 (DETECTING) → 2 (TRACKING), and
   `target_distance` must match a tape measure within ~5 cm.
5. Switch the micro to auto-aim mode: the gimbal must track the plate as you
   carry it around. If yaw runs AWAY from the target, flip `gimbal.yaw_sign`.
   If pitch mirrors (looks up when the plate is low), see
   `micro_pitch_feedback_opposite_sign` in INSTRUCTIONS.md — the semantics are
   unchanged from the old pipeline.
6. Only then: pellets in, `shooting_active:=true`, static target, single
   bursts. Then calibrate (next section).

### Debug topics / rosbag

For full RealSense bench debug:

```bash
ros2 launch autoaim_v2 standard_rs.launch.py \
  debug:=true debug_image:=true viewer:=true shooting_active:=false
```

Useful topics:

| topic | type | purpose |
|-------|------|---------|
| `/aimv2/debug_frame` | `autoaim/msg/AimDebugFrame` | full per-frame detector/PnP/tracker/aimer/serial state |
| `/aimv2/debug` | `std_msgs/Float32MultiArray` | compact legacy debug vector used by tests |
| `/aimv2/debug_image` | `sensor_msgs/Image` | RealSense image in the same orientation used by the model |
| `/aimv2/viewer_image` | `sensor_msgs/Image` | annotated viewer output for RViz/image_view |
| `/aimv2/markers` | `visualization_msgs/MarkerArray` | optional RViz world markers when `debug_markers:=true` |

Rosbag the useful debug stream:

```bash
ros2 bag record \
  /aimv2/debug_frame \
  /aimv2/debug \
  /aimv2/debug_image \
  /aimv2/viewer_image \
  /aimv2/markers \
  /camera_info
```

## 4. Calibration order (do them in this order)

| step | what | how |
|------|------|-----|
| 1 | `bullet_speed` | Chrono / referee readout, 10-shot average. Update yaml. |
| 2 | `pitch_offset_deg`, `yaw_offset_deg` | Static plate at 3 m, 20 shots, mark the group center. +pitch_offset if the group is high, +yaw_offset if it is left. |
| 3 | `barrel_offset_y/z` | Repeat step 2 at 1.0 m. If the group moved vs 3 m, the residual is parallax: adjust barrel_offset (offset ≈ miss·d₁·d₂/(d₂−d₁) — or just iterate). |
| 4 | `feeder_delay` | 240 fps phone video of feeder LED/wheel + muzzle flash, or: aim at a spinner in TIMED mode and scan feeder_delay in 5 ms steps until hits peak. |
| 5 | `actuation_latency` | Strafe test: robot translating at ~1.5 m/s, shots trailing → increase in 5 ms steps; leading → decrease. |
| 6 | fire gates | Tighten `p_spray_min` (0.5→0.65) and `fire_lock_*` once groups are tight. |

## 5. Systemd autostart

```bash
sudo cp src/autoaim_v2/deploy/aimv2.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable aimv2
```

The service runs `deploy/aimv2_start.sh` which sets clocks, waits for
`/dev/ttyACM0`, sources the overlay and launches with `shooting_active:=true`.
Edit the script if your workspace lives elsewhere (container users: mirror the
old `autoaim_auto_startup/standard/` pattern — the service just needs to run
the same launch line inside the container).

## 6. Match-day switches

| switch | where | match value |
|--------|-------|-------------|
| `shooting_active` | launch arg | `true` |
| `input_mode` | launch arg | `zed_trt` or `rs_trt` (matches your camera) |
| `rotating_chassis` | yaml | your call (spin-to-win) |
| `fixed_target_class` | yaml | `-1` (auto from referee color) |
| `debug_enable` | launch arg `debug` | `false` |
| `ego_velocity_available` | yaml | `true` only if micro vx/vy validated |

## 7. What to watch if something is off

- **Tracks but every shot misses a spinner consistently early/late** →
  `feeder_delay` wrong (step 4). Each 5 ms ≈ 30 mm at 200 RPM.
- **Shots trail a strafing enemy** → increase `actuation_latency`.
- **Vertical stringing at long range** → re-measure `bullet_speed`; check
  `drag_k` is 0.0196 (not 0).
- **Gimbal oscillates on a static target** → your gimbal PID; reduce
  `cmd_deadband_*` only after the PID is stiff.
- **p_hit stays ~0 on a visible plate** → tracker not TRACKING: check
  `/aimv2/debug[1]`, likely wrong enemy color (`fixed_target_class`).
- **`camera stalled — reopening` spam** → GMSL cable/power; the node
  self-recovers but fix the cable.

## 8. A/B against the old pipeline

Run the old Python detector + new tracker to isolate detector vs tracker
changes:

```bash
ros2 launch autoaim standard.launch.py camera:=zed &        # old detector only
ros2 run autoaim_v2 aim_node --ros-args -p input_mode:=ros_topics \
  --params-file src/autoaim_v2/config/standard_1v1.yaml
```

(Disable the old `autoaim_node` in that launch or remap its `/cmd_vel_AI` away
so only one node commands the serial bridge — or simply unplug serial and
watch `/aimv2/debug`.)

## 9. Optional micro patch: gimbal feedforward

With `send_gimbal_feedforward: true`, TX[4]/TX[5] carry yaw/pitch rate
feedforward [rad/s]. On the micro (`gimbal_control.c`, GIMBAL_AUTO_AIM case):

```c
// after the slew-limit block:
gimbal.u[0] += k_ff_cv * fwd_bwd_cv;   // yaw rate feedforward, tune k_ff_cv
```

This closes the last 10-20 ms of tracking lag on fast movers. Entirely
optional; everything works with it disabled.
