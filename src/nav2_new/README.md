# nav2

Complete ROS 2 Humble navigation stack for a RoboMaster competition robot:

- **Livox Mid-360** LiDAR (flipped mounting)
- **FAST_LIO** for odometry
- **Nav2** stack with AMCL localization and combat-tuned costmaps
- **SLAM Toolbox** for building maps
- **Strategic waypoint manager** — team-aware, runtime-switchable
- **Game state FSM** — automatic strategy changes driven by `/micro_status`
- **Turret yaw multiplexer** — arbitrates between CV aim and chassis heading
- **`cmd_vel` safety** — watchdog, e-stop, hard velocity clamp
- **Integration with a separate CV container** via ROS 2 DDS on host net

---

## Contents

1. [Quick reference](#quick-reference)
2. [Install on a fresh 64 GB Jetson](#1-install-on-a-fresh-64-gb-jetson)
3. [Docker setup & CV container integration](#2-docker-setup--cv-container-integration)
4. [Livox network configuration](#3-livox-network-configuration)
5. [LiDAR flip correction](#4-lidar-flip-correction)
6. [Build & test the package](#5-build--test-the-package)
7. [Build a map with SLAM](#6-build-a-map-with-slam)
8. [Capture waypoints (with names)](#7-capture-waypoints-with-names)
9. [Strategies & arena paths](#8-strategies--arena-paths)
10. [Micro status protocol & game FSM](#9-micro-status-protocol--game-fsm)
11. [Turret yaw multiplexer](#10-turret-yaw-multiplexer)
12. [Running a competition match](#11-running-a-competition-match)
13. [Testing offline (no hardware)](#12-testing-offline-no-hardware)
14. [Troubleshooting](#13-troubleshooting)

---

## Quick reference

```bash
# Build a map
ros2 launch nav2_new slam.launch.py
ros2 run nav2_new save_map --ros-args -p name:=arena_01

# Capture named waypoints in RViz
ros2 run nav2_new waypoint_editor
./scripts/set_waypoint_name.sh blue_tunnel_1
# ... then click "2D Goal Pose" in RViz

# Test navigation on the map (manual goals via RViz)
ros2 launch nav2_new lab_test.launch.py map:=~/roboto_maps/arena_01.yaml

# Full competition pipeline (real hardware)
ros2 launch nav2_new competition.launch.py \
    map:=~/roboto_maps/arena_final.yaml team:=blue

# Integration test without hardware (fake micro + fake CV)
ros2 launch nav2_new test_pipeline.launch.py \
    map:=~/roboto_maps/lab_map.yaml team:=blue
```

---

## 1. Install on a fresh 64 GB Jetson (Docker, shares domain with CV container)

JetPack 6 = Ubuntu 22.04. Your CV container already runs at `ROS_DOMAIN_ID=0`. We'll build a separate Nav container that joins the same domain.

### Step 1 — Workspace layout on the Jetson host

```bash
# Create folders on the Jetson (not inside any container)
mkdir -p ~/roboto/nav2_ws/src
mkdir -p ~/roboto/roboto_maps
```

### Step 2 — Drop sources into `~/roboto/nav2_ws/src/`

You need 5 packages. This `nav2` one, plus the Livox / FAST_LIO / converter packages:

```bash
cd ~/roboto/nav2_ws/src

# 1. This package — unzip nav2_new.zip here:
unzip /path/to/nav2.zip    # creates nav2/

# 2. Livox SDK2 (C++ library)
git clone https://github.com/Livox-SDK/Livox-SDK2.git

# 3. Livox ROS 2 driver
git clone https://github.com/Livox-SDK/livox_ros_driver2.git

# 4. FAST_LIO for ROS 2
git clone https://github.com/Ericsii/FAST_LIO_ROS2.git

# 5. livox_converter — your team's CustomMsg→PointCloud2 converter
#    This is team-specific. Copy from your old repo if you have it, or
#    write a minimal version based on nav2/converter_node.py
#    reference in the README. If you don't have it, ask Gianlu/Nik.
```

Verify:
```bash
ls ~/roboto/nav2_ws/src/
# Expected: FAST_LIO_ROS2  Livox-SDK2  livox_converter  livox_ros_driver2  nav2
```

### Step 3 — Copy scripts to a convenient location

```bash
cp -r ~/roboto/nav2_ws/src/nav2_new/scripts ~/roboto/scripts
chmod +x ~/roboto/scripts/*.sh
```

### Step 4 — Build the Nav Docker image

The Dockerfile expects the build context to contain `./nav2_ws/src/`, so run `docker build` from `~/roboto`:

```bash
cd ~/roboto
docker build -t roboto/nav:latest \
    -f nav2_ws/src/nav2_new/docker/Dockerfile.jetson .
```

First build: 15–30 min. Subsequent rebuilds cache most of it.

### Step 5 — Configure the Jetson's ethernet port for the Livox

Run on the **host** (not in the container — `--net=host` means the container shares the host's NIC configuration anyway):

```bash
ip link   # find the interface name — probably eth0
sudo IFACE=eth0 ~/roboto/scripts/setup_livox_net.sh
```

Expected output:
```
✓ eth0 configured with 192.168.1.5/24
✓ LiDAR is reachable.
```

To make this persistent across reboots, create a systemd unit (see [Livox network configuration](#3-livox-network-configuration)).

### Step 6 — Run the Nav container

```bash
xhost +local:docker    # allow GUI apps from the container
~/roboto/scripts/run_nav_docker.sh
```

You should land in a shell inside the container at `/root/nav2_ws`, with ROS 2 already sourced. The default `ROS_DOMAIN_ID` is **0** — the same as your CV container.

### Step 7 — Verify CV ↔ Nav communication

Inside the Nav container:

```bash
echo $ROS_DOMAIN_ID          # should print 0
ros2 topic list              # should include CV container's topics
```

If you see topics like `/cv/target`, `/micro_status`, or whatever your CV publishes, you're connected. Similarly, from inside the CV container, `ros2 topic list` should now show Nav topics once you start them.

If one direction works but not the other → usually means the CV container is missing `--net=host`. Check its run command.

### Native install alternative

If you prefer a native install (no Docker), see [Appendix: Native install](#appendix-native-install) at the bottom.

---

## 2. Topic topology (Nav ↔ CV ↔ serial bridge ↔ STM32)

The CV container owns the serial bridge to the STM32 (a single node,
`serial_new_communication_USB_C_final.py`). That node is the ONLY publisher
talking to the STM32. It reads two Twist topics and publishes micro telemetry
as a single `Float32MultiArray`:

```
┌───────────────────────────┐     ┌──────────────────────────────────┐
│  Nav container             │     │  CV container                     │
│                            │     │                                   │
│  Nav2         ──/cmd_vel_NAV──►  │                                   │
│                            │     │  CV detection ──/cmd_vel_AI────►  │
│  waypoint_manager          │     │                                   │
│  game_state_manager        │     │  ┌──────── serial bridge ──────┐ │
│  micro_status_adapter  ◄───┼─/micro_status─── │ RX: reads STM32    │ │
│                            │     │  │ TX: packs & sends:          │ │
│                            │     │  │  AI yaw/pitch/shoot         │ │
│                            │     │  │  NAV x/y/yaw                │ │
│                            │     │  └─── USB serial ─── STM32 ────┘ │
└───────────────────────────┘     └──────────────────────────────────┘
```

### What the Nav container provides

| Topic | Direction | Type | Source |
|---|---|---|---|
| `/cmd_vel_NAV` | Nav → CV bridge | `geometry_msgs/Twist` | Nav2 controller |
| `/team` | adapter → FSM | `std_msgs/String` | unpacked from `/micro_status` |
| `/match_started` | adapter → FSM | `std_msgs/Bool` | unpacked from `/micro_status` |
| `/health` | adapter → FSM | `std_msgs/Int32` | unpacked from `/micro_status` |
| `/center_captured` | adapter → FSM | `std_msgs/Bool` | unpacked from `/micro_status` |
| `/strategy` | FSM → waypoint_manager | `std_msgs/String` | auto |

### What `/cmd_vel_NAV` looks like

Standard `geometry_msgs/Twist` in SI units:

| Field | Meaning | Unit | Typical range |
|---|---|---|---|
| `linear.x` | forward velocity | **m/s** | 0 to 0.4 in combat, 0 to 0.5 in lab |
| `linear.y` | lateral velocity | **m/s** | 0 for diff-drive; used by mecanum |
| `angular.z` | yaw rate | **rad/s** (CCW+) | ±1.2 combat, ±1.5 lab |

Nav2 publishes these values directly. The serial bridge reads them and
forwards to the STM32 as floats. **The STM32 is responsible for converting
m/s & rad/s to motor RPM** using its own kinematics model (wheelbase,
wheel radius, gear ratio).

Velocity limits are configured in `config/nav2_params_combat.yaml`:

```yaml
max_vel_x: 0.4        # m/s
max_vel_theta: 1.2    # rad/s
```

### How the rename to `/cmd_vel_NAV` works

Nav2's default output is `/cmd_vel`. We remap it globally at the composition
container in `launch/navigation.launch.py`:

```python
nav2_remappings = [
    ('/tf', 'tf'),
    ('/tf_static', 'tf_static'),
    ('/cmd_vel', cmd_vel_topic),          # default /cmd_vel_NAV
    ('/cmd_vel_smoothed', cmd_vel_topic),
]
```

All Nav2 nodes publish to `/cmd_vel_NAV`. To disable the rename for debugging:

```bash
ros2 launch nav2_new competition.launch.py cmd_vel_topic:=/cmd_vel
```

### The `/micro_status` packet format

Your serial bridge publishes `std_msgs/Float32MultiArray` with 6 floats:

| Index | Field | Meaning |
|---|---|---|
| 0 | color | 0.0 = red, 1.0 = blue |
| 1 | start | 1.0 = match running |
| 2 | health | 0-100 %HP |
| 3 | ammo | (unused by Nav) |
| 4 | center | 1.0 = we're capturing the center |
| 5 | resupply | (unused by Nav) |

The `micro_status_adapter` node unpacks this into the individual topics
consumed by `game_state_manager`. If your firmware team changes the TEAM
color convention (0/1 flipped), update `TEAM_MAP` in
`nav2/micro_status_adapter.py`.

### Turret yaw

The CV container owns yaw/pitch/shoot for the turret — the serial bridge
forwards AI values directly to the STM32. Nav does NOT publish `/turret/cmd`
by default, so `turret_yaw_mux` is disabled in `competition.launch.py`.
If you ever want Nav to keep the barrel pointed forward during idle (when CV
isn't tracking), launch with `enable_turret_mux:=true`.

### Optional: `cmd_vel_safety` watchdog

Off by default. Your serial bridge already loops at 100 Hz with the last-
received values, so if Nav2 stops publishing, the bridge just keeps sending
the last command. Safety against a runaway command is the STM32 firmware's
job — add a receive timeout there that zeros motor output if no fresh packet
arrives in ~100 ms.

If you DO want an extra software-level watchdog in the Nav container:

```bash
ros2 launch nav2_new competition.launch.py enable_cmd_vel_safety:=true
```

This inserts the safety node between Nav2 and `/cmd_vel_NAV`, with an
e-stop subscriber on `/emergency_stop`.

---

## 3. Livox network configuration

The Mid-360 communicates over ethernet. Default LiDAR IP is `192.168.1.1xx` (the `xx` is set by DIP switches on the sensor body). The host needs an IP on the same `/24` subnet.

### One-time setup

Run **on the Jetson host** (or inside the container if running with `--net=host`, which is identical):

```bash
# Find the ethernet interface name:
ip link
# Look for something like "eth0", "enP8p1s0", etc. — NOT the Wi-Fi one.

# Edit scripts/setup_livox_net.sh and change IFACE if needed, then:
./scripts/setup_livox_net.sh

# Or one-shot:
IFACE=eth0 HOST_IP=192.168.1.5 ./scripts/setup_livox_net.sh
```

### Make it persistent across reboots

Create a systemd unit at `/etc/systemd/system/livox-net.service`:

```ini
[Unit]
Description=Livox LiDAR Network Setup
After=network.target

[Service]
Type=oneshot
ExecStart=/home/user/nav2_new/scripts/setup_livox_net.sh
Environment=IFACE=eth0
Environment=HOST_IP=192.168.1.5

[Install]
WantedBy=multi-user.target
```

Then:

```bash
sudo systemctl daemon-reload
sudo systemctl enable livox-net.service
sudo systemctl start livox-net.service
```

### Configuring the driver

The `livox_ros_driver2` also needs to know which interface to use. Check your `msg_MID360_launch.py` for something like:

```python
ethernet_interface = 'eth0'
host_ip = '192.168.1.5'
```

These must match what `setup_livox_net.sh` configured.

---

## 4. LiDAR flip correction

The Mid-360 is mounted **upside-down** (rotated 180° about the X-axis). Two places agree:

| File | What | Value |
|---|---|---|
| `config/mid360.yaml` | FAST_LIO `extrinsic_R` | `[1, 0, 0,  0, -1, 0,  0, 0, -1]` |
| `launch/sensors.launch.py` | Static TF `base_link → livox_frame` | quaternion `(1, 0, 0, 0)` |

If your mount differs, pass `mount:=yaw180` or `mount:=normal` to any launch file AND edit `config/mid360.yaml` accordingly — **both must match** or AMCL won't converge.

Verify: run `rqt_tf_tree` and confirm `map → odom → base_link → livox_frame` all rotate together when you move the robot.

---

## 5. Build & test the package

```bash
cd ~/nav2_ws
colcon build --packages-select nav2 --symlink-install
source install/setup.bash

# Sanity check sensors alone:
ros2 launch nav2_new sensors.launch.py
ros2 topic hz /scan /odom   # both should publish
```

---

## 6. Build a map with SLAM

```bash
ros2 launch nav2_new slam.launch.py
# Drive / push the robot around slowly
# In another terminal:
ros2 run nav2_new save_map --ros-args -p name:=arena
# Writes: ~/roboto_maps/arena.pgm  +  ~/roboto_maps/arena.yaml
```

### Map format

Standard Nav2 map format:

- `.pgm` — grayscale occupancy grid (0=occupied, 254=free, 205=unknown)
- `.yaml` — metadata (resolution, origin, thresholds) — this is what Nav2's map server loads

### Mapping tips for the RoboMaster arena

- **Drive around the bumpers, not over them.** They'll show up as walls, which is what you want — the robot must avoid them.
- Drive **into** the tunnel during mapping so it's recognized as passable.
- Keep speed < 0.3 m/s and turning < 0.5 rad/s.
- Close a loop by returning to the start before saving.

### The bumpers

The black patterned areas at the bottom of the arena are **soft bumper pads**. Do NOT drive over them:

- They compress unevenly → robot tilts → LiDAR ground-plane fails
- FAST_LIO odometry drifts on soft surfaces
- Your wheel encoders will slip

Map them as obstacles by driving _around_ them, not on them.

---

## 7. Capture waypoints (with names)

The arena coordinates in `config/arena_waypoints.yaml` are estimates. Replace them with measurements from YOUR saved map.

```bash
# Terminal 1 — start navigation on your saved map:
ros2 launch nav2_new lab_test.launch.py \
    map:=~/roboto_maps/arena.yaml

# Terminal 2 — start the waypoint editor:
ros2 run nav2_new waypoint_editor
```

Now, for each waypoint you want to capture:

```bash
# Terminal 3 — set the name for the NEXT click:
./scripts/set_waypoint_name.sh blue_tunnel_1

# Then in RViz: click "2D Goal Pose" and drag at the desired location.
# → The editor logs "Captured 'blue_tunnel_1' at (x, y, yaw)"
# → A labelled green arrow appears at the click position.
# → ~/waypoints_captured.yaml is appended.

# Next waypoint:
./scripts/set_waypoint_name.sh blue_tunnel_2
# ... click in RViz ...

# And so on.
```

When done, open `~/waypoints_captured.yaml`, copy the entries, and paste them into `config/arena_waypoints.yaml` under `waypoints:`.

Rebuild the package: `colcon build --packages-select nav2`

---

## 8. Strategies & arena paths

Strategies live in `config/arena_waypoints.yaml` under `strategies:`. Each strategy lists waypoint names per team.

Predefined strategies:

| Strategy | What it does |
|---|---|
| `wait_at_spawn` | Hold the spawn pose (match not started) |
| `rush_center` | Drive to center via team-specific tunnel path |
| `hold_center_diagonal` | Loop around the 4 corners of the capture point |
| `retreat_to_spawn` | Back to spawn through the tunnel |
| `straight_to_center` | Direct path, ignores tunnel (lab-only) |

### Blue team tunnel path

For blue team, `rush_center` goes `spawn → blue_tunnel_2 → blue_tunnel_1 → blue_center_approach → center`. The two tunnel waypoints are positioned **inside** the narrow gap between the central walls, forcing Nav2's global planner to route through there rather than over the bumpers.

### Why the combat costmap matters here

Default inflation (0.55 m) + robot radius (0.25 m) = a planner that needs 1.6 m of clear width. The tunnel is ~0.7–1.0 m wide — it would be blocked.

`nav2_params_combat.yaml` uses `inflation_radius: 0.15` → planner needs only 0.8 m of clear width, which fits the tunnel with a small safety margin.

The competition launch uses `nav2_params_combat.yaml` by default; `lab_test.launch.py` uses the safer `nav2_params.yaml`.

---

## 9. Micro status protocol & game FSM

### The protocol

Your STM32 publishes a single JSON string on `/micro_status` containing all match telemetry:

```json
{
  "team": "blue",
  "health": 85,
  "match_state": "running",
  "center_captured": false,
  "enemy_captured_center": false,
  "match_time": 123
}
```

Fields:

| Field | Type | Values | Meaning |
|---|---|---|---|
| `team` | string | `"red"` `"blue"` | Our team color |
| `health` | int | 0-100 | %HP |
| `match_state` | string | `"waiting"` `"running"` `"paused"` `"ended"` | Referee match state |
| `center_captured` | bool | | Are WE capturing the center |
| `enemy_captured_center` | bool | | Is the enemy capturing it |
| `match_time` | int | seconds | Elapsed match time |

The `micro_status_parser` node splits this into individual topics (`/team`, `/health`, `/match_started`, `/center_captured`, etc.) for other nodes to subscribe to.

### The state machine

`game_state_manager` listens to those topics and drives the strategy:

```
 ┌─ WAITING ──── match_started=false or no team yet
 │   │
 │   │ match_started=true
 │   ▼
 ├─ RUSHING ──── going to center via tunnel
 │   │
 │   │ center_captured=true
 │   ▼
 ├─ HOLDING ──── patrolling diagonal of center
 │   │
 │   │ center_captured=false (lost it)
 │   ▼
 │   (back to RUSHING)
 │
 └─ RETREATING ── HP < 30%
     │
     │ HP > 70% (hysteresis prevents flapping)
     ▼
     (back to RUSHING)
```

Tune thresholds in `arena_waypoints.yaml` under `health:`.

### Override from the command line

Even with the FSM running, you can force a strategy change:

```bash
ros2 topic pub --once /strategy std_msgs/String "data: 'retreat_to_spawn'"
```

The FSM will override it again on the next tick based on its state, unless you pause the FSM by launching with `enable_game_state_fsm:=false`.

---

## 10. Turret yaw multiplexer

`turret_yaw_mux` is the **single publisher** of `/turret/cmd`. Logic:

- If `/cv/target` has been published within the last 300 ms → forward CV yaw/pitch, mode=1
- Otherwise → publish `yaw=0` (chassis-relative) + idle pitch, mode=0

Your CV node just publishes `/cv/target` whenever it detects something. Your micro driver subscribes to `/turret/cmd` as the single source of truth. They don't need to coordinate.

The Nav2 stack never talks to the turret topic — it only cares about `/cmd_vel`. So there's **no real conflict** — just different layers owning different topics.

Config: `config/turret_mux.yaml`. Key parameters:

- `cv_timeout` (0.3 s) — how long CV counts as "fresh"
- `cv_yaw_frame` (`"chassis"` or `"world"`) — what frame your CV publishes in
- `publish_rate` (50 Hz) — should match what your micro expects

### Testing the mux without real CV

```bash
# Sweep pattern (fake CV always active, yaw oscillates ±0.5 rad):
ros2 run nav2_new fake_cv_publisher --ros-args -p pattern:=sweep

# Burst pattern (1.5s on, 1.0s off — tests timeout fallback):
ros2 run nav2_new fake_cv_publisher --ros-args -p pattern:=burst

# Watch the mux output:
ros2 topic echo /turret/cmd
# Field z: 0.0 = chassis mode, 1.0 = CV mode
```

---

## 11. Running a competition match

```bash
# Blue team
ros2 launch nav2_new competition.launch.py \
    team:=blue \
    map:=~/roboto_maps/arena_final.yaml

# Red team
ros2 launch nav2_new competition.launch.py \
    team:=red \
    map:=~/roboto_maps/arena_final.yaml
```

The `team` arg is the default used until `/team` arrives from the real `/micro_status`. Once the micro publishes, the game FSM uses that value.

### What launches

1. Sensors (Livox + FAST_LIO)
2. Nav2 with `nav2_params_combat.yaml` (tight costmap for the tunnel)
3. `set_initial_pose` — seeds AMCL at the team's spawn
4. `waypoint_manager` — starts in `wait_at_spawn`
5. `micro_status_parser` — parses STM32 telemetry
6. `game_state_manager` — drives strategy transitions
7. `turret_yaw_mux` — runs alongside CV
8. `cmd_vel_safety` — watchdog + clamp
9. RViz

### Expected timeline at match start

```
t=0s:   launch file runs, everything starts
t=5s:   Nav2 active
t=10s:  initial pose published, AMCL converges
t=15s:  waypoint_manager waits on "wait_at_spawn" strategy
...
Match starts (micro publishes match_state="running")
t=X:    game_state_manager → WAITING → RUSHING → /strategy=rush_center
t=X+0.5s: waypoint_manager receives it, starts navigating
```

If HP drops mid-match:

```
HP=25:  game_state_manager → RUSHING → RETREATING → /strategy=retreat_to_spawn
        waypoint_manager cancels current path, starts retreat
HP=75:  game_state_manager → RETREATING → RUSHING → /strategy=rush_center
```

---

## 12. Testing offline (no hardware)

End-to-end integration test with fake micro + fake CV:

```bash
ros2 launch nav2_new test_pipeline.launch.py \
    map:=~/roboto_maps/lab_map.yaml team:=blue
```

This launches the full competition pipeline + fake publishers. To drive test events:

```bash
# Start the match — the FSM transitions to RUSHING
ros2 topic pub --once /fake_micro/start_match std_msgs/Bool "data: true"

# Simulate capturing the center — FSM goes to HOLDING
ros2 topic pub --once /fake_micro/center_captured std_msgs/Bool "data: true"

# Simulate damage — FSM goes to RETREATING
ros2 topic pub --once /fake_micro/set_hp std_msgs/Int32 "data: 20"

# Recover — FSM returns to RUSHING
ros2 topic pub --once /fake_micro/set_hp std_msgs/Int32 "data: 85"
```

Watch the logs and RViz to see the robot's behavior change.

---

## 13. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| AMCL won't converge | LiDAR TF wrong, or map doesn't match reality | Check `rqt_tf_tree`. Confirm `mount:=` matches physical mounting. Verify `extrinsic_R` in `mid360.yaml` agrees with the launch TF |
| `/odom` published but twist stays zero | Known FAST_LIO issue | Controller still works in `OPEN_LOOP` mode. To fix, patch FAST_LIO to fill twist from EKF |
| Topics don't cross between Nav and CV containers | Different `ROS_DOMAIN_ID`, or missing `--net=host` | Both must be identical. See [docker/README.md](docker/README.md) |
| Robot plans a path but doesn't move | `/cmd_vel_nav` not reaching motor driver via safety | Check `ros2 topic echo /cmd_vel_nav` and `/cmd_vel` — both should be nonzero when navigating |
| Robot plans _around_ the tunnel instead of through it | Inflation too wide | You're using lab params (`nav2_params.yaml`). Use `nav2_params_combat.yaml` |
| Costmap shows the tunnel as blocked even with combat params | Map occupancy bleeding through | Re-map with slower motion. In RViz check the static map vs. live scan — if static map walls are thicker than reality, lower `occupied_thresh` in the map yaml |
| `fake_micro_status` runs but FSM stays in WAITING | `/team` not published yet | The default `team` parameter is used until `/team` arrives. Check `ros2 topic echo /team` |
| Turret twitches when CV loses target | `cv_timeout` too short | Increase `cv_timeout` in `turret_mux.yaml` to 0.5 s |
| Turret stays at last position instead of returning to forward | Mux not publishing chassis fallback | Check `cv_yaw_frame` param matches your CV convention |
| Livox "no data" error | Network not configured | Run `./scripts/setup_livox_net.sh`, verify `ping 192.168.1.xxx` |

### Useful debug commands

```bash
# TF tree
ros2 run rqt_tf_tree rqt_tf_tree

# All running nodes
ros2 node list

# Active strategy + game state
ros2 topic echo /waypoint_manager/status
ros2 topic echo /game_state_manager/state

# Current AMCL pose
ros2 topic echo /amcl_pose --once

# Turret mux output (at 50 Hz)
ros2 topic hz /turret/cmd

# Emergency stop
ros2 topic pub --once /emergency_stop std_msgs/Bool "data: true"
ros2 topic pub --once /emergency_stop std_msgs/Bool "data: false"
```

---

## Package layout

```
nav2/
├── docker/
│   ├── Dockerfile.jetson             # Self-contained Jetson image
│   └── README.md                     # Multi-container setup guide
├── scripts/
│   ├── run_nav_docker.sh             # Docker run helper
│   ├── setup_livox_net.sh            # Configure Jetson ethernet for LiDAR
│   └── set_waypoint_name.sh          # Set waypoint name for next RViz click
├── config/
│   ├── nav2_params.yaml              # Nav2 — lab mode (wide inflation)
│   ├── nav2_params_combat.yaml       # Nav2 — combat mode (tunnel-friendly)
│   ├── slam_toolbox.yaml
│   ├── mid360.yaml                   # FAST_LIO + LiDAR flip
│   ├── arena_waypoints.yaml          # Waypoints + strategies + health thresholds
│   └── turret_mux.yaml               # Turret yaw mux params
├── launch/
│   ├── sensors.launch.py             # LiDAR pipeline, configurable mount
│   ├── slam.launch.py                # Build a map
│   ├── navigation.launch.py          # Nav2 + AMCL
│   ├── lab_test.launch.py            # Interactive goal setting
│   ├── competition.launch.py         # Full match pipeline
│   └── test_pipeline.launch.py       # Integration test w/ fake micro + fake CV
├── maps/
│   └── arena_map.{pgm,yaml}          # Placeholder — replace with real map
├── rviz/
│   ├── navigation.rviz
│   └── slam.rviz
└── nav2/
    ├── waypoint_manager.py           # Strategy executor
    ├── waypoint_editor.py            # Named waypoint capture from RViz
    ├── set_initial_pose.py           # Seed AMCL from spawn config
    ├── turret_yaw_mux.py             # CV/chassis yaw arbiter
    ├── cmd_vel_safety.py             # Watchdog, e-stop, clamp
    ├── micro_status_parser.py        # JSON /micro_status → topics (optional)
    ├── micro_status_adapter.py       # Float32MultiArray → topics (matches your serial bridge)
    ├── game_state_manager.py         # Behavior FSM
    ├── health_monitor.py             # Legacy standalone HP monitor
    ├── save_map.py                   # map_saver_cli wrapper
    ├── fake_micro_status.py          # Test fake for /micro_status
    └── fake_cv_publisher.py          # Test fake for /cv/target
```

---

## Appendix: Native install (no Docker)

If you want ROS 2 and Nav2 installed directly on the Jetson (no container),
use this instead of Section 1.

```bash
# Step 1 — locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Step 2 — ROS 2 apt source
sudo apt install -y software-properties-common curl gnupg lsb-release
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Step 3 — install ROS 2 Humble + Nav2 + SLAM + tools
sudo apt update
sudo apt install -y \
    ros-humble-desktop \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-simple-commander \
    ros-humble-slam-toolbox \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-tf-transformations \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-rqt-tf-tree \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    libpcl-dev libeigen3-dev

pip3 install pyyaml transforms3d

# Step 4 — rosdep
sudo rosdep init || true
rosdep update

# Step 5 — shell integration
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc      # same as your CV container
source ~/.bashrc

# Step 6 — create the workspace and add sources
mkdir -p ~/roboto/nav2_ws/src
cd ~/roboto/nav2_ws/src
# Unzip nav2_new.zip here, plus clone Livox-SDK2, livox_ros_driver2,
# FAST_LIO_ROS2, and copy livox_converter from your old repo.

# Step 7 — Livox SDK2 (C++ library)
cd ~/roboto/nav2_ws/src/Livox-SDK2
mkdir -p build && cd build
cmake .. && make -j$(nproc) && sudo make install

# Step 8 — Build the workspace
cd ~/roboto/nav2_ws
colcon build --symlink-install \
    --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble
echo "source ~/roboto/nav2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Step 9 — configure the Livox ethernet port (on the Jetson host)
sudo ~/roboto/nav2_ws/src/nav2_new/scripts/setup_livox_net.sh

# Step 10 — smoke test
ros2 launch nav2_new sensors.launch.py
# You should see /scan, /odom, /livox/lidar publishing.
```

The rest of the workflow (SLAM, waypoint capture, navigation) is identical
whether you run in a container or natively — the launch files are the same.

Cross-container communication works automatically: native ROS 2 sees the
CV container's topics as long as the CV container runs with `--net=host`
and `ROS_DOMAIN_ID=0`.
