# autoaim — Field Manual

This document covers everything you need to operate, tune, and debug the auto-aim
package on your own. It is organized in four parts:

  PART 1 — Build, launch, and verify the package is alive.
  PART 2 — Architecture: how the pieces fit, what each parameter controls.
  PART 3 — Tuning playbook: symptom → cause → which parameter to change.
  PART 4 — Full change history from the original package.

The last part is mainly archival; if you're handing this off, start at Part 1.

---

# PART 1 — Build and launch

## Build (always use --symlink-install)

```bash
cd ~/ros2_ws   # or wherever your workspace is
colcon build --packages-select autoaim --symlink-install
source install/setup.bash
```

`--symlink-install` is mandatory. Without it, every change to the launch file or any
Python script (viewer, serial bridge) requires a full rebuild before taking effect.
With it, edits to those files are live as soon as you re-launch.

C++ files (`tracker.cpp`, `autoaim_node.cpp`, `pnp_solver.cpp`) still require
a rebuild after each change.

## Launch

```bash
ros2 launch autoaim standard.launch.py
ros2 launch autoaim hero.launch.py
ros2 launch autoaim sentry.launch.py
```

Each launch starts four nodes:

- `zed_detector` — ZED X Mini capture, TensorRT inference, keypoints,
  camera info, and IMU.
- `micro_communications_node` — serial bridge to the micro (publishes `/micro_status`,
  subscribes `/cmd_vel_AI`). Auto-reconnects on cable unplug or micro reflash.
- `autoaim` — main C++ node (subscribes to detector, publishes `/cmd_vel_AI`).
- `autoaim_viewer` — debug overlay (publishes `/tracker/debug_image`).

## First-launch sanity check

After launching, in another terminal run each of these and verify what you see:

```bash
# Detector is publishing keypoints?
ros2 topic hz /detector/armors_keypoints
# Should show ~70 Hz (or whatever your detector runs at).

# Serial bridge is alive?
ros2 topic echo /micro_status --once
# Should print data: [yaw, pitch, vx, vy, ...]. If empty array,
# the micro isn't sending or the port is wrong.

# Auto-aim node is publishing commands?
ros2 topic echo /cmd_vel_AI --once
# Should print a Twist message. linear.x = distance to target,
# angular.x = fire flag (0 or 1), angular.y = absolute pitch [rad],
# angular.z = absolute yaw [rad].

# RViz debug image alive?
ros2 topic hz /tracker/debug_image
# Open RViz, add Image display on /tracker/debug_image. You should see
# detection bboxes, keypoint quads, and an aim crosshair.
```

If any of these fails, jump to Part 3 ("Symptom: no commands published" or
"Symptom: serial bridge can't connect").

---

# PART 2 — Architecture and parameter reference

## Data flow

```
   Camera frames
        │
        ▼
   YOLO detector
        │
        ├──► /detector/armors           (bboxes only — legacy)
        └──► /detector/armors_keypoints (4-point armor corners — preferred)
                                  │
                                  ▼
                       autoaim node
                  ┌───────────────────────┐
                  │  1. PnP from kpts     │ → 3D armor pose in camera frame
                  │  2. TF camera → odom  │ → 3D armor in world (odom) frame
                  │  3. EKF tracker       │ → predict target center, vyaw, radius
                  │  4. computeAim        │ → barrel angles + fire decision
                  │  5. Smooth + rate lim │ → safety/feasibility
                  └───────────────────────┘
                                  │
                                  ▼
                       /cmd_vel_AI (Twist)
                                  │
                                  ▼
                       serial bridge node
                                  │
                                  ▼
                       Micro (gimbal + shooter)
                                  │
                                  ▼
                       /micro_status (yaw, pitch, vx, vy, ...)
                       (feeds back into autoaim)
```

## Topic conventions

`/cmd_vel_AI` (Twist):
- `linear.x` = distance to target [m] (informational; some firmware uses it)
- `angular.x` = fire flag (0.0 or 1.0)
- `angular.y` = absolute pitch target [rad]
- `angular.z` = absolute yaw target [rad]

`/micro_status` (Float32MultiArray) — layout the C++ node expects:
- `data[0]` = current micro yaw [rad]
- `data[1]` = current micro pitch [rad]
- `data[2]` = chassis vx [m/s] (when ego-motion enabled)
- `data[3]` = chassis vy [m/s] (when ego-motion enabled)
- `data[N]` = chassis yaw [rad] (optional; set `chassis_heading_index` to N to use it)
- The serial bridge appends a TX echo after these: ai_yaw, ai_pitch, shoot, ...

## Coordinate frames

- **camera frame**: X right, Y down, Z forward (OpenCV convention).
- **odom (world) frame**: X forward of robot at startup, Y left, Z up (REP-103).
- **gimbal body frame**: barrel forward (X), left (Y), up (Z).
- The TF chain converts armor pose from camera → odom using the gimbal's current
  yaw/pitch (from `/micro_status[0..1]`). When ego-motion is enabled, the chain also
  adds the integrated robot position `robot_x_/robot_y_` so `imu_translation_` is the
  current camera location in world coordinates.

## The EKF state vector (9D)

`x = [px, vx, py, vy, pz, vz, yaw, vyaw, r]`

| Index | Symbol | Meaning |
|---|---|---|
| 0 | `px` | target center x in odom [m] |
| 1 | `vx` | target center x-velocity [m/s] |
| 2 | `py` | target center y in odom [m] |
| 3 | `vy` | target center y-velocity [m/s] |
| 4 | `pz` | target center z (altitude) [m] |
| 5 | `vz` | target center z-velocity [m/s] |
| 6 | `yaw` | spin angle of the chassis [rad] — face 0 is at this yaw |
| 7 | `vyaw` | spin rate [rad/s] (positive = CCW from above) |
| 8 | `r` | chassis radius (center-to-armor) [m] |

`armorFromState(x) = (px - r·cos(yaw), py - r·sin(yaw), pz)` gives the
position of the *currently-visible* face in odom.

## The state machine

```
   LOST ── new detection ──► DETECTING ── confirm_frames matches ──► TRACKING
    ▲                            │                                      │
    │                            └── all misses ──► LOST                │
    │                                                                   │
    │  lost_timeout no match                                            │
    └─────── coast expired ─────── TEMP_LOST ◄─── miss while TRACKING ──┘
```

- **LOST**: pick the closest detection, init EKF, → DETECTING.
- **DETECTING**: must match `confirm_frames` consecutive frames → TRACKING.
- **TRACKING**: full normal operation, fire enabled.
- **TEMP_LOST**: a few frames of coasting on the EKF prediction.

## Parameter index

Every tunable parameter is listed below, grouped by what it controls. Where it
lives in the launch file is shown so you can find it fast.

### Detector input

| Param | Default | What it does |
|---|---|---|
| `target_classes` | `['0']` | Which class IDs the auto-aim attacks. Current YOLO26 labels: `0`=blue, `1`=grey, `2`=red. Grey is ignored by the tracker. Set to YOUR ENEMIES' color. |
| `use_keypoints` | `true` | If true, subscribe to `/detector/armors_keypoints`. If false, fall back to bbox-only PnP. |
| `keypoint_topic` | `/detector/armors_keypoints` | Where to read keypoint detections. |
| `min_keypoint_score` | `0.0` | Reject keypoints with score below this. Raise to 0.3–0.5 if walls/floor cause false dets. |
| `max_reproj_error` | `25.0` | PnP solutions with average reprojection error above this many pixels are rejected. |
| `light_ratio` | `0.85` | Shrink factor for the bbox PnP fallback. |
| `max_armor_distance` | `6.0` | Armors farther than this [m] from the camera are dropped. |
| `max_armor_z` | `4.0` | Armors above this height [m] are dropped (ceiling reflections, etc.). |

### Tracker state machine

| Param | Default | What it does |
|---|---|---|
| `confirm_frames` | `3` | Consecutive detections needed to confirm DETECTING → TRACKING. |
| `lost_timeout` | `0.30` | Seconds to coast in TEMP_LOST before giving up → LOST. |

### EKF process noise (how fast can the target change?)

| Param | Default | What it does |
|---|---|---|
| `q_pos` | `5.0` | XY position process noise. Higher = target can accelerate harder. |
| `q_yaw` | `10.0` | Spin rate process noise. Higher = target can change RPM more quickly. |
| `q_r` | `1e-6` | Radius process noise. Should stay near zero (radius is constant per robot). |

### EKF measurement noise (how much do we trust each detection?)

| Param | Default | What it does |
|---|---|---|
| `r_pos_base` | `0.05` | XY measurement noise at zero range [m]. |
| `r_pos_slope` | `0.04` | Extra noise per meter of range [m/m]. |
| `r_yaw_base` | `0.05` | Armor yaw measurement noise at zero range [rad]. |
| `r_yaw_slope` | `0.005` | Extra yaw noise per meter [rad/m]. |
| `max_oblique_deg` | `65.0` | Beyond this oblique angle the armor yaw measurement is ignored. |

### Velocity damping

| Param | Default | What it does |
|---|---|---|
| `alpha_pos` | `0.99` | Damping on XY velocity per frame at `ref_freq`. Closer to 1.0 = velocity persists longer. |
| `alpha_yaw` | `1.00` | Damping on spin rate. Keep at 1.0 for spinning enemies. |
| `alpha_coast` | `0.95` | Damping while coasting in TEMP_LOST. |
| `ref_freq` | `70.0` | Damping reference frequency. MUST match your detector rate. |

### Armor geometry

| Param | Default | What it does |
|---|---|---|
| `initial_radius` | `0.24` | First guess at chassis radius [m]. Small bot 0.24, hero 0.28. |
| `radius_ema_alpha` | `0.05` | How fast the radius adapts to measurements. |
| `initial_dz` | `0.05` | Known height step [m] between armor pairs (faces 1,3 are this much higher than faces 0,2). Set to 0.0 if all four plates are at the same height. |

### Ballistics

| Param | Default | What it does |
|---|---|---|
| `bullet_speed` | `25.0` | Bullet muzzle velocity [m/s]. MEASURE THIS — wrong value = vertical miss. |
| `gravity` | `9.8` | Gravitational acceleration [m/s²]. |
| `gimbal_height` | `0.325` | Height of camera/gimbal pivot above ground [m]. |
| `barrel_offset_x` | `0.0` | Offset from camera to barrel exit (forward) [m]. |
| `barrel_offset_y` | `0.0` | Offset from camera to barrel exit (left) [m]. |
| `barrel_offset_z` | `-0.05` | Offset from camera to barrel exit (up). NEGATIVE = barrel below camera. |

### Fire gate

| Param | Default | What it does |
|---|---|---|
| `angular_window` | `0.13` | Max angle [rad] between face normal and barrel-target line for fire. ~7.5°. |
| `window_ref_dist` | `3.0` | Reference distance for the angular window. |
| `min_fire_dist` | `0.5` | Don't fire closer than this [m]. |
| `max_fire_dist` | `6.0` | Don't fire farther than this [m]. |

### Timing and prediction

| Param | Default | What it does |
|---|---|---|
| `time_bias` | `0.025` | Total system latency [s] added to bullet flight time prediction. |

### EKF data association

| Param | Default | What it does |
|---|---|---|
| `max_match_dist` | `0.5` | Max position gap [m] between prediction and detection for a match. |
| `maha_threshold` | `13.3` | Mahalanobis gate. 9.49=strict (95%), 13.3=loose (99%), 16.3=very loose. |

### Target switching

| Param | Default | What it does |
|---|---|---|
| `switch_range_ratio` | `0.85` | New target must be < this × current range to switch. |
| `switch_cooldown` | `10` | Frames before allowing another switch. |
| `same_target_identity_dist` | `1.0` | New detection within this [m] of current target's predicted face = same robot (no switch). |

### Command smoothing and safety

| Param | Default | What it does |
|---|---|---|
| `cmd_smooth_alpha` | `0.85` | EMA on the gimbal target. 1.0 = no smoothing. |
| `cmd_deadband_yaw` | `0.005` | Below this yaw error [rad], don't update yaw command. |
| `cmd_deadband_pitch` | `0.005` | Same for pitch. |
| `cmd_rate_limit_yaw` | `0.0` | Max yaw command change per second [rad/s]. 0 = disabled. |
| `cmd_rate_limit_pitch` | `0.0` | Same for pitch. |
| `fire_lock_yaw` | `0.05` | Gimbal must be within this yaw error of command before firing [rad]. |
| `fire_lock_pitch` | `0.04` | Same for pitch. |
| `cmd_hold_time` | `0.25` | After target lost, hold last command for this long [s] before relaxing. |
| `cmd_max_delta_yaw` | `0.80` | Max one-frame yaw command jump [rad]. |
| `cmd_max_delta_pitch` | `0.35` | Same for pitch. |
| `require_aim_inside_frame` | `false` | If true, never fire when predicted aim is outside the camera frame. |
| `micro_pitch_feedback_opposite_sign` | `true` | If micro reports pitch with opposite sign from command. **PHYSICAL CHECK REQUIRED.** |

### Ego-motion compensation

| Param | Default | What it does |
|---|---|---|
| `use_ego_motion_compensation` | `true` | Master switch for ego-motion code paths. |
| `ego_velocity_available` | `false` | **SAFETY**: is the micro actually sending good vx/vy? Until True, robot_x_/y_ stay zero. |
| `ego_velocity_body_frame` | `true` | True if micro sends vx/vy in chassis body frame (needs chassis heading). |
| `ego_velocity_scale_x` | `1.0` | Multiply incoming vx by this. |
| `ego_velocity_scale_y` | `1.0` | Multiply incoming vy by this. |
| `ego_velocity_max` | `3.0` | Clip ego velocity magnitude [m/s]. Above this = assumed glitch. |
| `ego_position_max_drift` | `0.0` | Reset threshold for dead-reckoning drift [m]. 0.0 = disabled. |
| `chassis_heading_index` | `-1` | Index in `/micro_status` where chassis world-frame yaw is stored. -1 = use gimbal yaw (wrong during chassis spin). |

### Gimbal sign

| Param | Default | What it does |
|---|---|---|
| `gimbal.yaw_sign` | `1.0` | Flip to -1.0 if gimbal moves the wrong way in yaw. |
| `gimbal.pitch_sign` | `1.0` | Flip to -1.0 if gimbal moves the wrong way in pitch. |

### Serial bridge

| Param | Default | What it does |
|---|---|---|
| `serial_port` | `/dev/ttyACM0` | Serial device path. Use a udev rule for stable naming. |
| `serial_baudrate` | `500000` | Must match micro firmware. |
| `serial_tx_hz` | `100.0` | TX rate. Lower if the micro can't keep up. |
| `serial_reconnect_interval` | `2.0` | Retry period when port is closed [s]. |
| `serial_rx_timeout` | `3.0` | No-RX timeout before forcing reconnect [s]. |

---

# PART 3 — Tuning playbook

## How to use this section

Each entry is `Symptom → Causes (in order of probability) → Fix`. Work top-down
within each entry; don't jump around or you'll have trouble isolating the cause.

The single most important habit while tuning: **change ONE parameter at a time,
then test**. If you change three things and it gets better, you don't know which
of the three actually helped. The launch file uses `--symlink-install` so each
change just needs a re-launch, no rebuild.

## Diagnosing by topic

When something feels wrong, the fastest way to localize the problem is to look
at the topics:

```bash
# Is the detector running?            ros2 topic hz /detector/armors_keypoints
# Is the auto-aim node alive?         ros2 topic hz /cmd_vel_AI
# Is the micro feeding back?          ros2 topic hz /micro_status
# Is the tracker in a good state?     ros2 topic echo /cmd_vel_AI
#   ↳ if linear.x is ~0, no target.
#   ↳ if angular.x is always 0, never firing.
#   ↳ if angular.y/z change wildly, the tracker is unstable.
```

## SYMPTOM: nothing happens, no commands published

Possible causes, in order:

1. **Detector isn't publishing.** Check `ros2 topic hz /detector/armors_keypoints`.
   If 0 Hz: the YOLO detector node isn't running. Check its launch.

2. **target_classes is wrong.** If the enemies are red (class `2`) but
   `target_classes: ['0']`, every detection is filtered out. Check enemy color
   and update. In the launch file:
   ```python
   {'target_classes': ['2']},   # red enemies
   {'target_classes': ['0']},   # blue enemies
   {'target_classes': ['0','2']},  # any color (training/mixed match)
   ```

3. **Camera info missing.** PnP needs the camera intrinsics. Check
   `ros2 topic echo /camera_info --once`. If nothing prints, the camera driver
   isn't publishing intrinsics.

4. **All detections rejected by max_reproj_error.** Look in the auto-aim node
   logs for `REJECTED: rel_range=...` or PnP failure messages. If many rejections,
   raise `max_reproj_error` to 40 in the launch file.

## SYMPTOM: tracker detects but never fires

1. **`fire_lock_yaw` / `fire_lock_pitch` too tight.** The gimbal can never get
   close enough to satisfy the lock. Raise to 0.07 / 0.06 and see if firing
   resumes. Tighten once you have hits.

2. **`angular_window` too narrow.** The face is never aligned enough to be
   "hittable" by the geometric gate. Raise to 0.16 (≈9°). Tighten back down
   once everything else works.

3. **`min_fire_dist` too large.** If you tested at 0.4m but `min_fire_dist=0.5`,
   no fire. Lower to 0.3.

4. **Pitch sign wrong → permanent pitch error.** If `micro_pitch_feedback_opposite_sign`
   is wrong, the geometry believes the gimbal is pointing in the wrong direction.
   The pitch fire-lock error will be large and constant. Test: tilt the gimbal
   up manually and watch RViz — the detected armor should appear higher in odom.
   If it appears lower, flip `micro_pitch_feedback_opposite_sign` in the launch file.

5. **`cmd_deadband_yaw` >= `fire_lock_yaw`.** The deadband freezes correction
   before the lock can be satisfied. Always keep deadband < lock threshold.

6. **Holding stale command.** Watch the node logs for "Holding last cmd". If
   that prints constantly, `command_valid` is failing. Usually `aim.target_valid`
   is false — check that the EKF reached TRACKING with `ros2 topic echo /cmd_vel_AI`.

## SYMPTOM: shots land behind the plate (over-prediction or too-slow EKF)

The aim is correct on a static target but lags behind a moving one.

1. **`time_bias` too small.** This is the most common cause. Increase by 0.010 s
   per test until the aim leads the target correctly. Typical good value:
   0.030–0.045 s.

2. **`ref_freq` wrong.** Run `ros2 topic hz /detector/armors_keypoints`. If it
   reports 65 Hz but `ref_freq: 100`, the damping formula multiplies dt × 100
   = 1.54 frames-equivalent per actual frame, so damping is too strong, and the
   tracker's velocity decays before the bullet arrives. Set `ref_freq` to the
   measured rate.

3. **`alpha_pos` too low.** At 0.99 the per-frame velocity damping is mild; at
   0.95 it's quite aggressive. Try `alpha_pos: 0.995` or even `1.0` (no damping).

4. **`q_pos` too low.** If the EKF process noise is too small, it doesn't react
   to acceleration. Raise to 8 or 10. Watch for noisier aim on static targets.

5. **`cmd_smooth_alpha` too low.** The EMA on the command smooths out fast
   corrections. Raise to 0.95 or 1.0 (no smoothing).

## SYMPTOM: shots land in front of the plate (over-prediction)

Aim leads too much. The bullet arrives where the target was going to be but isn't.

1. **`time_bias` too large.** Reduce by 0.005 s steps.

2. **`alpha_pos` at 1.0 with a decelerating target.** No damping means a target
   that just stopped is still predicted as moving. Try 0.98.

3. **`q_pos` too high.** Process noise inflates velocity estimates from each
   measurement. Reduce to 3 or 4.

## SYMPTOM: shots land left or right of center

1. **`barrel_offset_y` wrong.** Measure with a ruler from camera centre to
   barrel centre. Positive = barrel left of camera. Typical RoboMaster: a few cm.

2. **`yaw_offset_deg`** (if you have it in the launch file as a static yaw bias).
   Adjust in 0.3° steps if there is a residual lateral bias after other tuning.

3. **`gimbal.yaw_sign` wrong.** If the gimbal moves the wrong way in yaw, every
   shot lands on the opposite side. Flip to `-1.0`.

## SYMPTOM: shots land high or low

1. **`bullet_speed` wrong.** This is the most common cause for vertical miss.
   MEASURE with a chronograph. A 10% error in bullet speed causes ~2 cm vertical
   miss at 3 m.

2. **`barrel_offset_z` wrong.** Measure camera-to-barrel offset with a ruler.
   Standard RoboMaster: -0.05 to -0.15 (barrel below camera). If shots land
   below aim, make this less negative.

3. **`gimbal_height` wrong.** Measure from ground to camera center. Affects
   gravity drop compensation.

4. **`micro_pitch_feedback_opposite_sign` wrong.** See the section on "never
   fires" above.

## SYMPTOM: gimbal slow / sluggish, never catches a fast target

1. **`cmd_smooth_alpha` too low.** Raise toward 1.0.

2. **`cmd_rate_limit_yaw` / `cmd_rate_limit_pitch` limiting too much.** If
   non-zero, this caps slew rate. Set to 0.0 for no limit.

3. **Gimbal motor PID is the bottleneck.** Not a software issue — increase
   motor current / PID gains on the micro.

## SYMPTOM: tracker flickers (TRACKING ↔ LOST repeatedly)

1. **`lost_timeout` too short.** Raise to 0.5 s. At 300 RPM all 4 armor faces
   cycle in 200 ms; if the detector misses 2-3 in a row that's 100 ms of no
   measurement, and a 0.3 s timeout barely covers it.

2. **`maha_threshold` too strict.** Raise to 13.3 or 16.3. With keypoint PnP
   noise, the strict 9.49 rejects valid associations.

3. **`max_match_dist` too small.** Raise to 0.7 or 0.8.

4. **`q_yaw` too low for changing spin rate.** If the enemy spins up/down
   mid-match, the EKF's vyaw estimate gets stale and face jumps fail to match.
   Raise `q_yaw` to 15 or 20.

## SYMPTOM: tracker locked onto a far enemy while closer ones are visible

This is target-switching behaviour. The fix depends on what you want.

1. **`switch_range_ratio` too low (too sticky).** Default 0.85 means the new
   target must be < 85% of current range. Raise toward 1.0 (aggressive switching)
   to almost always go for the closest.

2. **`same_target_identity_dist` too large.** If two close-together enemies
   are being treated as the same robot, lower this — try 0.7 m.

3. **`switch_cooldown` too long.** Reduce frames if you want faster switching.

## SYMPTOM: target jumps between two close-together enemies

Opposite of the above.

1. **`same_target_identity_dist` too small.** Two robots passing close get
   identified as different targets every frame. Raise to 1.5 m.

2. **`switch_cooldown` too short.** Raise to 20 frames.

3. **`switch_range_ratio` too high.** Lower to 0.7 (much closer must be the new
   target for a switch).

## SYMPTOM: serial bridge can't connect

1. **Wrong port.** Check with `ls /dev/tty{ACM,USB}*`. Update `serial_port`
   in the launch file.

2. **Permissions.** `sudo chmod 666 /dev/ttyACM0`. For a permanent fix add a
   udev rule.

3. **Baudrate mismatch.** Verify the micro firmware uses the same baud
   (default 500000).

4. **Parity mismatch.** The Python bridge uses `PARITY_EVEN`. If your micro
   uses none, edit the serial bridge file (line near `parity=serial.PARITY_EVEN`).

## SYMPTOM: ego-motion makes everything worse

Symptoms: shots stop landing once you start driving, aim wanders, or the
tracker loses targets while moving.

1. **`ego_velocity_available` should be False.** If your micro isn't sending
   validated vx/vy, this MUST be False. Default is False — only flip to True
   after verifying micro data with `ros2 topic echo /micro_status`.

2. **`chassis_heading_index` wrong / -1 while chassis spins.** Without chassis
   heading, the body-frame velocity is rotated by the gimbal yaw, which is
   wrong during 300 RPM spin. Either fix the micro firmware to send chassis
   heading, or set `ego_velocity_body_frame: false` (micro sends world-frame
   velocity directly).

3. **`ego_position_max_drift` reset misbehaving.** Keep at 0.0 (disabled) until
   you've fully validated ego-motion. The reset now translates the EKF state,
   but it's still better to never reset than to reset wrongly.

## SYMPTOM: viewer shows no detections

1. **Wrong topic in detector launch.** The C++ node and viewer expect
   `/detector/armors_keypoints`. Check the detector node's topic name.

2. **Camera image stream missing.** The viewer uses `/yolo/debug_image`. If
   that's a different name, edit `viewer_node.py` line 89.

3. **Message type not built.** If the viewer prints "ArmorKeypointArray not
   available", rebuild the package: `colcon build --packages-select autoaim`.

---

# PART 4 — Full change history

This is what changed from the original package snapshot:
with, in the order the fixes were made.

## Round 1 — bugs from the very first review

| # | What changed | Why |
|---|---|---|
| 1 | `cmd_deadband_yaw/pitch` 0.100/0.015 → 0.005/0.005 rad | The deadband was wider than the fire lock — system was stuck in a band where correction was frozen but lock wasn't satisfied. Fire was impossible. |
| 2 | `cmd_smooth_alpha` 0.30 → 0.85 | At 0.30 the gimbal lagged 2-4 frames behind a moving target. |
| 3 | Removed duplicate fire gate in node | The node was applying a stricter camera-space gate on top of the tracker's gate, blocking fire whenever the barrel offset put the ballistic aim point away from the optical center. |
| 4 | `angular_window` 0.30 → 0.13 rad | 0.30 (~17°) allowed firing at plates so oblique the ball would skim the edge. |
| 5 | Dynamic vyaw cap with covariance inflation, replacing hard clamp at 25 rad/s with full P reset | 300 RPM = 31.4 rad/s exceeded the clamp every frame. P_ was reset constantly, the EKF could never converge on a fast spinner. New cap is 52.4 rad/s (mechanical limit) and only inflates the vyaw row of P. |
| 6 | Ballistic seed distance is now the actual target distance | Was hardcoded 1.5/bullet_speed. Wrong for any range except 1.5 m. |
| 7 | `ref_freq` 100 → 70 | Mismatch between detector rate (70 Hz) and damping reference (100) over-damped position/yaw by 43%. |
| 8 | `time_bias` 0.10 → 0.025 s | At 300 RPM, 0.10 s over-predicted 180° of rotation — aim landed on the opposite face. |
| 9 | Added `chassis_heading_index` parameter | Gimbal yaw is not chassis yaw — the head is rotationally decoupled. Body-frame velocity needs chassis heading to rotate into world. |
| 10 | Serial RX drain keeps only the most recent packet | Old code processed the oldest queued packet, using 20-30 ms stale IMU data. |
| 11 | `barrel_offset_z` +0.1 → -0.05 (barrel below camera) | The original sign was inverted from physical reality. |

## Round 2 — bugs from the second review

| # | What changed | Why |
|---|---|---|
| 12 | `computeAim` now takes `robot_x`, `robot_y` and uses them for the barrel position | Without ego position, the barrel was assumed to be at the world origin. Once the robot moved, the aim vector had wrong range and bearing. |
| 13 | `cmd_hold_time` actually expires | Was `if (within_hold_time || has_safe_cmd_)` — the `|| has_safe_cmd_` made the stale-command branch always fire. Gimbal stayed pointed at stale positions forever. |
| 14 | `shouldSwitch` allows switching during DETECTING (initially) | The original code blocked DETECTING switches, contradicting the documented policy. |
| 15 | Target switch resets the node's smooth/rate state | Without this, after a switch the EMA blended old + new target's angles for 2-3 frames. |
| 16 | Fire hysteresis re-checks `aim.fire` | The old hysteresis kept firing for 1-2 frames after the face had rotated past the window. |
| 17 | Added `ego_velocity_available` safety flag | Without it, garbage velocity from the micro would integrate into robot_x_/y_ and corrupt the odom frame. New flag lets you keep ego-motion code enabled but inactive until micro firmware is ready. |
| 18 | Added `ego_position_max_drift` drift cap | Pure dead-reckoning integration drifts over time. Cap resets to origin when integrated position is implausibly large. |
| 19 | Visibility check uses robot position | Tiebreaker between faces of similar margin. Minor but consistent. |
| 20 | Serial bridge auto-reconnects | Original code crashed if the micro was reflashed or the cable briefly unplugged. New code detects RX timeout / open failure and retries every `serial_reconnect_interval` seconds. |

## Round 3 — bugs from the third review

| # | What changed | Why |
|---|---|---|
| R3-1 | `shouldSwitch` during DETECTING only switches to different `class_id` (later refined further in R4-1) | The old fix from Round 2 made the tracker stuck — every frame switched to the same target, resetting `detect_count_` to 0, never reaching TRACKING. |
| R3-2 | Distance filter uses `p_cam.length()` instead of `hypot(p_odom.x, p_odom.y)` | World-origin distance was wrong once the robot moved away from origin. Close targets were rejected as "too far". |
| R3-3 | Inward-yaw bearing in node uses robot position | The "armor faces inward" correction referenced the world origin. Once the robot moved, this flipped armor yaw the wrong way. |
| R3-4 | `computeAim` takes `robot_vx`, `robot_vy` and propagates the barrel by `velocity × pred_t` | At 1 m/s with 0.1 s flight, the barrel moves 10 cm during bullet flight. Old code used barrel-now, target-future. |
| R3-5 | Added `setEgoPose` to tracker, all internal bearings use it; `ArmorDetection.rel_range` field is the camera-relative distance | Inconsistency between detection-side (correct after R3-2) and tracker-side range/bearing math. |

## Round 4 — bugs from the fourth review

| # | What changed | Why |
|---|---|---|
| R4-1 | `shouldSwitch` uses SPATIAL identity, not class_id | YOLO `class_id` is COLOR, not robot identity. With all enemies the same color, the R3 fix prevented all switching — locked onto first detection forever. New logic: same robot iff candidate is spatially close to predicted current target. |
| R4-2 | `ekfUpdate` and `ekfMahalanobis` use camera-relative range for noise scaling | World-origin distance scaled measurement noise wrong once ego-motion was active. |
| R4-3 | Pitch sign correction applied to GEOMETRY pitch too, not just lock pitch | If `micro_pitch_feedback_opposite_sign=true`, the camera→odom rotation was wrong, flipping armors vertically in odom. |
| R4-4 | Viewer subscribes to `/detector/armors_keypoints` and draws 4-point quads | Old viewer only drew bboxes from `/detector/armors`. With `use_keypoints: true` the viewer showed nothing. |
| R4-5 | RViz aim marker position starts at `robot_x_/y_` | Old marker was at world origin even after ego-motion moved the robot. RViz-only, doesn't affect shots. |
| R4-6 | Drift-cap reset also calls `Tracker::shiftWorldFrame()` | Without shifting the EKF state, the target appeared to teleport by the reset amount → Mahalanobis gate rejected everything. Default for `ego_position_max_drift` is now 0.0 (disabled) until ego-motion is fully validated. |

## Round 5 — robustness polish

| # | What changed | Why |
|---|---|---|
| R5-1 | PnP falls back to `SOLVEPNP_ITERATIVE` if IPPE fails | IPPE is great for planar targets but fails on degenerate keypoint configurations (collinear, very small area, partial occlusion). Costs ~1 ms only on failure frames. |
| R5-2 | `max_reproj_error` aligned at 25.0 in launch, node, and header | Launch was 40, node default 15, header default 10/15 — three different effective values. Aligned to 25 (mid-permissive). |
| R5-3 | `maha_threshold` raised to 13.3 in launch | 9.49 (95%) was too strict for noisy keypoint PnP — frequent re-init via the divergence path. 13.3 (99%) is more forgiving. |

---

# Quick reference — emergency tuning

If a match is starting in 5 minutes and you have to get something working:

```python
# Open the launch file:
# install/autoaim/share/autoaim/launch/sentry.launch.py
# (only works if you built with --symlink-install; otherwise edit in src/)

# DETECTOR
{'target_classes': ['0']},      # blue=0, red=2 — match enemy color
{'min_keypoint_score': 0.0},
{'max_reproj_error': 40.0},     # permissive — accept noisy keypoints

# EGO-MOTION — keep disabled for first match unless thoroughly validated
{'use_ego_motion_compensation': True},
{'ego_velocity_available': False},
{'ego_position_max_drift': 0.0},

# FIRE GATE — loose for first match, tighten after observing hits
{'angular_window': 0.16},       # 9.2° — wider window
{'fire_lock_yaw': 0.07},
{'fire_lock_pitch': 0.06},
{'cmd_deadband_yaw': 0.005},
{'cmd_deadband_pitch': 0.005},

# PREDICTION — start conservative
{'time_bias': 0.030},
{'ref_freq': 70.0},             # set to your detector rate
{'cmd_smooth_alpha': 0.90},

# BALLISTICS — measure these
{'bullet_speed': 25.0},         # ← chronograph
{'barrel_offset_z': -0.05},     # ← ruler

# TRACKER — forgiving
{'maha_threshold': 13.3},
{'lost_timeout': 0.50},
{'max_match_dist': 0.7},
{'same_target_identity_dist': 1.0},

# GIMBAL SIGNS — verify with manual gimbal command
{'gimbal.yaw_sign': 1.0},
{'gimbal.pitch_sign': 1.0},
{'micro_pitch_feedback_opposite_sign': True},   # ← physically verify
```

After the first match, tighten step by step (see Part 3) based on what you saw.
