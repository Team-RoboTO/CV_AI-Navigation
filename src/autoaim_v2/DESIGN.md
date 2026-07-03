# autoaim_v2 — 1v1 standard auto-aim, design notes

Purpose-built for the ARC 1v1 Infantry Match: one enemy, close–mid range
(0.5–6 m), both robots moving fast, walls/occlusion, spin (up to ~400 RPM),
burst fire that is expected to land nearly every projectile.

This package replaces the Python detector + `autoaim` node + `serial_bridge`
three-process pipeline with **one C++ process** whose hot path never touches
DDS. The old pipeline in `src/autoaim` is kept untouched as a fallback and for
A/B tests (`input_mode: ros_topics` lets the new tracker run behind the old
Python detector).

## Why (measured/derived numbers)

All numbers below were verified numerically (see `test/` and
`tools/math_verify.py`); they drive every design decision.

### 1. Latency is the #1 miss source
Plate tangential displacement per unmodeled millisecond, r = 0.20 m:

| spin    | 1 ms | 5 ms | 10 ms | 20 ms |
|---------|------|------|-------|-------|
| 100 RPM | 2 mm | 10 mm| 21 mm (31 % of half-plate) | 42 mm (62 %) |
| 200 RPM | 4 mm | 21 mm| 42 mm (62 %)               | 84 mm (miss) |
| 300 RPM | 6 mm | 31 mm| 63 mm (93 %)               | 126 mm (miss)|

Consequences:
- detector rewritten in C++/TensorRT with ZED GPU-zero-copy (was: Python,
  CPU retrieve + PyCUDA + numpy decode → ~15–25 ms, jittery);
- serial TX is **event-driven** (command sent the instant it is computed,
  ~0.2 ms) instead of a 100 Hz timer (median +5 ms, worst +10 ms);
- gimbal angles consumed at the **native 1 kHz** rate of the micro status
  stream (was: 100 Hz timer drain → up to ±10 ms of transform staleness;
  at 3 rad/s gimbal slew that alone was a 9 cm error at 3 m);
- everything predicted with per-frame measured latency (kept from old code).

### 2. Ballistics with drag
17 mm / 3.2 g pellet, Cd 0.47: k/m = 0.0196 1/m. Versus the drag-free model:

| range | extra drop | extra flight time |
|-------|-----------|-------------------|
| 3 m   |  3 mm     | +4 ms  |
| 4 m   |  7 mm     | +7 ms  |
| 5 m   | 14 mm     | +10 ms |
| 6 m   | 24 mm     | +15 ms |

Plate half-height ≈ 60 mm ⇒ at 5–6 m the drag term alone eats 20–40 % of the
vertical margin, and the flight-time error rotates a 200 RPM spinner by up to
0.3 rad. `ballistics.hpp` integrates quadratic drag (RK2, 1 ms step) inside a
Newton solve on pitch; residual < 0.01 mm, cost < 10 µs.

### 3. Spherical EKF observations
Keypoint noise σ ≈ 0.7 px, fx ≈ 730 px @ 960×600:

| range | bearing σ | PnP depth σ | ratio |
|-------|-----------|-------------|-------|
| 1.5 m | 1.4 mm    |  23 mm      | 16×   |
| 3.0 m | 2.9 mm    |  90 mm      | 31×   |
| 4.5 m | 4.3 mm    | 203 mm      | 47×   |
| 6.0 m | 5.8 mm    | 362 mm      | 63×   |

The old tracker observed Cartesian (x, y, z) with near-isotropic R, i.e. it
polluted the millimetre-grade bearing with the decimetre-grade depth. The new
tracker observes **(yaw, pitch, distance, plate-angle)** with per-axis noise —
the same structure Tongji sp_vision uses — so what decides hits (angles) is
trusted at its true accuracy.

### 4. Plate-orientation from reprojection search
PnP orientation of a small near-planar quad is noisy (±10–20° at range,
bimodal near frontal). Instead: keep PnP **position**, then grid-search the
plate yaw (fixed 15° physical back-tilt) minimizing corner reprojection error
(±80° around the line of sight, 2° step + parabolic refine). Synthetic tests
recover yaw to ≈1–2°. This feeds ω estimation → spin phase prediction.

### 5. Fire policy (the "dump heat, hit everything" part)
Center-hold spray against a spinner covers a plate ~43 % of the time
(4·plate_w/(2πr), independent of RPM). Phase-timed shots, with the measured
arrival-time budget (serial ≈1.5 ms, feeder σ ≈6 ms, bullet-speed σ 1.5–3 %,
phase σ ≈3 ms):

| spin    | timed p(hit) | spray p(hit) |
|---------|--------------|--------------|
| ≤100 RPM| ~100 %       | 43 %         |
| 200 RPM | 94–98 %      | 43 %         |
| 300 RPM | 79–88 %      | 43 %         |
| 400 RPM | 66–75 %      | 43 %         |

Fire controller therefore has three regimes, chosen by |ω|:
- **TRACK** (|ω| < ω_track): aim the visible plate, hold `shoot=1` while the
  predicted hit probability ≥ `p_spray_min` (full-auto dump);
- **SWEEP** (ω_track ≤ |ω| < ω_timed): aim the *incoming* plate at impact time
  (sp_vision coming/leaving logic), full-auto while locked;
- **TIMED** (|ω| ≥ ω_timed): hold the gimbal on the spin center (no gimbal
  motion ⇒ maximal mechanical repeatability) and pulse `shoot` so each round
  *arrives* exactly when a plate crosses the bore line:
  `t_cmd = t_crossing − t_flight − feeder_delay`.

Heat is guarded by the STM32 (it already stops at `heat_limit −
heat_per_projectile`), so the Jetson never throttles below the micro's budget;
it just avoids wasting heat on < p_min shots.

### 6. ω convergence
EKF with 2° plate-angle noise at 120 Hz converges to |ω_err| < 0.3 rad/s in
100–200 ms ⇒ ≤ 8 mm of phase error over a 150 ms horizon. Two plates are
simultaneously visible ~44 % of spin phase; both are used as measurements in
the same update (better center/radius conditioning than single-plate).

## Architecture

```
             ┌────────────────────────── aim_node (single process) ─────────────────────────┐
 ZED X Mini  │  capture/infer/aim thread (hot path, no DDS, no locks on the fast path)      │
 GMSL2 ──────┼─ zed grab → GPU BGRA → CUDA letterbox → TensorRT pose → decode               │
             │   → PnP position + yaw reprojection search (solver)                          │
             │   → world transform @ capture time (gimbal_buffer, 1 kHz samples)            │
             │   → whole-car EKF, 11 states (tracker)                                       │
             │   → predict → plate select → drag ballistics → fire policy (aimer)           │
             │   → 28-byte command written to serial immediately                            │
 STM32H723 ──┼─ serial RX thread: 1 kHz status → gimbal_buffer / game state                 │
 USB CDC     │  TX heartbeat thread: 100 Hz keepalive + stale-command shoot kill            │
             │  debug thread (optional): rclcpp publishers ≤ 30 Hz, param loading           │
             └───────────────────────────────────────────────────────────────────────────────┘
```

- Serial protocol is byte-identical to the old bridge (7×f32 up / 10×f32 down,
  no framing) — no firmware change required to deploy.
- `flag_rot` (TX[6]) and the two nav floats (TX[4], TX[5]) are kept; the nav
  floats optionally carry yaw/pitch-rate feedforward when
  `send_gimbal_feedforward: true` (requires the documented 5-line micro patch;
  default false ⇒ zeros, exactly the legacy behaviour).
- State estimate → command chain runs at camera rate (120 fps).

## State model (tracker)

x = [xc ẋc yc ẏc zc żc θ ω r l h]ᵀ — car center, plate-0 angle θ, spin ω,
plate-0 radius r, radius difference l (pair 1 = r+l), height difference h.
Observation per matched plate i∈{0..3}:
z = [yaw, pitch, dist, θᵢ] of the plate center, with
plateᵢ = center − rᵢ·[cos φᵢ, sin φᵢ, 0] + [0,0,hᵢ], φᵢ = θ + iπ/2.
R diag: bearing (4 mrad)², pitch (4 mrad)², distance (0.03+0.05·d)²·obliquity
inflation, plate-angle (σ from search, inflated with obliquity).
Q: white-noise acceleration (v1 on translation, v2 on spin), tiny on r/l/h,
clamped to physical ranges (r∈[0.10,0.35], |l|≤0.15, |h|≤0.12,
|ω| ≤ 52 rad/s ≈ 500 RPM).

1v1 simplification: exactly one enemy. All enemy-color detections in a frame
are associated to the single car by predicted-plate angular distance; unmatched
detections re-seed the car only when LOST (or teleport-level divergence —
covers wall re-appearance on the far side).

## Files

| file | role |
|------|------|
| `types.hpp` | shared PODs (detections, gimbal samples, command, params) |
| `gimbal_buffer.hpp` | lock-guarded 1 kHz ring; angle interpolation at capture time |
| `protocol.hpp` | byte-exact legacy 28 B / 40 B packets |
| `serial_port.*` | termios CDC-ACM port, low-latency |
| `ballistics.*` | quadratic-drag integrator + Newton pitch solve |
| `solver.*` | PnP (IPPE small/large) + plate-yaw reprojection search |
| `tracker.*` | 11-state spherical EKF whole-car tracker |
| `aimer.*` | prediction, plate selection, fire policy, hit probability |
| `detector_trt.*` | TensorRT pose engine (raw + post-NMS layouts) |
| `zed_camera.*` | ZED SDK wrapper (GPU retrieve, µs exposure) |
| `preprocess.cu` | BGRA→letterboxed NCHW CUDA kernel |
| `aim_node.cpp` | process wiring, threads, ROS debug layer |
| `test/*` | gtest: ballistics, solver, tracker, aimer, protocol, buffer |

Build flavors: on the Jetson (ZED SDK + TensorRT found) everything builds; on
a dev machine without them, the core library + tests + `ros_topics` input mode
still build and run (CMake auto-detect).
