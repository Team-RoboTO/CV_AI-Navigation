# autoaim_v2

Single-process C++ auto-aim for the **ARC 1v1 Infantry Match** — ZED X Mini on
Jetson AGX Orin, STM32H723 over USB-CDC (byte-compatible with the existing
firmware, no micro changes needed to deploy).

- **[DESIGN.md](DESIGN.md)** — architecture and the verified math behind every
  decision (latency budget, drag ballistics, spherical EKF, fire regimes).
- **[DEPLOYMENT.md](DEPLOYMENT.md)** — build on the Jetson, safety checklist,
  calibration order, systemd autostart, troubleshooting.

## What it does differently from `src/autoaim` (old pipeline)

| area | old | new |
|------|-----|-----|
| processes | 3 (Python detector, tracker node, serial bridge) + DDS hops | 1 process, hot path never touches DDS |
| detector | Python + PyCUDA, CPU image retrieve | C++ TensorRT, ZED GPU zero-copy |
| gimbal angles | 100 Hz topic, ring buffer | native 1 kHz serial, interpolated at capture time |
| serial TX | 100 Hz timer (+5 ms median) | event-driven (~0.2 ms), 100 Hz heartbeat fallback |
| EKF | 9-state, Cartesian xyz observation | 11-state whole-car, spherical (yaw/pitch/dist/θ) observation — angles trusted at mm grade, PnP depth at its real dm grade |
| plate yaw | raw PnP orientation | reprojection grid search (~1–2° vs ±10–20°) |
| ballistics | gravity only | quadratic drag (17 mm pellet), Newton solve |
| fire | angular window + hysteresis | 3 regimes by spin rate: TRACK (full-auto on p_hit), SWEEP (incoming plate), TIMED (center-hold + sub-ms scheduled shots so pellets *arrive* on plate crossings) |
| target loss | coast + timeout | ω preserved behind walls, teleport re-seed on far-side reappearance |

## Quick start

```bash
colcon build --packages-up-to autoaim_v2 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch autoaim_v2 standard_1v1.launch.py            # fire OFF by default
ros2 launch autoaim_v2 standard_1v1.launch.py shooting_active:=true debug:=true
```

Tests: `colcon test --packages-select autoaim_v2` (25 gtests) and the
no-hardware full-pipeline replay in DEPLOYMENT.md §1.

## Layout

```
include/autoaim_v2/   types, gimbal ring buffer, legacy serial protocol
src/ballistics.cpp    drag integrator + Newton pitch solve
src/solver.cpp        PnP position + plate-yaw reprojection search
src/tracker.cpp       11-state spherical EKF (translation/rotation rescue)
src/aimer.cpp         prediction, plate selection, fire regimes, p_hit
src/detector_trt.cpp  TensorRT pose engine (raw + end2end layouts)
src/zed_camera.cpp    ZED SDK wrapper (GPU retrieve, µs exposure)
src/preprocess.cu     BGRA→NCHW letterbox kernel
src/aim_node.cpp      process wiring: threads, serial, ROS params/debug
test/                 25 gtests incl. timed-fire arrival invariant
tools/                math_verify.py (design study), replay_synthetic.py (e2e)
deploy/               systemd unit + container start script
```
