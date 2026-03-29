# auto_aim_targeting

`auto_aim_targeting` is the owner-path targeting runtime for the auto-aim stack.
It consumes detector output plus camera state, produces tracked targets and gimbal
commands, and keeps the old public ROS interfaces stable while the internal
targeting pipeline is consolidated into one package.

## Canonical entrypoint

Use the owner-path launch:

- [`debug_targeting.launch.py`](/Users/salvatoremarchese/Documents/Coding/Robotics/CV_AI-Navigation/src/launch_pkg/launch/debug_targeting.launch.py)

The component entrypoint is `rm_auto_aim::AutoAimNode`.

## Module map

- `pipeline/`
  - Node entrypoint, grouped config, frame handoff types
- `measurement/`
  - Detector-to-armor measurement building, PnP, camera-frame conversion
- `tracking/`
  - EKF-backed per-target tracking, tracker-set coordination, tracker construction
- `planning/`
  - Prediction, plan selection, ballistics, latency compensation, fire gating
- `io/`
  - Pose-source adaptation, command shaping, debug/output publication

## Runtime flow

Each detection frame follows this sequence:

1. `AutoAimNode` receives `/detector/armors`
2. `GimbalPoseAdapter` updates the current gimbal pose state
3. `DetectionConverter` turns detector boxes into tracked-frame armor measurements
4. `Trackers` advances, matches, updates, spawns, and prunes the tracker set
5. `Trackers` emits `TrackSnapshot` objects
6. `LatencyCompensator` and `ShotPlanner` build a planning context
7. `EngagementPlanner` selects the best `AimDecision`
8. `FireGate` decides whether the selected shot may fire
9. `TargetingOutputPublisher` publishes the legacy target, marker, and command topics

## Where to change what

| If you need to change... | Start here |
| --- | --- |
| Detector boxes to tracked-frame armor measurements | `measurement/detection_converter.*` |
| PnP geometry and image-to-pose math | `measurement/pnp_solver.*` |
| Tracker lifecycle and assignment sequence | `tracking/runtime_manager.*` |
| EKF process / measurement model | `tracking/runtime_factory.*` and `tracking/ekf.*` |
| Tracker-owned face prediction / stationary behavior | `tracking/target_tracker.*` |
| Visible-direct / predicted-direct / indirect planning | `planning/predictor.*` |
| Best-target selection | `planning/engagement_planner.*` |
| Command smoothing / clamp semantics | `io/gimbal_command_controller.*` |
| Published ROS outputs and debug markers | `io/output_publisher.*` and `io/debug_publisher.*` |

## Important invariants

- Public topics and parameter names remain compatible with the old runtime.
- Existing maintained launches stay untouched; only `debug_targeting.launch.py` is the owner-path launch.
- `pose_source='none'` remains a first-class mode.
- `RAW` and `CMD` are intentionally different concepts: raw intercept vs clamped/smoothed command.
