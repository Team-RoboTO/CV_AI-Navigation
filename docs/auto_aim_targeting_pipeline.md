# `auto_aim_targeting` pipeline

## Entry points

- Launch entrypoint:
  - [`debug_targeting.launch.py`](/Users/salvatoremarchese/Documents/Coding/Robotics/CV_AI-Navigation/src/launch_pkg/launch/debug_targeting.launch.py)
- Component entrypoint:
  - `rm_auto_aim::TargetingNode`
- Frame callback:
  - `TargetingNode::armorsCallback()`

## Per-frame sequence

1. The launch starts the composable container and loads `rm_auto_aim::TargetingNode`.
2. `TargetingNode` receives `/detector/armors`.
3. `GimbalPoseAdapter` updates pose freshness and broadcasts the camera transform for the frame.
4. `ArmorMeasurementBuilder`:
   - reads the detector message
   - uses camera info + PnP
   - transforms armor poses into `target_frame`
   - filters implausible detections
   - returns `FrameInput` with tracked-frame armor measurements
5. `TrackerRuntimeManager`:
   - advances existing tracker predictions
   - matches measurements to trackers
   - updates matched trackers
   - spawns trackers for unmatched detections
   - removes lost trackers
   - emits `TrackSnapshot` values
6. `LatencyCompensator` updates the transport-bias estimate.
7. `ShotPlanPredictor` and `EngagementPlanner`:
   - build the planning context
   - evaluate visible-direct, predicted-direct, or indirect modes
   - choose the best `AimDecision`
8. `FireGate` evaluates whether the selected plan is allowed to fire.
9. `TargetingOutputPublisher` publishes:
   - `/tracker/target`
   - `/tracker/targets`
   - `/detections_output/optimal_target`
   - `/tracker/cmd_gimbal`
   - `/cmd_vel`
   - `/trajectory/marker`
   - `/tracker/marker`
   - `/tracker/info`

## Module ownership

| Module | Owns |
| --- | --- |
| `pipeline/node.*` | ROS entrypoint and frame orchestration |
| `pipeline/config.hpp` | grouped startup config |
| `pipeline/frame_types.hpp` | internal per-frame handoff structs |
| `measurement/armor_measurement_builder.*` | detector-to-armor measurement conversion |
| `measurement/pnp_solver.*` | image geometry to pose solve |
| `tracking/target_tracker.*` | EKF-backed tracker state, matched face, stationary handling |
| `tracking/runtime_manager.*` | per-frame coordination of the tracker set |
| `tracking/runtime_factory.*` | tracker and EKF construction |
| `planning/predictor.*` | future face prediction and shot plan construction |
| `planning/engagement_planner.*` | best-plan selection |
| `planning/ballistics_solver.*` | drag/gravity solve |
| `planning/latency_compensator.*` | frame transport bias estimation |
| `planning/fire_gate.*` | fire permission gating |
| `io/gimbal_pose_adapter.*` | pose-source adaptation and camera TF publication |
| `io/gimbal_command_controller.*` | clamp and smoothing of the command |
| `io/output_publisher.*` | ROS target/command/marker publication |
| `io/debug_publisher.*` | target/debug message formatting |

## Notes that matter in debugging

### `pose_source='none'`

This mode is valid by design. The targeting pipeline still runs with a static camera pose and must not depend on external microcontroller feedback to stay alive.

### Visible-direct vs predicted-direct

- `VISIBLE_DIRECT` follows the currently matched visible face while that face is fresh and observable.
- `PREDICTED_DIRECT` is only the fallback path when the matched face is stale or no longer observable.

### `RAW` vs `CMD`

- `RAW` is the raw intercept/impact solution.
- `CMD` is the commanded gimbal motion after clamp and smoothing.
- A difference between them can be legitimate saturation, not necessarily a planner bug.
