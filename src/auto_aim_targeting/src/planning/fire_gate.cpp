// ============================================================================
// fire_gate.cpp — Deterministic fire permission gate.
//
// Checks are ordered from cheapest/most-likely-to-block first.
// Each check returns a specific blocker string for diagnostics.
//
// Key design decisions:
//   - No relative_yaw/pitch check: the gimbal controller downstream handles
//     aim convergence. The fire gate only checks world-frame conditions that
//     are independent of gimbal feedback quality.
//   - Pose freshness is a soft gate: without pose, we still allow fire if
//     allow_fire_without_pose is set (for testing without IMU/micro_pose).
//   - fire_window_margin is THE primary angular gate: it already accounts for
//     face obliquity, EKF uncertainty, distance scaling, and angular window.
// ============================================================================
#include "auto_aim_targeting/planning/fire_gate.hpp"

#include <cmath>

namespace rm_auto_aim
{

FireGateResult FireGate::evaluate(
  const ShotPlan & plan,
  bool pose_fresh,
  bool has_pose_source) const
{
  FireGateResult result;

  // 1. Tracking validity: must be in TRACKING state (not DETECTING or LOST)
  if (!plan.tracking_valid) {
    result.blocker = "not_tracking";
    return result;
  }

  // 2. TEMP_LOST: target briefly occluded — prediction-only aim, suppress fire
  if (plan.temp_lost) {
    result.blocker = "temp_lost";
    return result;
  }

  // 3. Measurement freshness: reject if detection is too old
  if (plan.measurement_stale) {
    result.blocker = "stale_measurement";
    return result;
  }

  // 4. Ballistic validity: physics must produce a real solution
  if (!plan.ballistic_valid) {
    result.blocker = "invalid_ballistics";
    return result;
  }

  // 5. Physical reachability: gimbal must be able to reach the pitch angle
  if (!plan.reachable) {
    result.blocker = "unreachable_pitch";
    return result;
  }

  // 6. Range bounds
  if (plan.range < this->config_.min_fire_dist || plan.range > this->config_.max_fire_dist) {
    result.blocker = "out_of_range";
    return result;
  }

  // 7. Angular margin: the shot planner already computed whether the face
  //    angle error fits within the distance-scaled angular window minus
  //    EKF uncertainty. Negative margin = face too oblique or prediction
  //    too uncertain for a reliable hit.
  if (plan.fire_window_margin < 0.0) {
    result.blocker = "negative_window";
    return result;
  }

  // 8. Pose source check: if a pose source is configured, it must be fresh.
  //    If no pose source exists, fire is gated by allow_fire_without_pose.
  if (has_pose_source && !pose_fresh) {
    result.blocker = "stale_pose";
    return result;
  }
  if (!has_pose_source && !this->config_.allow_fire_without_pose) {
    result.blocker = "no_pose_source";
    return result;
  }

  result.fire = true;
  return result;
}

}  // namespace rm_auto_aim
