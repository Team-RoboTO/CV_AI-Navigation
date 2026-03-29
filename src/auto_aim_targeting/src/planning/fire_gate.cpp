// ============================================================================
// fire_gate.cpp — Final fire permission check before shooting.
//
// ENTRYPOINT: FireGate::evaluate()
//   Validates safety constraints (pose freshness, measurement age, ballistics,
//   range, slew rate).  Returns FireGateResult with fire=true or a blocker
//   string explaining why fire was denied.
// ============================================================================
#include "auto_aim_targeting/planning/fire_gate.hpp"

#include <algorithm>
#include <cmath>

namespace rm_auto_aim
{

FireGateResult FireGate::evaluate(const ShotPlan & plan, bool pose_fresh) const
{
  FireGateResult result;
  if (!pose_fresh) {
    result.blocker = "stale_pose";
    return result;
  }
  if (plan.measurement_stale) {
    result.blocker = "stale_measurement";
    return result;
  }
  if (plan.temp_lost) {
    result.blocker = "temp_lost";
    return result;
  }
  if (!plan.ballistic_valid) {
    result.blocker = "invalid_ballistics";
    return result;
  }
  if (!plan.reachable) {
    result.blocker = "unreachable_pitch";
    return result;
  }
  if (plan.range < this->config_.min_fire_dist || plan.range > this->config_.max_fire_dist) {
    result.blocker = "out_of_range";
    return result;
  }
  if (plan.fire_window_margin < 0.0) {
    result.blocker = "negative_window";
    return result;
  }

  const bool can_slew =
    std::abs(plan.relative_yaw) <=
      this->config_.max_gimbal_yaw_rate * std::max(plan.predict_time, 0.03) &&
    std::abs(plan.relative_pitch) <=
      this->config_.max_gimbal_pitch_rate * std::max(plan.predict_time, 0.03);
  if (!can_slew) {
    result.blocker = "slew_limit";
    return result;
  }

  result.fire = true;
  return result;
}

}  // namespace rm_auto_aim
