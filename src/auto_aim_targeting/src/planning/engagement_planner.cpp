// ============================================================================
// engagement_planner.cpp — Selects the best shot plan across all tracked targets.
//
// ENTRYPOINT: selectBestPlan()
//   Iterates all tracker snapshots, solves a ShotPlan for each, scores via
//   multi-objective cost function, returns the lowest-cost plan.
//
// HELPER (stepdown):
//   computePlanScore → weighted cost from range, slew, uncertainty, visibility
// ============================================================================
#include "auto_aim_targeting/planning/engagement_planner.hpp"

#include <algorithm>
#include <cmath>

namespace rm_auto_aim
{

std::optional<AimDecision> EngagementPlanner::selectBestPlan(
  const std::vector<TrackSnapshot> & snapshots,
  const PlanningContext & ctx,
  const rclcpp::Time & stamp) const
{
  AimDecision best_decision;
  bool found = false;

  for (const auto & target : snapshots) {
    if (!target.tracking) {
      continue;
    }

    const double abs_vyaw = std::abs(target.v_yaw);
    const bool same_as_previous = (target.tracker_id == ctx.previous_tracker_id);
    const bool use_indirect =
      target.armors_num > 1 &&
      ((same_as_previous && ctx.previous_indirect_mode)
        ? (abs_vyaw > 0.7 * ctx.indirect_vyaw_threshold)
        : (abs_vyaw > ctx.indirect_vyaw_threshold));

    std::optional<ShotPlan> plan_opt;
    if (!use_indirect && target.matched_face.valid &&
        target.matched_face.fresh && target.matched_face.observable) {
      plan_opt = this->shot_planner_.solveVisibleDirect(target, ctx);
    }
    if (!plan_opt.has_value()) {
      plan_opt = use_indirect ? this->shot_planner_.solveIndirect(target, ctx)
                              : this->shot_planner_.solvePredictedDirect(target, ctx);
    }
    if (!plan_opt.has_value()) {
      continue;
    }

    ShotPlan plan = *plan_opt;
    const double base_age = (stamp - target.last_measurement_stamp).seconds();
    plan.measurement_age = std::max(0.0, base_age + ctx.transport_delay);
    plan.measurement_stale = plan.measurement_age > ctx.max_measurement_age;
    this->computePlanScore(target, ctx, plan);

    if (!found || plan.score < best_decision.plan.score) {
      best_decision.track = target;
      best_decision.plan = plan;
      found = true;
    }
  }

  if (!found) {
    return std::nullopt;
  }
  return best_decision;
}

double EngagementPlanner::computePlanScore(
  const TrackSnapshot & target,
  const PlanningContext & ctx,
  ShotPlan & plan) const
{
  const double yaw_budget = std::max(
    ctx.max_gimbal_yaw_rate * std::max(plan.predict_time, 0.03), 1e-3);
  const double pitch_budget = std::max(
    ctx.max_gimbal_pitch_rate * std::max(plan.predict_time, 0.03), 1e-3);
  const double slew_cost =
    std::pow(plan.relative_yaw / yaw_budget, 2) +
    std::pow(plan.relative_pitch / pitch_budget, 2);

  const double uncertainty_cost =
    std::pow(plan.sigma_angular / std::max(ctx.angular_window, 1e-3), 2);
  const double visibility_cost =
    std::pow(std::max(0.0, 1.0 - plan.visibility), ctx.visibility_exponent);

  double score = 0.0;
  score += this->weights_.range * (plan.range / std::max(ctx.max_fire_dist, 1.0));
  score += this->weights_.flight_time * plan.flight_time;
  score += this->weights_.uncertainty * uncertainty_cost;
  score += this->weights_.slew * slew_cost;
  score += this->weights_.staleness * plan.measurement_age;
  score += this->weights_.low_visibility * visibility_cost;

  if (plan.temp_lost) {
    score += this->weights_.temp_lost;
  }
  if (ctx.previous_tracker_id >= 0 && ctx.previous_tracker_id != target.tracker_id) {
    score += this->weights_.switch_target;
  }
  if (plan.fire_window_margin < 0.0) {
    score += this->weights_.negative_margin * std::abs(plan.fire_window_margin);
  }
  if (!plan.ballistic_valid || plan.range < ctx.min_fire_dist || plan.range > ctx.max_fire_dist) {
    score += 10.0;
  }
  if (!plan.reachable) {
    score += 2.0;
  }

  plan.score = score;
  return score;
}

}  // namespace rm_auto_aim
