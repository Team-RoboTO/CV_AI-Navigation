#include <rclcpp/rclcpp.hpp>
#include "rm_trajectory/engagement_selector.hpp"

#include <cmath>
#include <limits>

namespace rm_auto_aim
{

double EngagementSelector::computeScore(
  const auto_aim_interfaces::msg::Target & target,
  const SolverContext & ctx,
  ShotPlan & plan) const
{
  // SCORE SYSTEM: Lower is better!
  // This function evaluates a target's quality. We penalize high distances, large movements (slew),
  // old measurements (staleness), and poor visibility (plate not facing us perfectly).

  const double yaw_budget = std::max(ctx.max_gimbal_yaw_rate * std::max(plan.predict_time, 0.03), 1e-3);
  const double pitch_budget = std::max(ctx.max_gimbal_pitch_rate * std::max(plan.predict_time, 0.03), 1e-3);
  const double slew_cost =
    std::pow(plan.relative_yaw / yaw_budget, 2) +
    std::pow(plan.relative_pitch / pitch_budget, 2);

  const double uncertainty_cost =
    std::pow(plan.sigma_angular / std::max(ctx.angular_window, 1e-3), 2);

  const double visibility_cost =
    std::pow(std::max(0.0, 1.0 - plan.visibility), ctx.visibility_exponent);

  double score = 0.0;
  score += weights_.range * (plan.range / std::max(ctx.max_fire_dist, 1.0));
  score += weights_.flight_time * plan.flight_time;
  score += weights_.uncertainty * uncertainty_cost;
  score += weights_.slew * slew_cost;
  score += weights_.staleness * plan.measurement_age;
  score += weights_.low_visibility * visibility_cost;

  if (plan.temp_lost) {
    score += weights_.temp_lost;
  }
  if (ctx.previous_tracker_id >= 0 && ctx.previous_tracker_id != target.tracker_id) {
    score += weights_.switch_target;
  }
  if (plan.fire_window_margin < 0.0) {
    score += weights_.negative_margin * std::abs(plan.fire_window_margin);
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

std::optional<ShotPlan> EngagementSelector::selectBestTarget(
  const auto_aim_interfaces::msg::Targets & targets_msg,
  const SolverContext & ctx) const
{
  // MULTI-TARGET FILTER: Iterates through all detected robots to pick the easiest kill.

  ShotPlan best;
  bool found = false;

  for (const auto & target : targets_msg.targets) {
    if (!target.tracking) {
      continue;
    }

    const double abs_vyaw = std::abs(target.v_yaw);
    const bool same_as_previous = (target.tracker_id == ctx.previous_tracker_id);
    const double enter_thr = ctx.indirect_vyaw_threshold;
    const double exit_thr = 0.7 * ctx.indirect_vyaw_threshold;

    // HYSTERESIS LOGIC:
    // Prevents the turret from jittering back and forth between Direct and Indirect modes
    // when the enemy robot speed hovers exactly at the threshold.
    // If we're already in Indirect, we use a lower threshold to stay in it.
    const bool use_indirect =
      target.armors_num > 1 &&
      ((same_as_previous && ctx.previous_indirect_mode)
        ? (abs_vyaw > exit_thr)
        : (abs_vyaw > enter_thr));

    auto plan_opt = use_indirect ? predictor_.solveIndirect(target, ctx)
                                 : predictor_.solveDirect(target, ctx);
    if (!plan_opt.has_value()) {
      continue;
    }
    
    ShotPlan plan = *plan_opt;

    const double base_age =
      (rclcpp::Time(targets_msg.header.stamp) - rclcpp::Time(target.last_measurement_stamp)).seconds();
    const double measurement_age = std::max(0.0, base_age + ctx.transport_delay);
    plan.measurement_age = measurement_age;
    plan.measurement_stale = measurement_age > ctx.max_measurement_age;

    computeScore(target, ctx, plan);
    if (!found || plan.score < best.score) {
      best = plan;
      found = true;
    }
  }

  if (!found) {
    return std::nullopt;
  }
  return best;
}

}  // namespace rm_auto_aim
