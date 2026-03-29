#ifndef AUTO_AIM_TARGETING__PLANNING__ENGAGEMENT_PLANNER_HPP_
#define AUTO_AIM_TARGETING__PLANNING__ENGAGEMENT_PLANNER_HPP_

#include <optional>
#include <vector>

#include <rclcpp/time.hpp>

#include "auto_aim_targeting/planning/shot_planner.hpp"
#include "auto_aim_targeting/planning_types.hpp"

namespace rm_auto_aim
{

class EngagementPlanner
{
public:
  EngagementPlanner(const ShotPlanner & predictor, const CostWeights & weights)
  : shot_planner_(predictor), weights_(weights) {}

  std::optional<AimDecision> selectBestPlan(
    const std::vector<TrackSnapshot> & snapshots,
    const PlanningContext & ctx,
    const rclcpp::Time & stamp) const;

private:
  double computePlanScore(
    const TrackSnapshot & target,
    const PlanningContext & ctx,
    ShotPlan & plan) const;

  const ShotPlanner & shot_planner_;
  CostWeights weights_;
};

}  // namespace rm_auto_aim

#endif  // AUTO_AIM_TARGETING__PLANNING__ENGAGEMENT_PLANNER_HPP_
