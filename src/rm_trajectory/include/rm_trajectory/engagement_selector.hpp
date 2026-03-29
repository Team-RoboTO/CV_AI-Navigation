#ifndef RM_TRAJECTORY__ENGAGEMENT_SELECTOR_HPP_
#define RM_TRAJECTORY__ENGAGEMENT_SELECTOR_HPP_

#include <optional>

#include "auto_aim_interfaces/msg/targets.hpp"
#include "rm_trajectory/armor_predictor.hpp"
#include "rm_trajectory/trajectory_types.hpp"

namespace rm_auto_aim
{

class EngagementSelector
{
public:
  EngagementSelector(
    const ArmorPredictor & predictor,
    CostWeights weights)
  : predictor_(predictor), weights_(weights) {}

  std::optional<ShotPlan> selectBestTarget(
    const auto_aim_interfaces::msg::Targets & targets_msg,
    const SolverContext & ctx) const;

private:
  double computeScore(
    const auto_aim_interfaces::msg::Target & target,
    const SolverContext & ctx,
    ShotPlan & plan) const;

  const ArmorPredictor & predictor_;
  CostWeights weights_;
};

}  // namespace rm_auto_aim

#endif  // RM_TRAJECTORY__ENGAGEMENT_SELECTOR_HPP_
