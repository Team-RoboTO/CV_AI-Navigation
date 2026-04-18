#ifndef AUTO_AIM_TARGETING__PLANNING__FIRE_GATE_HPP_
#define AUTO_AIM_TARGETING__PLANNING__FIRE_GATE_HPP_

#include <utility>

#include "auto_aim_targeting/types.hpp"
#include "auto_aim_targeting/config.hpp"

namespace rm_auto_aim
{

class FireGate
{
public:
  explicit FireGate(EngagementConfig config) : config_(std::move(config)) {}

  // Deterministic fire permission check.
  // pose_fresh: whether gimbal feedback is available and recent.
  // has_pose_source: whether a pose source is configured at all.
  FireGateResult evaluate(
    const ShotPlan & plan,
    bool pose_fresh,
    bool has_pose_source) const;

private:
  EngagementConfig config_;
};

}  // namespace rm_auto_aim

#endif  // AUTO_AIM_TARGETING__PLANNING__FIRE_GATE_HPP_
