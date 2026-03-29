#include <gtest/gtest.h>

#include "auto_aim_targeting/planning/fire_gate.hpp"

namespace rm_auto_aim
{
namespace
{

EngagementConfig makeConfig()
{
  EngagementConfig config;
  config.min_fire_dist = 0.5;
  config.max_fire_dist = 10.0;
  config.max_gimbal_yaw_rate = 6.0;
  config.max_gimbal_pitch_rate = 4.0;
  return config;
}

TEST(FireGateTest, BlocksOnStaleMeasurement)
{
  FireGate gate(makeConfig());
  ShotPlan plan;
  plan.measurement_stale = true;

  const auto result = gate.evaluate(plan, true);
  EXPECT_FALSE(result.fire);
  EXPECT_EQ(result.blocker, "stale_measurement");
}

TEST(FireGateTest,AllowsFireWhenAllChecksPass)
{
  FireGate gate(makeConfig());
  ShotPlan plan;
  plan.measurement_stale = false;
  plan.temp_lost = false;
  plan.ballistic_valid = true;
  plan.reachable = true;
  plan.range = 5.0;
  plan.fire_window_margin = 0.01;
  plan.predict_time = 0.1;
  plan.relative_yaw = 0.1;
  plan.relative_pitch = 0.1;

  const auto result = gate.evaluate(plan, true);
  EXPECT_TRUE(result.fire);
  EXPECT_TRUE(result.blocker.empty());
}

}  // namespace
}  // namespace rm_auto_aim
