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
  config.fire_yaw_tolerance = 0.15;
  config.fire_pitch_tolerance = 0.15;
  config.pose_source_is_none = false;
  config.allow_fire_without_pose = false;
  return config;
}

ShotPlan makeGoodPlan()
{
  ShotPlan plan;
  plan.measurement_stale = false;
  plan.temp_lost = false;
  plan.tracking_valid = true;
  plan.ballistic_valid = true;
  plan.reachable = true;
  plan.range = 5.0;
  plan.fire_window_margin = 0.01;
  plan.predict_time = 0.1;
  plan.relative_yaw = 0.01;
  plan.relative_pitch = 0.01;
  return plan;
}

TEST(FireGateTest, BlocksOnStaleMeasurement)
{
  FireGate gate(makeConfig());
  ShotPlan plan = makeGoodPlan();
  plan.measurement_stale = true;

  const auto result = gate.evaluate(plan, true, true);
  EXPECT_FALSE(result.fire);
  EXPECT_EQ(result.blocker, "stale_measurement");
}

TEST(FireGateTest, AllowsFireWhenAllChecksPass)
{
  FireGate gate(makeConfig());
  ShotPlan plan = makeGoodPlan();

  const auto result = gate.evaluate(plan, true, true);
  EXPECT_TRUE(result.fire);
  EXPECT_TRUE(result.blocker.empty());
}

TEST(FireGateTest, BlocksWhenNoPoseSourceAndNotAllowed)
{
  auto config = makeConfig();
  config.allow_fire_without_pose = false;
  FireGate gate(config);
  ShotPlan plan = makeGoodPlan();

  const auto result = gate.evaluate(plan, true, false);
  EXPECT_FALSE(result.fire);
  EXPECT_EQ(result.blocker, "no_pose_source");
}

TEST(FireGateTest, AllowsFireWithoutPoseWhenExplicitlyAllowed)
{
  auto config = makeConfig();
  config.allow_fire_without_pose = true;
  FireGate gate(config);
  ShotPlan plan = makeGoodPlan();

  const auto result = gate.evaluate(plan, false, false);
  EXPECT_TRUE(result.fire);
}

TEST(FireGateTest, BlocksOnNegativeWindowMargin)
{
  FireGate gate(makeConfig());
  ShotPlan plan = makeGoodPlan();
  plan.fire_window_margin = -0.05;

  const auto result = gate.evaluate(plan, true, true);
  EXPECT_FALSE(result.fire);
  EXPECT_EQ(result.blocker, "negative_window");
}

TEST(FireGateTest, BlocksWhenTempLost)
{
  FireGate gate(makeConfig());
  ShotPlan plan = makeGoodPlan();
  plan.temp_lost = true;

  const auto result = gate.evaluate(plan, true, true);
  EXPECT_FALSE(result.fire);
  EXPECT_EQ(result.blocker, "temp_lost");
}

TEST(FireGateTest, BlocksOnStalePoseWhenPoseSourceExists)
{
  FireGate gate(makeConfig());
  ShotPlan plan = makeGoodPlan();

  const auto result = gate.evaluate(plan, false, true);
  EXPECT_FALSE(result.fire);
  EXPECT_EQ(result.blocker, "stale_pose");
}

}  // namespace
}  // namespace rm_auto_aim
