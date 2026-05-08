#include <gtest/gtest.h>

#include <cmath>

#include "auto_aim_targeting/io/gimbal_command_controller.hpp"

namespace rm_auto_aim
{
namespace
{

TEST(CommandControllerTest, HoldOutputResetsCommandAndDeletesMarker)
{
  GimbalCommandController controller(30.0, 180.0);
  std_msgs::msg::Header header;
  header.frame_id = "odom";

  const auto output = controller.makeHoldOutput(header);

  EXPECT_DOUBLE_EQ(output.cmd.pitch, 0.0);
  EXPECT_DOUBLE_EQ(output.cmd.yaw, 0.0);
  EXPECT_DOUBLE_EQ(output.cmd.distance, 0.0);
  EXPECT_FALSE(output.cmd.fire_cmd);
  EXPECT_EQ(output.marker.action, visualization_msgs::msg::Marker::DELETE);
}

TEST(CommandControllerTest, ShotOutputUsesAbsoluteAngles)
{
  GimbalCommandController controller(30.0, 180.0);
  std_msgs::msg::Header header;
  header.frame_id = "odom";

  ShotPlan plan;
  plan.absolute_yaw = 45.0 * M_PI / 180.0;
  plan.absolute_pitch = -20.0 * M_PI / 180.0;
  plan.range = 4.2;
  plan.x = 1.0;
  plan.y = 2.0;
  plan.z = 3.0;

  const auto output = controller.makeShotOutput(header, plan, true);
  EXPECT_NEAR(output.cmd.yaw, 45.0, 1e-4);
  EXPECT_NEAR(output.cmd.pitch, -20.0, 1e-4);
  EXPECT_TRUE(output.cmd.fire_cmd);
  EXPECT_DOUBLE_EQ(output.cmd.distance, 4.2);
  EXPECT_EQ(output.marker.action, visualization_msgs::msg::Marker::ADD);
  EXPECT_DOUBLE_EQ(output.marker.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(output.marker.pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(output.marker.pose.position.z, 3.0);
}

TEST(CommandControllerTest, ShotOutputClampsAngles)
{
  GimbalCommandController controller(30.0, 90.0);
  std_msgs::msg::Header header;
  header.frame_id = "odom";

  ShotPlan plan;
  plan.absolute_yaw = 120.0 * M_PI / 180.0;
  plan.absolute_pitch = -45.0 * M_PI / 180.0;
  plan.range = 3.0;

  const auto output = controller.makeShotOutput(header, plan, false);
  EXPECT_NEAR(output.cmd.yaw, 90.0, 1e-4);
  EXPECT_NEAR(output.cmd.pitch, -30.0, 1e-4);
  EXPECT_FALSE(output.cmd.fire_cmd);
}

TEST(CommandControllerTest, ClampedShotSuppressesFire)
{
  GimbalCommandController controller(30.0, 90.0);
  std_msgs::msg::Header header;
  header.frame_id = "odom";

  ShotPlan plan;
  plan.absolute_yaw = 120.0 * M_PI / 180.0;
  plan.absolute_pitch = 0.0;
  plan.range = 3.0;

  const auto output = controller.makeShotOutput(header, plan, true);
  EXPECT_NEAR(output.cmd.yaw, 90.0, 1e-4);
  EXPECT_FALSE(output.cmd.fire_cmd);
  EXPECT_DOUBLE_EQ(output.twist.angular.x, 0.0);
}

TEST(CommandControllerTest, SmoothingSuppressesFireUntilCommandCatchesUp)
{
  GimbalCommandController controller(30.0, 180.0, 0.4, 0.03, 0.03);
  std_msgs::msg::Header header;
  header.frame_id = "odom";

  ShotPlan first;
  first.tracker_id = 1;
  first.absolute_yaw = 0.0;
  first.absolute_pitch = 0.0;
  first.range = 3.0;
  auto output = controller.makeShotOutput(header, first, true);
  EXPECT_TRUE(output.cmd.fire_cmd);

  ShotPlan moved = first;
  moved.absolute_yaw = 20.0 * M_PI / 180.0;
  output = controller.makeShotOutput(header, moved, true);
  EXPECT_FALSE(output.cmd.fire_cmd);
  EXPECT_LT(output.cmd.yaw, 20.0);
}

TEST(CommandControllerTest, SmoothingResetsWhenTrackerChanges)
{
  GimbalCommandController controller(30.0, 180.0, 0.4, 0.03, 0.03);
  std_msgs::msg::Header header;
  header.frame_id = "odom";

  ShotPlan first;
  first.tracker_id = 1;
  first.absolute_yaw = 0.0;
  first.absolute_pitch = 0.0;
  first.range = 3.0;
  auto output = controller.makeShotOutput(header, first, true);
  EXPECT_TRUE(output.cmd.fire_cmd);

  ShotPlan second = first;
  second.tracker_id = 2;
  second.absolute_yaw = 20.0 * M_PI / 180.0;
  output = controller.makeShotOutput(header, second, true);

  EXPECT_NEAR(output.cmd.yaw, 20.0, 1e-4);
  EXPECT_TRUE(output.cmd.fire_cmd);
}

}  // namespace
}  // namespace rm_auto_aim
