#include <gtest/gtest.h>

#include <cmath>

#include "auto_aim_targeting/io/gimbal_command_controller.hpp"

namespace rm_auto_aim
{
namespace
{

TEST(CommandControllerTest, HoldOutputResetsCommandAndDeletesMarker)
{
  GimbalCommandController controller(0.5, 30.0, 180.0);
  std_msgs::msg::Header header;
  header.frame_id = "odom";

  const auto output = controller.makeHoldOutput(header);

  EXPECT_DOUBLE_EQ(output.cmd.pitch, 0.0);
  EXPECT_DOUBLE_EQ(output.cmd.yaw, 0.0);
  EXPECT_DOUBLE_EQ(output.cmd.distance, 0.0);
  EXPECT_FALSE(output.cmd.fire_cmd);
  EXPECT_EQ(output.marker.action, visualization_msgs::msg::Marker::DELETE);
}

TEST(CommandControllerTest, ShotOutputClampsAndSmoothsRelativeAngles)
{
  GimbalCommandController controller(0.5, 30.0, 180.0);
  std_msgs::msg::Header header;
  header.frame_id = "odom";

  ShotPlan first_plan;
  first_plan.relative_yaw = 45.0 * M_PI / 180.0;
  first_plan.relative_pitch = -30.0 * M_PI / 180.0;
  first_plan.range = 4.2;
  first_plan.x = 1.0;
  first_plan.y = 2.0;
  first_plan.z = 3.0;

  const auto first = controller.makeShotOutput(header, first_plan, true, true);
  EXPECT_NEAR(first.cmd.yaw, 45.0, 1e-6);
  EXPECT_NEAR(first.cmd.pitch, -30.0, 1e-6);
  EXPECT_TRUE(first.cmd.fire_cmd);
  EXPECT_DOUBLE_EQ(first.cmd.distance, 4.2);
  EXPECT_EQ(first.marker.action, visualization_msgs::msg::Marker::ADD);
  EXPECT_DOUBLE_EQ(first.marker.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(first.marker.pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(first.marker.pose.position.z, 3.0);

  ShotPlan second_plan;
  second_plan.relative_yaw = 0.0;
  second_plan.relative_pitch = 0.0;
  second_plan.range = 4.2;
  const auto second = controller.makeShotOutput(header, second_plan, false, false);

  EXPECT_NEAR(second.cmd.yaw, 22.5, 1e-6);
  EXPECT_NEAR(second.cmd.pitch, -15.0, 1e-6);
  EXPECT_FALSE(second.cmd.fire_cmd);
}

}  // namespace
}  // namespace rm_auto_aim
