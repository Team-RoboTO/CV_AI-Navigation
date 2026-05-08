#include <gtest/gtest.h>

#include <cmath>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "auto_aim_targeting/measurement/detection_converter.hpp"

namespace rm_auto_aim
{
namespace
{

double yawOf(const auto_aim_interfaces::msg::Armor & armor)
{
  tf2::Quaternion q;
  tf2::fromMsg(armor.pose.orientation, q);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

auto_aim_interfaces::msg::Armor makeArmor(double x, double y, double yaw)
{
  auto_aim_interfaces::msg::Armor armor;
  armor.pose.position.x = x;
  armor.pose.position.y = y;
  armor.pose.position.z = 0.3;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  armor.pose.orientation = tf2::toMsg(q);
  return armor;
}

TEST(DetectionConverterTest, CorrectsFlippedPnpYawToInwardBearing)
{
  auto armor = makeArmor(3.0, 0.0, M_PI);

  const bool changed = correctArmorYawAgainstBearing(armor, 65.0 * M_PI / 180.0);

  EXPECT_TRUE(changed);
  EXPECT_NEAR(yawOf(armor), 0.0, 1e-9);
}

TEST(DetectionConverterTest, FallsBackToBearingWhenYawRemainsTooOblique)
{
  auto armor = makeArmor(3.0, 3.0, 2.0);

  const bool changed = correctArmorYawAgainstBearing(armor, 20.0 * M_PI / 180.0);

  EXPECT_TRUE(changed);
  EXPECT_NEAR(yawOf(armor), std::atan2(3.0, 3.0), 1e-9);
}

TEST(DetectionConverterTest, LeavesConsistentYawUntouched)
{
  auto armor = makeArmor(3.0, 0.0, 0.1);

  const bool changed = correctArmorYawAgainstBearing(armor, 65.0 * M_PI / 180.0);

  EXPECT_FALSE(changed);
  EXPECT_NEAR(yawOf(armor), 0.1, 1e-9);
}

}  // namespace
}  // namespace rm_auto_aim
