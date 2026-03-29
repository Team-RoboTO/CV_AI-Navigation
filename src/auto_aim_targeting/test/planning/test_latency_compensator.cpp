#include <gtest/gtest.h>

#include "auto_aim_targeting/planning/latency_compensator.hpp"

namespace rm_auto_aim
{
namespace
{

TEST(LatencyCompensatorTest, UpdatesBiasDuringWarmup)
{
  LatencyCompensator compensator(0.08, 0.5, 2.5, 5);
  const rclcpp::Time frame_stamp(1, 0, RCL_ROS_TIME);
  const rclcpp::Time now(1, 100000000, RCL_ROS_TIME);

  compensator.updateFromFrameStamp(now, frame_stamp);

  EXPECT_GT(compensator.currentBias(), 0.08);
}

TEST(LatencyCompensatorTest, ComputesNonNegativeTransportDelay)
{
  LatencyCompensator compensator(0.08, 0.5, 2.5, 5);
  const rclcpp::Time frame_stamp(1, 0, RCL_ROS_TIME);
  const rclcpp::Time now(1, 50000000, RCL_ROS_TIME);

  EXPECT_DOUBLE_EQ(compensator.transportDelay(now, frame_stamp), 0.05);
  EXPECT_DOUBLE_EQ(compensator.transportDelay(frame_stamp, now), 0.0);
}

}  // namespace
}  // namespace rm_auto_aim
