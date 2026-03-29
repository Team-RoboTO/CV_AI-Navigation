#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>

#include "auto_aim_targeting/io/gimbal_pose_adapter.hpp"

namespace rm_auto_aim
{
namespace
{

class GimbalPoseAdapterTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      char ** argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(GimbalPoseAdapterTest, BroadcastIncludesPivotHeightAndCameraOffsetsAtNeutralPose)
{
  auto node = std::make_shared<rclcpp::Node>("pose_source_adapter_neutral_test");
  auto buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

  PoseConfig config;
  config.pose_source = "micro_pose";
  config.gimbal_height = 0.325;
  config.camera_offset_x = 0.107;
  config.camera_offset_z = 0.136;

  GimbalPoseAdapter adapter(
    config,
    "odom",
    buffer,
    broadcaster,
    node->get_logger(),
    node->get_clock());

  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = node->now();
  msg.pose.position.x = 0.0;
  msg.pose.position.y = 0.0;

  adapter.onMicroPose(std::make_shared<const geometry_msgs::msg::PoseStamped>(msg));

  const auto transform = buffer->lookupTransform(
    "odom",
    "camera_color_optical_frame",
    rclcpp::Time(0, 0, node->get_clock()->get_clock_type()),
    rclcpp::Duration::from_seconds(0.1));

  EXPECT_NEAR(transform.transform.translation.x, 0.107, 1e-9);
  EXPECT_NEAR(transform.transform.translation.y, 0.0, 1e-9);
  EXPECT_NEAR(transform.transform.translation.z, 0.325 + 0.136, 1e-9);
}

TEST_F(GimbalPoseAdapterTest, BroadcastRotatesCameraOffsetWithYaw)
{
  auto node = std::make_shared<rclcpp::Node>("pose_source_adapter_yaw_test");
  auto buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

  PoseConfig config;
  config.pose_source = "micro_pose";
  config.gimbal_height = 0.325;
  config.camera_offset_x = 0.107;
  config.camera_offset_z = 0.136;

  GimbalPoseAdapter adapter(
    config,
    "odom",
    buffer,
    broadcaster,
    node->get_logger(),
    node->get_clock());

  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = node->now();
  msg.pose.position.x = 0.0;
  msg.pose.position.y = M_PI / 2.0;

  adapter.onMicroPose(std::make_shared<const geometry_msgs::msg::PoseStamped>(msg));

  const auto transform = buffer->lookupTransform(
    "odom",
    "camera_color_optical_frame",
    rclcpp::Time(0, 0, node->get_clock()->get_clock_type()),
    rclcpp::Duration::from_seconds(0.1));

  EXPECT_NEAR(transform.transform.translation.x, 0.0, 1e-9);
  EXPECT_NEAR(transform.transform.translation.y, 0.107, 1e-9);
  EXPECT_NEAR(transform.transform.translation.z, 0.325 + 0.136, 1e-9);
}

}  // namespace
}  // namespace rm_auto_aim
