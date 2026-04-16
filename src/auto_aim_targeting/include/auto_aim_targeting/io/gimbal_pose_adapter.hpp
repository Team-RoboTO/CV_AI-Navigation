#ifndef AUTO_AIM_TARGETING__IO__GIMBAL_POSE_ADAPTER_HPP_
#define AUTO_AIM_TARGETING__IO__GIMBAL_POSE_ADAPTER_HPP_

#include <memory>
#include <mutex>
#include <utility>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>

#include "auto_aim_targeting/types.hpp"
#include "auto_aim_targeting/config.hpp"

namespace rm_auto_aim
{

class GimbalPoseAdapter
{
public:
  GimbalPoseAdapter(
    PoseConfig config,
    std::string target_frame,
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer,
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster,
    rclcpp::Logger logger,
    rclcpp::Clock::SharedPtr clock);

  void onMicroPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr & msg);
  void onCameraImu(const sensor_msgs::msg::Imu::ConstSharedPtr & msg);
  void updateForFrame(const rclcpp::Time & stamp);
  bool isFresh(const rclcpp::Time & now) const;
  GimbalPoseState currentPoseState(const rclcpp::Time & now) const;

private:
  bool isFreshLocked(const rclcpp::Time & now) const;
  void broadcastCameraTF(const rclcpp::Time & stamp);

  PoseConfig config_;
  std::string target_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

  mutable std::mutex pose_mutex_;
  tf2::Quaternion q_orientation_{tf2::Quaternion::getIdentity()};
  double current_pitch_ = 0.0;
  double current_yaw_ = 0.0;
  rclcpp::Time last_self_orientation_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace rm_auto_aim

#endif  // AUTO_AIM_TARGETING__IO__GIMBAL_POSE_ADAPTER_HPP_
