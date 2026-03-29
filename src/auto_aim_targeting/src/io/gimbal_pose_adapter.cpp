// ============================================================================
// gimbal_pose_adapter.cpp — Manages gimbal orientation and broadcasts camera TF.
//
// ENTRYPOINTS:
//   onMicroPose      → callback: update yaw/pitch from lower computer
//   onCameraImu      → callback: update yaw/pitch from camera IMU
//   updateForFrame   → per-frame: check staleness + broadcast TF
//   currentPoseState → query: return current yaw/pitch/freshness
//
// HELPER (stepdown):
//   broadcastCameraTF → build and send the camera_color_optical_frame transform
//   isFresh           → check if pose feedback is within timeout
// ============================================================================
#include "auto_aim_targeting/io/gimbal_pose_adapter.hpp"

#include <cmath>

#include <rclcpp/logging.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace rm_auto_aim
{

GimbalPoseAdapter::GimbalPoseAdapter(
  PoseConfig config,
  std::string target_frame,
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer,
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster,
  rclcpp::Logger logger,
  rclcpp::Clock::SharedPtr clock)
: config_(std::move(config)),
  target_frame_(std::move(target_frame)),
  tf2_buffer_(std::move(tf2_buffer)),
  tf2_broadcaster_(std::move(tf2_broadcaster)),
  logger_(std::move(logger)),
  clock_(std::move(clock))
{
}

void GimbalPoseAdapter::onMicroPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr & msg)
{
  const double pitch = this->config_.pitch_sign * msg->pose.position.x;
  const double yaw = this->config_.yaw_sign * msg->pose.position.y;

  if (std::abs(pitch) < 1.6 && std::abs(yaw) < 1.6) {
    this->current_yaw_ = yaw;
    this->current_pitch_ = pitch;
    this->q_orientation_.setRPY(0.0, pitch, yaw);
  }
  this->last_self_orientation_time_ = this->clock_->now();
  this->broadcastCameraTF(msg->header.stamp);
}

void GimbalPoseAdapter::onCameraImu(const sensor_msgs::msg::Imu::ConstSharedPtr & msg)
{
  tf2::Quaternion q;
  tf2::fromMsg(msg->orientation, q);

  this->q_orientation_ = q;

  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  this->current_yaw_ = yaw;
  this->current_pitch_ = pitch;

  this->last_self_orientation_time_ = this->clock_->now();
  this->broadcastCameraTF(msg->header.stamp);
}

void GimbalPoseAdapter::updateForFrame(const rclcpp::Time & stamp)
{
  if (this->config_.pose_source != "none") {
    const double age = (stamp - this->last_self_orientation_time_).seconds();
    if (this->last_self_orientation_time_.nanoseconds() == 0 ||
      age > this->config_.self_orientation_timeout)
    {
      RCLCPP_WARN_THROTTLE(
        this->logger_, *this->clock_, 2000,
        "Gimbal feedback stale (%.2fs > %.2fs) -- TF using last known angles",
        age, this->config_.self_orientation_timeout);
    }
  }
  this->broadcastCameraTF(stamp);
}

GimbalPoseState GimbalPoseAdapter::currentPoseState(const rclcpp::Time & now) const
{
  GimbalPoseState state;
  state.yaw = this->current_yaw_;
  state.pitch = this->current_pitch_;
  state.orientation = tf2::toMsg(this->q_orientation_);
  state.last_update = this->last_self_orientation_time_;
  state.fresh = this->isFresh(now);
  return state;
}

bool GimbalPoseAdapter::isFresh(const rclcpp::Time & now) const
{
  if (this->config_.pose_source == "none") {
    return true;
  }
  if (this->last_self_orientation_time_.nanoseconds() == 0) {
    return false;
  }
  return (now - this->last_self_orientation_time_).seconds() <= this->config_.pose_timeout;
}

void GimbalPoseAdapter::broadcastCameraTF(const rclcpp::Time & stamp)
{
  tf2::Quaternion q_convention;
  q_convention.setRPY(-M_PI / 2.0, 0.0, -M_PI / 2.0);

  const tf2::Quaternion q_final = this->q_orientation_ * q_convention;
  const tf2::Vector3 pivot_position(0.0, 0.0, this->config_.gimbal_height);
  const tf2::Vector3 camera_offset(
    this->config_.camera_offset_x, 0.0, this->config_.camera_offset_z);
  const tf2::Vector3 translation =
    pivot_position + tf2::quatRotate(this->q_orientation_, camera_offset);

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = this->target_frame_;
  transform.child_frame_id = "camera_color_optical_frame";
  transform.transform.translation.x = translation.x();
  transform.transform.translation.y = translation.y();
  transform.transform.translation.z = translation.z();
  transform.transform.rotation = tf2::toMsg(q_final);

  this->tf2_buffer_->setTransform(transform, "auto_aim_targeting");
  this->tf2_broadcaster_->sendTransform(transform);
}

}  // namespace rm_auto_aim
