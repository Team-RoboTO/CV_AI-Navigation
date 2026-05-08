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

namespace
{

bool quaternionComponentsFinite(const geometry_msgs::msg::Quaternion & q)
{
  return std::isfinite(q.x) && std::isfinite(q.y) &&
         std::isfinite(q.z) && std::isfinite(q.w);
}

}  // namespace

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

  const rclcpp::Time stamp =
    msg->header.stamp.nanosec == 0 && msg->header.stamp.sec == 0 ?
    this->clock_->now() : rclcpp::Time(msg->header.stamp);
  if (!this->updateStoredPose(pitch, yaw, stamp)) {
    return;
  }
  this->broadcastCameraTF(stamp);
}

void GimbalPoseAdapter::onMicroImu(const std_msgs::msg::Float32MultiArray::ConstSharedPtr & msg)
{
  if (msg->data.size() < 2) {
    RCLCPP_DEBUG(this->logger_, "Rejecting /micro_imu sample with fewer than 2 values");
    return;
  }

  const double yaw = this->config_.yaw_sign * static_cast<double>(msg->data[0]);
  const double pitch = this->config_.pitch_sign * static_cast<double>(msg->data[1]);
  const rclcpp::Time stamp = this->clock_->now();
  if (!this->updateStoredPose(pitch, yaw, stamp)) {
    return;
  }
  this->broadcastCameraTF(stamp);
}

void GimbalPoseAdapter::onGimbalState(
  const auto_aim_interfaces::msg::GimbalState::ConstSharedPtr & msg)
{
  if (!msg->valid) {
    RCLCPP_DEBUG(this->logger_, "Ignoring invalid GimbalState sample");
    return;
  }

  const double yaw = this->config_.yaw_sign * msg->yaw;
  const double pitch = this->config_.pitch_sign * msg->pitch;
  const rclcpp::Time stamp =
    msg->header.stamp.nanosec == 0 && msg->header.stamp.sec == 0 ?
    this->clock_->now() : rclcpp::Time(msg->header.stamp);
  if (!this->updateStoredPose(pitch, yaw, stamp)) {
    return;
  }
  this->broadcastCameraTF(stamp);
}

void GimbalPoseAdapter::onCameraImu(const sensor_msgs::msg::Imu::ConstSharedPtr & msg)
{
  if (!quaternionComponentsFinite(msg->orientation)) {
    RCLCPP_DEBUG(this->logger_, "Rejecting IMU pose sample with non-finite quaternion");
    return;
  }

  tf2::Quaternion q;
  tf2::fromMsg(msg->orientation, q);
  const double q_norm = q.length();
  if (!std::isfinite(q_norm) || q_norm < 0.5 || q_norm > 2.0) {
    RCLCPP_DEBUG(
      this->logger_, "Rejecting IMU pose sample with invalid quaternion norm %.6f", q_norm);
    return;
  }
  q.normalize();

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  pitch *= this->config_.pitch_sign;
  yaw *= this->config_.yaw_sign;

  {
    std::lock_guard<std::mutex> lock(this->pose_mutex_);
    this->current_yaw_ = yaw;
    this->current_pitch_ = pitch;
    this->q_orientation_.setRPY(roll, pitch, yaw);
    this->last_self_orientation_time_ = this->clock_->now();
  }
  this->broadcastCameraTF(msg->header.stamp);
}

void GimbalPoseAdapter::updateForFrame(const rclcpp::Time & stamp)
{
  double age = 0.0;
  bool stale = false;
  if (this->config_.pose_source != "none") {
    {
      std::lock_guard<std::mutex> lock(this->pose_mutex_);
      if (this->last_self_orientation_time_.nanoseconds() == 0) {
        stale = true;
      } else {
        age = (this->clock_->now() - this->last_self_orientation_time_).seconds();
        stale = age > this->config_.self_orientation_timeout;
      }
    }
    if (stale) {
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
  std::lock_guard<std::mutex> lock(this->pose_mutex_);
  state.yaw = this->current_yaw_;
  state.pitch = this->current_pitch_;
  state.orientation = tf2::toMsg(this->q_orientation_);
  state.last_update = this->last_self_orientation_time_;
  state.fresh = this->isFreshLocked(now);
  return state;
}

bool GimbalPoseAdapter::isFresh(const rclcpp::Time & now) const
{
  std::lock_guard<std::mutex> lock(this->pose_mutex_);
  return this->isFreshLocked(now);
}

bool GimbalPoseAdapter::isFreshLocked(const rclcpp::Time & now) const
{
  if (this->config_.pose_source == "none") {
    return true;
  }
  if (this->last_self_orientation_time_.nanoseconds() == 0) {
    return false;
  }
  return (now - this->last_self_orientation_time_).seconds() <= this->config_.pose_timeout;
}

bool GimbalPoseAdapter::updateStoredPose(double pitch, double yaw, const rclcpp::Time & stamp)
{
  constexpr double kPitchSanityLimit = M_PI / 2.0 + 0.2;
  if (std::abs(pitch) >= kPitchSanityLimit || !std::isfinite(pitch) || !std::isfinite(yaw)) {
    RCLCPP_DEBUG(
      this->logger_, "Rejecting invalid gimbal pose sample: pitch=%.3f yaw=%.3f",
      pitch, yaw);
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(this->pose_mutex_);
    this->current_yaw_ = yaw;
    this->current_pitch_ = pitch;
    this->q_orientation_.setRPY(0.0, pitch, yaw);
    this->last_self_orientation_time_ = stamp;
  }
  return true;
}

void GimbalPoseAdapter::broadcastCameraTF(const rclcpp::Time & stamp)
{
  tf2::Quaternion q_orientation;
  {
    std::lock_guard<std::mutex> lock(this->pose_mutex_);
    q_orientation = this->q_orientation_;
  }

  tf2::Quaternion q_convention;
  q_convention.setRPY(-M_PI / 2.0, 0.0, -M_PI / 2.0);

  const tf2::Quaternion q_final = q_orientation * q_convention;
  const tf2::Vector3 pivot_position(0.0, 0.0, this->config_.gimbal_height);
  const tf2::Vector3 camera_offset(
    this->config_.camera_offset_x, 0.0, this->config_.camera_offset_z);
  const tf2::Vector3 translation =
    pivot_position + tf2::quatRotate(q_orientation, camera_offset);

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
