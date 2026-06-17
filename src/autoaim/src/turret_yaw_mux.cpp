// =============================================================================
// turret_yaw_mux.cpp
//
// Single authority for turret yaw/pitch/shoot.
//
// IMPORTANT for your current CV pipeline:
//   /cmd_vel_AI keeps publishing the last yaw/pitch even after losing target.
//   Therefore /cmd_vel_AI freshness is NOT used as "CV is detecting".
//   Detection is inferred from /detector/armors, type vision_msgs/Detection2DArray.
//
// Inputs:
//   /cmd_vel_AI              geometry_msgs::msg::Twist
//      angular.z = CV yaw absolute in micro frame [rad]
//      angular.y = CV pitch [rad]
//      angular.x = shoot flag
//
//   /detector/armors         vision_msgs::msg::Detection2DArray
//      detections.size() > 0 means CV/detector is currently seeing armor.
//      Empty or stale means idle/nav mode.
//
//   /micro_status            std_msgs::msg::Float32MultiArray
//      data[0] = current micro yaw
//      data[1] = current micro pitch
//
//   /turret/idle_target      geometry_msgs::msg::PointStamped
//      target point in map frame used when CV has no target
//
// TF:
//   lookup map -> base_link
//
// Output:
//   /turret/cmd              geometry_msgs::msg::Twist
//      angular.z = yaw absolute command for micro
//      angular.y = pitch command
//      angular.x = shoot command
//      linear.z  = mode: 0 idle/nav, 1 CV
//
// Runtime pipeline switches:
//   enable_cv_pipeline:   false -> ignore CV and use idle/hold
//   enable_idle_pipeline: false -> if no CV, hold current turret yaw/pitch
// =============================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{
double wrapAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

double yawFromQuat(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double limitAngleRate(double current, double target, double max_step)
{
  const double err = wrapAngle(target - current);
  const double step = std::clamp(err, -max_step, max_step);
  return wrapAngle(current + step);
}
}  // namespace

class TurretYawMux : public rclcpp::Node
{
public:
  TurretYawMux()
  : Node("turret_yaw_mux"),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
    tf_listener_(*tf_buffer_)
  {
    cv_cmd_topic_ = declare_parameter<std::string>("cv_cmd_topic", "/cmd_vel_AI");
    detection_topic_ = declare_parameter<std::string>("detection_topic", "/detector/armors");
    micro_status_topic_ = declare_parameter<std::string>("micro_status_topic", "/micro_status");
    idle_target_topic_ = declare_parameter<std::string>("idle_target_topic", "/turret/idle_target");
    turret_cmd_topic_ = declare_parameter<std::string>("turret_cmd_topic", "/turret/cmd");

    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    // Protects against autoaim node crash/no command. It is NOT the detection gate.
    cv_cmd_timeout_ = declare_parameter<double>("cv_cmd_timeout", 0.50);

    // Real CV mode gate: non-empty /detector/armors in this time window.
    detection_timeout_ = declare_parameter<double>("detection_timeout", 0.30);

    publish_rate_ = declare_parameter<double>("publish_rate", 50.0);
    tf_timeout_ = declare_parameter<double>("tf_timeout", 0.05);

    // Micro yaw convention:
    // yaw_cmd = yaw_sign * yaw_relative_to_chassis + yaw_zero_offset
    yaw_sign_ = declare_parameter<double>("yaw_sign", 1.0);
    yaw_zero_offset_ = declare_parameter<double>("yaw_zero_offset", 0.0);

    // 0.7 rad/s ≈ 40 deg/s; a 180 deg idle movement takes about 4.5 s.
    idle_yaw_rate_limit_ = declare_parameter<double>("idle_yaw_rate_limit", 0.7);

    idle_pitch_mode_ = declare_parameter<std::string>("idle_pitch_mode", "hold_last_micro");
    idle_pitch_static_ = declare_parameter<double>("idle_pitch_static", 0.0);

    micro_yaw_index_ = declare_parameter<int>("micro_yaw_index", 0);
    micro_pitch_index_ = declare_parameter<int>("micro_pitch_index", 1);

    use_default_idle_target_ = declare_parameter<bool>("use_default_idle_target", false);
    default_idle_target_x_ = declare_parameter<double>("default_idle_target_x", 0.0);
    default_idle_target_y_ = declare_parameter<double>("default_idle_target_y", 0.0);

    declare_parameter<bool>("enable_cv_pipeline", true);
    declare_parameter<bool>("enable_idle_pipeline", true);

    last_publish_time_ = monoNow();

    sub_cv_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
      cv_cmd_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&TurretYawMux::onCvCmd, this, std::placeholders::_1));

    sub_detection_ = create_subscription<vision_msgs::msg::Detection2DArray>(
      detection_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&TurretYawMux::onDetectionArray, this, std::placeholders::_1));

    sub_micro_status_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      micro_status_topic_, 10,
      std::bind(&TurretYawMux::onMicroStatus, this, std::placeholders::_1));

    sub_idle_target_ = create_subscription<geometry_msgs::msg::PointStamped>(
      idle_target_topic_, 10,
      std::bind(&TurretYawMux::onIdleTarget, this, std::placeholders::_1));

    pub_turret_cmd_ = create_publisher<geometry_msgs::msg::Twist>(turret_cmd_topic_, 10);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_)),
      std::bind(&TurretYawMux::publishCmd, this));

    RCLCPP_INFO(
      get_logger(),
      "turret_yaw_mux started: cv_cmd=%s detection=%s idle_target=%s output=%s map_frame=%s base_frame=%s",
      cv_cmd_topic_.c_str(),
      detection_topic_.c_str(),
      idle_target_topic_.c_str(),
      turret_cmd_topic_.c_str(),
      map_frame_.c_str(),
      base_frame_.c_str());
  }

private:
  double monoNow() const
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  bool getBoolParam(const std::string & name, bool fallback)
  {
    rclcpp::Parameter p;
    if (get_parameter(name, p) && p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      return p.as_bool();
    }
    return fallback;
  }

  void onCvCmd(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
    cv_yaw_ = msg->angular.z;
    cv_pitch_ = msg->angular.y;
    cv_shoot_ = msg->angular.x;
    last_cv_cmd_time_ = monoNow();
    have_cv_cmd_ = true;
  }

  void onDetectionArray(const vision_msgs::msg::Detection2DArray::ConstSharedPtr msg)
  {
    last_detection_msg_time_ = monoNow();

    if (!msg->detections.empty()) {
      last_nonempty_detection_time_ = last_detection_msg_time_;
      detector_has_target_ = true;
    } else {
      detector_has_target_ = false;
    }
  }

  void onMicroStatus(const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg)
  {
    const int max_idx = std::max(micro_yaw_index_, micro_pitch_index_);
    if (max_idx < 0 || static_cast<size_t>(max_idx) >= msg->data.size()) {
      return;
    }

    micro_yaw_ = static_cast<double>(msg->data[micro_yaw_index_]);
    micro_pitch_ = static_cast<double>(msg->data[micro_pitch_index_]);
    have_micro_status_ = true;

    if (!have_output_yaw_) {
      output_yaw_ = micro_yaw_;
      have_output_yaw_ = true;
    }
  }

  void onIdleTarget(const geometry_msgs::msg::PointStamped::ConstSharedPtr msg)
  {
    geometry_msgs::msg::PointStamped target_map;

    try {
      if (msg->header.frame_id.empty() || msg->header.frame_id == map_frame_) {
        target_map = *msg;
        target_map.header.frame_id = map_frame_;
      } else {
        target_map = tf_buffer_->transform(
          *msg, map_frame_, tf2::durationFromSec(tf_timeout_));
      }

      idle_target_x_ = target_map.point.x;
      idle_target_y_ = target_map.point.y;
      have_idle_target_ = true;

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Idle turret target: frame=%s x=%.3f y=%.3f",
        map_frame_.c_str(), idle_target_x_, idle_target_y_);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Cannot transform idle target to %s: %s", map_frame_.c_str(), e.what());
    }
  }

  bool computeIdleYaw(double & yaw_cmd)
  {
    double target_x = idle_target_x_;
    double target_y = idle_target_y_;

    if (!have_idle_target_) {
      if (!use_default_idle_target_) {
        return false;
      }
      target_x = default_idle_target_x_;
      target_y = default_idle_target_y_;
    }

    geometry_msgs::msg::TransformStamped tf;

    try {
      tf = tf_buffer_->lookupTransform(
        map_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(tf_timeout_));
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "No TF %s -> %s for idle turret yaw: %s",
        map_frame_.c_str(), base_frame_.c_str(), e.what());
      return false;
    }

    const double robot_x = tf.transform.translation.x;
    const double robot_y = tf.transform.translation.y;
    const double robot_yaw = yawFromQuat(tf.transform.rotation);

    const double target_angle_map = std::atan2(target_y - robot_y, target_x - robot_x);
    const double yaw_relative_to_chassis = wrapAngle(target_angle_map - robot_yaw);

    yaw_cmd = wrapAngle(yaw_sign_ * yaw_relative_to_chassis + yaw_zero_offset_);
    return true;
  }

  bool cvHasRecentTarget(double now) const
  {
    return detector_has_target_ &&
      ((now - last_nonempty_detection_time_) < detection_timeout_);
  }

  bool cvCommandFresh(double now) const
  {
    return have_cv_cmd_ &&
      ((now - last_cv_cmd_time_) < cv_cmd_timeout_);
  }

  void publishCmd()
  {
    const double now = monoNow();
    const double dt = std::max(0.001, now - last_publish_time_);
    last_publish_time_ = now;

    const bool enable_cv = getBoolParam("enable_cv_pipeline", true);
    const bool enable_idle = getBoolParam("enable_idle_pipeline", true);

    const bool use_cv = enable_cv && cvHasRecentTarget(now) && cvCommandFresh(now);

    geometry_msgs::msg::Twist out;

    if (use_cv) {
      out.angular.z = wrapAngle(cv_yaw_);
      out.angular.y = cv_pitch_;
      out.angular.x = cv_shoot_;
      out.linear.z = 1.0;  // mode = CV

      output_yaw_ = out.angular.z;
      have_output_yaw_ = true;

      pub_turret_cmd_->publish(out);
      return;
    }

    if (!have_output_yaw_) {
      output_yaw_ = have_micro_status_ ? micro_yaw_ : 0.0;
      have_output_yaw_ = true;
    }

    if (!enable_idle) {
      if (have_micro_status_) {
        output_yaw_ = micro_yaw_;
        out.angular.y = micro_pitch_;
      } else {
        out.angular.y = idle_pitch_static_;
      }

      out.angular.z = wrapAngle(output_yaw_);
      out.angular.x = 0.0;
      out.linear.z = 0.0;

      pub_turret_cmd_->publish(out);
      return;
    }

    double desired_yaw = 0.0;
    const bool have_yaw_target = computeIdleYaw(desired_yaw);

    if (have_yaw_target) {
      output_yaw_ = limitAngleRate(output_yaw_, desired_yaw, idle_yaw_rate_limit_ * dt);
    } else if (have_micro_status_) {
      output_yaw_ = micro_yaw_;
    }

    out.angular.z = wrapAngle(output_yaw_);

    if (idle_pitch_mode_ == "hold_last_micro" && have_micro_status_) {
      out.angular.y = micro_pitch_;
    } else {
      out.angular.y = idle_pitch_static_;
    }

    out.angular.x = 0.0;  // no shooting in idle/nav mode
    out.linear.z = 0.0;   // mode = idle/nav

    pub_turret_cmd_->publish(out);
  }

  std::string cv_cmd_topic_;
  std::string detection_topic_;
  std::string micro_status_topic_;
  std::string idle_target_topic_;
  std::string turret_cmd_topic_;
  std::string map_frame_;
  std::string base_frame_;
  std::string idle_pitch_mode_;

  double cv_cmd_timeout_ = 0.50;
  double detection_timeout_ = 0.30;
  double publish_rate_ = 50.0;
  double tf_timeout_ = 0.05;
  double yaw_sign_ = 1.0;
  double yaw_zero_offset_ = 0.0;
  double idle_yaw_rate_limit_ = 0.7;
  double idle_pitch_static_ = 0.0;

  int micro_yaw_index_ = 0;
  int micro_pitch_index_ = 1;

  bool use_default_idle_target_ = false;
  double default_idle_target_x_ = 0.0;
  double default_idle_target_y_ = 0.0;

  double cv_yaw_ = 0.0;
  double cv_pitch_ = 0.0;
  double cv_shoot_ = 0.0;
  double last_cv_cmd_time_ = 0.0;
  bool have_cv_cmd_ = false;

  double last_detection_msg_time_ = 0.0;
  double last_nonempty_detection_time_ = -1e9;
  bool detector_has_target_ = false;

  double micro_yaw_ = 0.0;
  double micro_pitch_ = 0.0;
  bool have_micro_status_ = false;

  double idle_target_x_ = 0.0;
  double idle_target_y_ = 0.0;
  bool have_idle_target_ = false;

  double output_yaw_ = 0.0;
  bool have_output_yaw_ = false;
  double last_publish_time_ = 0.0;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cv_cmd_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr sub_detection_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_micro_status_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_idle_target_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_turret_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurretYawMux>());
  rclcpp::shutdown();
  return 0;
}
