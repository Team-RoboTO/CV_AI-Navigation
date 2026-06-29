#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// Runtime ROS log gate. Launch files default enable_ros_logs=false in
// competition; these wrappers avoid entering RCLCPP_* macros at all when the
// flag is off, while keeping the original log call sites maintainable.
#define AA_RCLCPP_LOG_IF(enabled, macro_name, ...) \
  do { \
    if (enabled) { \
      macro_name(__VA_ARGS__); \
    } \
  } while (0)
#define AA_RCLCPP_DEBUG(enabled, ...) AA_RCLCPP_LOG_IF(enabled, RCLCPP_DEBUG, __VA_ARGS__)
#define AA_RCLCPP_INFO(enabled, ...) AA_RCLCPP_LOG_IF(enabled, RCLCPP_INFO, __VA_ARGS__)
#define AA_RCLCPP_WARN(enabled, ...) AA_RCLCPP_LOG_IF(enabled, RCLCPP_WARN, __VA_ARGS__)
#define AA_RCLCPP_ERROR(enabled, ...) AA_RCLCPP_LOG_IF(enabled, RCLCPP_ERROR, __VA_ARGS__)
#define AA_RCLCPP_FATAL(enabled, ...) AA_RCLCPP_LOG_IF(enabled, RCLCPP_FATAL, __VA_ARGS__)
#define AA_RCLCPP_INFO_THROTTLE(enabled, ...) AA_RCLCPP_LOG_IF(enabled, RCLCPP_INFO_THROTTLE, __VA_ARGS__)
#define AA_RCLCPP_WARN_THROTTLE(enabled, ...) AA_RCLCPP_LOG_IF(enabled, RCLCPP_WARN_THROTTLE, __VA_ARGS__)

class TurretYawMux : public rclcpp::Node
{
public:
  TurretYawMux()
  : Node("turret_yaw_mux")
  {
    cv_cmd_topic_ = declare_parameter<std::string>("cv_cmd_topic", "/cmd_vel_AI");
    detection_topic_ = declare_parameter<std::string>("detection_topic", "/detector/armors");
    micro_status_topic_ = declare_parameter<std::string>("micro_status_topic", "/micro_status");
    output_topic_ = declare_parameter<std::string>("output_topic", "/turret/cmd");
    // Runtime log gate for the sentry mux. False skips AA_RCLCPP_* wrappers
    // entirely and does not affect /turret/cmd arbitration.
    enable_ros_logs_ = declare_parameter<bool>("enable_ros_logs", false);

    idle_target_topic_ = declare_parameter<std::string>("idle_target_topic", "/turret/idle_target");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 50.0);

    detection_timeout_sec_ = declare_parameter<double>("detection_timeout", 0.50);
    cv_cmd_timeout_sec_ = declare_parameter<double>("cv_cmd_timeout", 0.50);
    min_detection_score_ = declare_parameter<double>("min_detection_score", 0.12);
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout", 0.05);

    valid_class_ids_ = declare_parameter<std::vector<std::string>>(
      "valid_class_ids",
      std::vector<std::string>{"0", "2"});

    target_classes_from_micro_status_ = declare_parameter<bool>(
      "target_classes_from_micro_status", true);
    target_color_status_index_ = declare_parameter<int>("target_color_status_index", 4);
    micro_color_target_classes_ = declare_parameter<std::vector<std::string>>(
      "micro_color_target_classes",
      std::vector<std::string>{"0", "2"});

    use_idle_target_when_no_detection_ = declare_parameter<bool>(
      "use_idle_target_when_no_detection", true);
    freeze_when_no_detection_ = declare_parameter<bool>(
      "freeze_when_no_detection", true);

    yaw_sign_ = declare_parameter<double>("yaw_sign", 1.0);
    yaw_zero_offset_ = declare_parameter<double>("yaw_zero_offset", 0.0);

    idle_yaw_rate_limit_ = declare_parameter<double>("idle_yaw_rate_limit", 1.50);
    idle_recompute_period_sec_ = declare_parameter<double>("idle_recompute_period", 0.10);
    idle_yaw_deadband_ = declare_parameter<double>("idle_yaw_deadband", 0.03);

    idle_pitch_mode_ = declare_parameter<std::string>("idle_pitch_mode", "hold_last_micro");
    idle_pitch_static_ = declare_parameter<double>("idle_pitch_static", -0.115);
    idle_pitch_sign_ = declare_parameter<double>("idle_pitch_sign", 1.0);
    idle_pitch_zero_offset_ = declare_parameter<double>("idle_pitch_zero_offset", -0.115);
    idle_target_z_ = declare_parameter<double>("idle_target_z", 0.42);
    use_idle_point_z_ = declare_parameter<bool>("use_idle_point_z", false);
    gimbal_height_ = declare_parameter<double>("gimbal_height", 0.42);

    use_default_idle_target_ = declare_parameter<bool>("use_default_idle_target", false);
    default_idle_target_x_ = declare_parameter<double>("default_idle_target_x", 6.0);
    default_idle_target_y_ = declare_parameter<double>("default_idle_target_y", 4.0);
    default_idle_target_z_ = declare_parameter<double>("default_idle_target_z", 0.42);

    target_reset_distance_ = declare_parameter<double>("target_reset_distance", 0.50);

    valid_class_set_ = makeClassSet(valid_class_ids_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cv_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cv_cmd_topic_,
      10,
      std::bind(&TurretYawMux::onCvCmd, this, std::placeholders::_1));

    detection_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
      detection_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&TurretYawMux::onDetections, this, std::placeholders::_1));

    micro_status_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      micro_status_topic_,
      10,
      std::bind(&TurretYawMux::onMicroStatus, this, std::placeholders::_1));

    idle_target_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      idle_target_topic_,
      10,
      std::bind(&TurretYawMux::onIdleTarget, this, std::placeholders::_1));

    turret_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      output_topic_,
      10);

    const double safe_rate = std::max(1.0, publish_rate_hz_);
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / safe_rate));

    timer_ = create_wall_timer(
      period,
      std::bind(&TurretYawMux::onTimer, this));

    parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&TurretYawMux::onSetParameters, this, std::placeholders::_1));

    AA_RCLCPP_INFO(enable_ros_logs_, get_logger(), "turret_yaw_mux started");
    AA_RCLCPP_INFO(enable_ros_logs_, get_logger(), "  cv_cmd_topic: %s", cv_cmd_topic_.c_str());
    AA_RCLCPP_INFO(enable_ros_logs_, get_logger(), "  detection_topic: %s", detection_topic_.c_str());
    AA_RCLCPP_INFO(enable_ros_logs_, get_logger(), "  micro_status_topic: %s", micro_status_topic_.c_str());
    AA_RCLCPP_INFO(enable_ros_logs_, get_logger(), "  idle_target_topic: %s", idle_target_topic_.c_str());
    AA_RCLCPP_INFO(enable_ros_logs_, get_logger(), "  output_topic: %s", output_topic_.c_str());
    AA_RCLCPP_INFO(enable_ros_logs_, get_logger(), "  map_frame: %s", map_frame_.c_str());
    AA_RCLCPP_INFO(enable_ros_logs_, get_logger(), "  base_frame: %s", base_frame_.c_str());
    AA_RCLCPP_INFO(enable_ros_logs_, get_logger(), "  detection_timeout: %.3f", detection_timeout_sec_);
    AA_RCLCPP_INFO(enable_ros_logs_, get_logger(), "  cv_cmd_timeout: %.3f", cv_cmd_timeout_sec_);
    AA_RCLCPP_INFO(enable_ros_logs_, get_logger(), "  min_detection_score: %.3f", min_detection_score_);
    AA_RCLCPP_INFO(enable_ros_logs_,
      get_logger(),
      "  target_classes_from_micro_status: %s",
      target_classes_from_micro_status_ ? "true" : "false");
    AA_RCLCPP_INFO(enable_ros_logs_, get_logger(), "  target_color_status_index: %d", target_color_status_index_);
    AA_RCLCPP_INFO(enable_ros_logs_,
      get_logger(),
      "  use_idle_target_when_no_detection: %s",
      use_idle_target_when_no_detection_ ? "true" : "false");
    AA_RCLCPP_INFO(enable_ros_logs_,
      get_logger(),
      "  freeze_when_no_detection fallback: %s",
      freeze_when_no_detection_ ? "true" : "false");
    AA_RCLCPP_INFO(enable_ros_logs_, get_logger(), "  yaw_sign: %.3f", yaw_sign_);
    AA_RCLCPP_INFO(enable_ros_logs_, get_logger(), "  yaw_zero_offset: %.3f", yaw_zero_offset_);
    AA_RCLCPP_INFO(enable_ros_logs_, get_logger(), "  idle_yaw_rate_limit: %.3f", idle_yaw_rate_limit_);
    AA_RCLCPP_INFO(enable_ros_logs_, get_logger(), "  idle_recompute_period: %.3f", idle_recompute_period_sec_);
    AA_RCLCPP_INFO(enable_ros_logs_, get_logger(), "  idle_yaw_deadband: %.3f", idle_yaw_deadband_);
    AA_RCLCPP_INFO(enable_ros_logs_, get_logger(), "  target_reset_distance: %.3f", target_reset_distance_);
    logValidClasses();
  }

private:
  static std::unordered_set<std::string> makeClassSet(
    const std::vector<std::string> & ids)
  {
    std::unordered_set<std::string> out;

    for (const auto & id : ids) {
      out.insert(id);
    }

    return out;
  }

  static double clampAbs(double v, double limit)
  {
    if (limit <= 0.0) {
      return v;
    }

    if (v > limit) {
      return limit;
    }

    if (v < -limit) {
      return -limit;
    }

    return v;
  }

  static double sqr(double x)
  {
    return x * x;
  }

  void logValidClasses()
  {
    std::string classes;

    for (size_t i = 0; i < valid_class_ids_.size(); ++i) {
      classes += valid_class_ids_[i];

      if (i + 1 < valid_class_ids_.size()) {
        classes += ", ";
      }
    }

    AA_RCLCPP_INFO(enable_ros_logs_, get_logger(), "  valid_class_ids: [%s]", classes.c_str());
  }

  void resetIdleUnwrap()
  {
    have_last_desired_internal_yaw_ = false;
    last_desired_internal_yaw_unwrapped_ = 0.0;
    have_idle_command_ = false;
  }

  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : params) {
      const auto & name = param.get_name();

      if (name == "detection_timeout") {
        const double v = param.as_double();

        if (v <= 0.0) {
          result.successful = false;
          result.reason = "detection_timeout must be > 0";
          return result;
        }

        detection_timeout_sec_ = v;
      }
      else if (name == "cv_cmd_timeout") {
        const double v = param.as_double();

        if (v <= 0.0) {
          result.successful = false;
          result.reason = "cv_cmd_timeout must be > 0";
          return result;
        }

        cv_cmd_timeout_sec_ = v;
      }
      else if (name == "min_detection_score") {
        const double v = param.as_double();

        if (v < 0.0 || v > 1.0) {
          result.successful = false;
          result.reason = "min_detection_score must be in [0, 1]";
          return result;
        }

        min_detection_score_ = v;
      }
      else if (name == "tf_timeout") {
        const double v = param.as_double();

        if (v <= 0.0) {
          result.successful = false;
          result.reason = "tf_timeout must be > 0";
          return result;
        }

        tf_timeout_sec_ = v;
      }
      else if (name == "valid_class_ids") {
        valid_class_ids_ = param.as_string_array();
        valid_class_set_ = makeClassSet(valid_class_ids_);
        logValidClasses();
      }
      else if (name == "target_classes_from_micro_status") {
        target_classes_from_micro_status_ = param.as_bool();
      }
      else if (name == "target_color_status_index") {
        target_color_status_index_ = param.as_int();
      }
      else if (name == "micro_color_target_classes") {
        micro_color_target_classes_ = param.as_string_array();
      }
      else if (name == "use_idle_target_when_no_detection") {
        use_idle_target_when_no_detection_ = param.as_bool();
        resetIdleUnwrap();
      }
      else if (name == "freeze_when_no_detection") {
        freeze_when_no_detection_ = param.as_bool();
      }
      else if (name == "yaw_sign") {
        yaw_sign_ = param.as_double();
        resetIdleUnwrap();
      }
      else if (name == "yaw_zero_offset") {
        yaw_zero_offset_ = param.as_double();
        resetIdleUnwrap();
      }
      else if (name == "idle_yaw_rate_limit") {
        idle_yaw_rate_limit_ = param.as_double();
      }
      else if (name == "idle_recompute_period") {
        const double v = param.as_double();

        if (v <= 0.0) {
          result.successful = false;
          result.reason = "idle_recompute_period must be > 0";
          return result;
        }

        idle_recompute_period_sec_ = v;
      }
      else if (name == "idle_yaw_deadband") {
        const double v = param.as_double();

        if (v < 0.0) {
          result.successful = false;
          result.reason = "idle_yaw_deadband must be >= 0";
          return result;
        }

        idle_yaw_deadband_ = v;
      }
      else if (name == "idle_pitch_mode") {
        idle_pitch_mode_ = param.as_string();
        have_idle_command_ = false;
      }
      else if (name == "idle_pitch_static") {
        idle_pitch_static_ = param.as_double();
        have_idle_command_ = false;
      }
      else if (name == "idle_pitch_sign") {
        idle_pitch_sign_ = param.as_double();
        have_idle_command_ = false;
      }
      else if (name == "idle_pitch_zero_offset") {
        idle_pitch_zero_offset_ = param.as_double();
        have_idle_command_ = false;
      }
      else if (name == "idle_target_z") {
        idle_target_z_ = param.as_double();
        have_idle_command_ = false;
      }
      else if (name == "use_idle_point_z") {
        use_idle_point_z_ = param.as_bool();
        have_idle_command_ = false;
      }
      else if (name == "gimbal_height") {
        gimbal_height_ = param.as_double();
        have_idle_command_ = false;
      }
      else if (name == "use_default_idle_target") {
        use_default_idle_target_ = param.as_bool();
        resetIdleUnwrap();
      }
      else if (name == "default_idle_target_x") {
        default_idle_target_x_ = param.as_double();
        resetIdleUnwrap();
      }
      else if (name == "default_idle_target_y") {
        default_idle_target_y_ = param.as_double();
        resetIdleUnwrap();
      }
      else if (name == "default_idle_target_z") {
        default_idle_target_z_ = param.as_double();
        resetIdleUnwrap();
      }
      else if (name == "target_reset_distance") {
        const double v = param.as_double();

        if (v < 0.0) {
          result.successful = false;
          result.reason = "target_reset_distance must be >= 0";
          return result;
        }

        target_reset_distance_ = v;
      }
    }

    return result;
  }

  void onCvCmd(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cv_cmd_ = *msg;
    last_cv_cmd_time_ = now();
    have_cv_cmd_ = true;
  }

  void onMicroStatus(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 2) {
      return;
    }

    micro_yaw_ = static_cast<double>(msg->data[0]);
    micro_pitch_ = static_cast<double>(msg->data[1]);
    have_micro_status_ = true;

    updateValidClassesFromMicroStatus(*msg);
  }

  void updateValidClassesFromMicroStatus(const std_msgs::msg::Float32MultiArray & msg)
  {
    if (!target_classes_from_micro_status_) {
      return;
    }

    if (target_color_status_index_ < 0 ||
        static_cast<size_t>(target_color_status_index_) >= msg.data.size())
    {
      AA_RCLCPP_WARN_THROTTLE(enable_ros_logs_,
        get_logger(), *get_clock(), 2000,
        "/micro_status[%d] is not available; keeping current valid_class_ids",
        target_color_status_index_);
      return;
    }

    const double raw_color =
      static_cast<double>(msg.data[static_cast<size_t>(target_color_status_index_)]);

    if (!std::isfinite(raw_color)) {
      return;
    }

    const int micro_color = static_cast<int>(std::lround(raw_color));

    if (std::abs(raw_color - static_cast<double>(micro_color)) > 0.25) {
      AA_RCLCPP_WARN_THROTTLE(enable_ros_logs_,
        get_logger(), *get_clock(), 2000,
        "/micro_status[%d] color should be near 0 or 1, got %.3f",
        target_color_status_index_, raw_color);
      return;
    }

    if (micro_color < 0 ||
        static_cast<size_t>(micro_color) >= micro_color_target_classes_.size())
    {
      AA_RCLCPP_WARN_THROTTLE(enable_ros_logs_,
        get_logger(), *get_clock(), 2000,
        "/micro_status[%d]=%d has no mapping in micro_color_target_classes",
        target_color_status_index_, micro_color);
      return;
    }

    const std::string target_class =
      micro_color_target_classes_[static_cast<size_t>(micro_color)];

    if (target_class.empty() || target_class == "1") {
      AA_RCLCPP_WARN_THROTTLE(enable_ros_logs_,
        get_logger(), *get_clock(), 2000,
        "Invalid target class from micro color mapping: '%s'",
        target_class.c_str());
      return;
    }

    if (valid_class_ids_.size() == 1 &&
        valid_class_ids_[0] == target_class &&
        have_micro_target_color_ &&
        last_micro_target_color_ == micro_color)
    {
      return;
    }

    valid_class_ids_ = std::vector<std::string>{target_class};
    valid_class_set_ = makeClassSet(valid_class_ids_);
    have_micro_target_color_ = true;
    last_micro_target_color_ = micro_color;

    have_valid_detection_ = false;
    last_valid_detection_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    AA_RCLCPP_INFO(enable_ros_logs_,
      get_logger(),
      "Target class from /micro_status[%d]=%d -> detector class '%s'",
      target_color_status_index_,
      micro_color,
      target_class.c_str());
  }

  void onIdleTarget(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    const bool first_target = !have_idle_target_;

    bool target_changed_significantly = first_target;

    if (!first_target) {
      const std::string old_frame =
        idle_target_.header.frame_id.empty() ? map_frame_ : idle_target_.header.frame_id;
      const std::string new_frame =
        msg->header.frame_id.empty() ? map_frame_ : msg->header.frame_id;

      const double dist = std::sqrt(
        sqr(msg->point.x - idle_target_.point.x) +
        sqr(msg->point.y - idle_target_.point.y) +
        sqr(msg->point.z - idle_target_.point.z));

      if (old_frame != new_frame || dist > target_reset_distance_) {
        target_changed_significantly = true;
      }
    }

    idle_target_ = *msg;
    have_idle_target_ = true;

    // Non resettare l'unwrap a ogni messaggio periodico uguale.
    // Resetta solo se il target cambia davvero.
    if (target_changed_significantly) {
      resetIdleUnwrap();

      AA_RCLCPP_INFO(enable_ros_logs_,
        get_logger(),
        "Idle target changed/reset unwrap: frame=%s target=(%.2f, %.2f, %.2f)",
        idle_target_.header.frame_id.c_str(),
        idle_target_.point.x,
        idle_target_.point.y,
        idle_target_.point.z);
    }
  }

  void onDetections(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
  {
    bool valid_detection = false;
    std::string best_class;
    double best_score = -1.0;

    for (const auto & det : msg->detections) {
      for (const auto & res : det.results) {
        const std::string & class_id = res.hypothesis.class_id;
        const double score = static_cast<double>(res.hypothesis.score);

        if (valid_class_set_.count(class_id) > 0 && score >= min_detection_score_) {
          valid_detection = true;

          if (score > best_score) {
            best_score = score;
            best_class = class_id;
          }
        }
      }
    }

    if (valid_detection) {
      last_valid_detection_time_ = now();
      have_valid_detection_ = true;

      AA_RCLCPP_DEBUG(enable_ros_logs_,
        get_logger(),
        "Valid detection: class_id=%s score=%.3f",
        best_class.c_str(),
        best_score);
    }
  }

  bool isFresh(const rclcpp::Time & stamp, double timeout_sec) const
  {
    const double age = (now() - stamp).seconds();
    return age >= 0.0 && age <= timeout_sec;
  }

  bool computeIdleCommand(geometry_msgs::msg::Twist & out, const rclcpp::Time & tnow)
  {
    geometry_msgs::msg::PointStamped target;

    if (have_idle_target_) {
      target = idle_target_;
    } else if (use_default_idle_target_) {
      target.header.frame_id = map_frame_;
      target.header.stamp.sec = 0;
      target.header.stamp.nanosec = 0;
      target.point.x = default_idle_target_x_;
      target.point.y = default_idle_target_y_;
      target.point.z = default_idle_target_z_;
    } else {
      return false;
    }

    if (!have_micro_status_) {
      AA_RCLCPP_WARN_THROTTLE(enable_ros_logs_,
        get_logger(), *get_clock(), 1000,
        "Cannot compute idle turret target: no /micro_status yaw feedback yet");
      return false;
    }

    const std::string source_frame =
      target.header.frame_id.empty() ? map_frame_ : target.header.frame_id;

    target.header.frame_id = source_frame;
    target.header.stamp.sec = 0;
    target.header.stamp.nanosec = 0;

    const double idle_age = (tnow - last_idle_recompute_time_).seconds();

    if (have_idle_command_ &&
        idle_age >= 0.0 &&
        idle_age < idle_recompute_period_sec_)
    {
      out.angular.z = idle_yaw_cmd_;
      out.angular.y = idle_pitch_cmd_;
      out.angular.x = 0.0;
      out.linear.z = 0.0;
      return true;
    }

    geometry_msgs::msg::PointStamped target_in_base;

    try {
      target_in_base = tf_buffer_->transform(
        target,
        base_frame_,
        tf2::durationFromSec(tf_timeout_sec_));
    } catch (const tf2::TransformException & ex) {
      AA_RCLCPP_WARN_THROTTLE(enable_ros_logs_,
        get_logger(), *get_clock(), 1000,
        "Cannot compute idle turret target: transform target %s -> %s unavailable: %s",
        source_frame.c_str(),
        base_frame_.c_str(),
        ex.what());
      return false;
    }

    const double dx_base = target_in_base.point.x;
    const double dy_base = target_in_base.point.y;
    const double dz_base = target_in_base.point.z;

    const double horizontal_dist = std::hypot(dx_base, dy_base);

    if (horizontal_dist < 0.05) {
      AA_RCLCPP_WARN_THROTTLE(enable_ros_logs_,
        get_logger(), *get_clock(), 1000,
        "Cannot compute idle turret target: target too close to robot");
      return false;
    }

    const double desired_yaw_base = std::atan2(dy_base, dx_base);

    const double pi = std::acos(-1.0);
    const double two_pi = 2.0 * pi;

    const double current_internal_yaw_unwrapped =
      yaw_sign_ * micro_yaw_ + yaw_zero_offset_;

    // Fix anti-salto ±2π:
    // scegli il ramo del desired_yaw_base rispetto al desired precedente,
    // non rispetto al micro_yaw corrente.
    const double unwrap_reference =
      have_last_desired_internal_yaw_
        ? last_desired_internal_yaw_unwrapped_
        : current_internal_yaw_unwrapped;

    const double k = std::round(
      (unwrap_reference - desired_yaw_base) / two_pi);

    const double desired_internal_yaw_unwrapped =
      desired_yaw_base + k * two_pi;

    have_last_desired_internal_yaw_ = true;
    last_desired_internal_yaw_unwrapped_ = desired_internal_yaw_unwrapped;

    double delta_internal =
      desired_internal_yaw_unwrapped - current_internal_yaw_unwrapped;

    if (std::abs(delta_internal) < idle_yaw_deadband_) {
      delta_internal = 0.0;
    }

    if (idle_yaw_rate_limit_ > 0.0) {
      const double max_step = idle_yaw_rate_limit_ * idle_recompute_period_sec_;
      delta_internal = clampAbs(delta_internal, max_step);
    }

    idle_yaw_cmd_ = micro_yaw_ + yaw_sign_ * delta_internal;

    if (idle_pitch_mode_ == "static") {
      idle_pitch_cmd_ = idle_pitch_static_;
    } else if (idle_pitch_mode_ == "target") {
      const double target_z_base =
        use_idle_point_z_ ? dz_base : (idle_target_z_ - gimbal_height_);

      idle_pitch_cmd_ =
        idle_pitch_sign_ * std::atan2(target_z_base, horizontal_dist) +
        idle_pitch_zero_offset_;
    } else if (idle_pitch_mode_ == "hold_last_micro" && have_micro_status_) {
      idle_pitch_cmd_ = micro_pitch_;
    } else {
      idle_pitch_cmd_ = last_output_pitch_;
    }

    last_idle_recompute_time_ = tnow;
    have_idle_command_ = true;

    out.angular.z = idle_yaw_cmd_;
    out.angular.y = idle_pitch_cmd_;
    out.angular.x = 0.0;
    out.linear.z = 0.0;

    AA_RCLCPP_INFO(enable_ros_logs_,
      get_logger(),
      "IDLE_TARGET update: source_frame=%s target_base=(%.2f, %.2f, %.2f), desired_base=%.3f, micro_yaw=%.3f, current_unwrapped=%.3f, ref_unwrapped=%.3f, desired_unwrapped=%.3f, cmd_yaw=%.3f, delta=%.3f",
      source_frame.c_str(),
      dx_base,
      dy_base,
      dz_base,
      desired_yaw_base,
      micro_yaw_,
      current_internal_yaw_unwrapped,
      unwrap_reference,
      desired_internal_yaw_unwrapped,
      idle_yaw_cmd_,
      delta_internal);

    return true;
  }

  void onTimer()
  {
    const rclcpp::Time tnow = now();

    const bool detection_fresh =
      have_valid_detection_ &&
      isFresh(last_valid_detection_time_, detection_timeout_sec_);

    const bool cv_cmd_fresh =
      have_cv_cmd_ &&
      isFresh(last_cv_cmd_time_, cv_cmd_timeout_sec_);

    const bool cv_mode = detection_fresh && cv_cmd_fresh;

    geometry_msgs::msg::Twist out;
    bool idle_mode = false;
    bool freeze_mode = false;

    if (cv_mode) {
      out.angular.z = last_cv_cmd_.angular.z;
      out.angular.y = last_cv_cmd_.angular.y;
      out.angular.x = last_cv_cmd_.angular.x;
      out.linear.z = 1.0;

      resetIdleUnwrap();
    }
    else if (use_idle_target_when_no_detection_ && computeIdleCommand(out, tnow)) {
      idle_mode = true;
    }
    else {
      freeze_mode = true;

      if (freeze_when_no_detection_ && have_micro_status_) {
        out.angular.z = micro_yaw_;
        out.angular.y = micro_pitch_;
      } else {
        out.angular.z = last_output_yaw_;
        out.angular.y = last_output_pitch_;
      }

      out.angular.x = 0.0;
      out.linear.z = 0.0;
    }

    last_output_yaw_ = out.angular.z;
    last_output_pitch_ = out.angular.y;

    if (cv_mode != last_cv_mode_ ||
        idle_mode != last_idle_mode_ ||
        freeze_mode != last_freeze_mode_)
    {
      if (cv_mode) {
        AA_RCLCPP_INFO(enable_ros_logs_,
          get_logger(),
          "Turret mode -> CV  detection_age=%.3f  cv_cmd_age=%.3f",
          (tnow - last_valid_detection_time_).seconds(),
          (tnow - last_cv_cmd_time_).seconds());
      } else if (idle_mode) {
        AA_RCLCPP_INFO(enable_ros_logs_, get_logger(), "Turret mode -> IDLE_TARGET");
      } else {
        AA_RCLCPP_INFO(enable_ros_logs_, get_logger(), "Turret mode -> FREEZE");
      }

      last_cv_mode_ = cv_mode;
      last_idle_mode_ = idle_mode;
      last_freeze_mode_ = freeze_mode;
    }

    turret_cmd_pub_->publish(out);
  }

private:
  std::string cv_cmd_topic_;
  std::string detection_topic_;
  std::string micro_status_topic_;
  std::string idle_target_topic_;
  std::string output_topic_;
  std::string map_frame_;
  std::string base_frame_;

  double publish_rate_hz_{50.0};
  double detection_timeout_sec_{0.50};
  double cv_cmd_timeout_sec_{0.50};
  double min_detection_score_{0.12};
  double tf_timeout_sec_{0.05};

  bool freeze_when_no_detection_{true};
  bool use_idle_target_when_no_detection_{true};
  bool enable_ros_logs_{false};

  double yaw_sign_{1.0};
  double yaw_zero_offset_{0.0};

  double idle_yaw_rate_limit_{1.50};
  double idle_recompute_period_sec_{0.10};
  double idle_yaw_deadband_{0.03};

  std::string idle_pitch_mode_{"hold_last_micro"};
  double idle_pitch_static_{0.0};
  double idle_pitch_sign_{1.0};
  double idle_pitch_zero_offset_{0.0};
  double idle_target_z_{0.42};
  bool use_idle_point_z_{false};
  double gimbal_height_{0.42};

  bool use_default_idle_target_{false};
  double default_idle_target_x_{6.0};
  double default_idle_target_y_{4.0};
  double default_idle_target_z_{0.42};

  double target_reset_distance_{0.50};

  std::vector<std::string> valid_class_ids_{"0", "2"};
  std::unordered_set<std::string> valid_class_set_;

  bool target_classes_from_micro_status_{true};
  int target_color_status_index_{4};
  std::vector<std::string> micro_color_target_classes_{"0", "2"};
  bool have_micro_target_color_{false};
  int last_micro_target_color_{-1};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cv_cmd_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr micro_status_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr idle_target_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr turret_cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    parameter_callback_handle_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  geometry_msgs::msg::Twist last_cv_cmd_;
  rclcpp::Time last_cv_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_valid_detection_time_{0, 0, RCL_ROS_TIME};

  bool have_cv_cmd_{false};
  bool have_valid_detection_{false};

  bool have_micro_status_{false};
  double micro_yaw_{0.0};
  double micro_pitch_{0.0};

  geometry_msgs::msg::PointStamped idle_target_;
  bool have_idle_target_{false};

  rclcpp::Time last_idle_recompute_time_{0, 0, RCL_ROS_TIME};
  bool have_idle_command_{false};
  double idle_yaw_cmd_{0.0};
  double idle_pitch_cmd_{0.0};

  bool have_last_desired_internal_yaw_{false};
  double last_desired_internal_yaw_unwrapped_{0.0};

  double last_output_yaw_{0.0};
  double last_output_pitch_{0.0};

  bool last_cv_mode_{false};
  bool last_idle_mode_{false};
  bool last_freeze_mode_{false};
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TurretYawMux>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
