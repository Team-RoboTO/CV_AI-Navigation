#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

using namespace std::chrono_literals;

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

    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 50.0);

    detection_timeout_sec_ = declare_parameter<double>("detection_timeout", 0.80);
    cv_cmd_timeout_sec_ = declare_parameter<double>("cv_cmd_timeout", 2.00);
    min_detection_score_ = declare_parameter<double>("min_detection_score", 0.05);

    valid_class_ids_ = declare_parameter<std::vector<std::string>>(
      "valid_class_ids",
      std::vector<std::string>{"0", "2"}
    );

    freeze_when_no_detection_ = declare_parameter<bool>("freeze_when_no_detection", true);

    valid_class_set_ = makeClassSet(valid_class_ids_);

    cv_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cv_cmd_topic_,
      10,
      std::bind(&TurretYawMux::onCvCmd, this, std::placeholders::_1)
    );

    detection_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
      detection_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&TurretYawMux::onDetections, this, std::placeholders::_1)
    );


    micro_status_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      micro_status_topic_,
      10,
      std::bind(&TurretYawMux::onMicroStatus, this, std::placeholders::_1)
    );

    turret_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      output_topic_,
      10
    );

    const double safe_rate = std::max(1.0, publish_rate_hz_);
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / safe_rate)
    );

    timer_ = create_wall_timer(
      period,
      std::bind(&TurretYawMux::onTimer, this)
    );

    parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&TurretYawMux::onSetParameters, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(), "turret_yaw_mux started");
    RCLCPP_INFO(get_logger(), "  cv_cmd_topic: %s", cv_cmd_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  detection_topic: %s", detection_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  micro_status_topic: %s", micro_status_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  output_topic: %s", output_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  detection_timeout: %.3f", detection_timeout_sec_);
    RCLCPP_INFO(get_logger(), "  cv_cmd_timeout: %.3f", cv_cmd_timeout_sec_);
    RCLCPP_INFO(get_logger(), "  min_detection_score: %.3f", min_detection_score_);
    RCLCPP_INFO(get_logger(), "  freeze_when_no_detection: %s",
                freeze_when_no_detection_ ? "true" : "false");
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

  void logValidClasses()
  {
    std::string classes;
    for (size_t i = 0; i < valid_class_ids_.size(); ++i) {
      classes += valid_class_ids_[i];
      if (i + 1 < valid_class_ids_.size()) {
        classes += ", ";
      }
    }

    RCLCPP_INFO(get_logger(), "  valid_class_ids: [%s]", classes.c_str());
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
        RCLCPP_INFO(get_logger(), "Updated detection_timeout: %.3f", detection_timeout_sec_);
      }

      else if (name == "cv_cmd_timeout") {
        const double v = param.as_double();

        if (v <= 0.0) {
          result.successful = false;
          result.reason = "cv_cmd_timeout must be > 0";
          return result;
        }

        cv_cmd_timeout_sec_ = v;
        RCLCPP_INFO(get_logger(), "Updated cv_cmd_timeout: %.3f", cv_cmd_timeout_sec_);
      }

      else if (name == "min_detection_score") {
        const double v = param.as_double();

        if (v < 0.0 || v > 1.0) {
          result.successful = false;
          result.reason = "min_detection_score must be in [0, 1]";
          return result;
        }

        min_detection_score_ = v;
        RCLCPP_INFO(get_logger(), "Updated min_detection_score: %.3f", min_detection_score_);
      }

      else if (name == "valid_class_ids") {
        valid_class_ids_ = param.as_string_array();
        valid_class_set_ = makeClassSet(valid_class_ids_);

        RCLCPP_INFO(get_logger(), "Updated valid_class_ids");
        logValidClasses();
      }

      else if (name == "freeze_when_no_detection") {
        freeze_when_no_detection_ = param.as_bool();
        RCLCPP_INFO(get_logger(), "Updated freeze_when_no_detection: %s",
                    freeze_when_no_detection_ ? "true" : "false");
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

    // /micro_status layout:
    // [0] yaw attuale micro
    // [1] pitch attuale micro
    micro_yaw_ = static_cast<double>(msg->data[0]);
    micro_pitch_ = static_cast<double>(msg->data[1]);
    have_micro_status_ = true;
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

      RCLCPP_DEBUG(
        get_logger(),
        "Valid detection: class_id=%s score=%.3f",
        best_class.c_str(),
        best_score
      );
    }
  }

  bool isFresh(const rclcpp::Time & stamp, double timeout_sec) const
  {
    const double age = (now() - stamp).seconds();
    return age >= 0.0 && age <= timeout_sec;
  }

  void onTimer()
  {
    const bool detection_fresh =
      have_valid_detection_ &&
      isFresh(last_valid_detection_time_, detection_timeout_sec_);

    const bool cv_cmd_fresh =
      have_cv_cmd_ &&
      isFresh(last_cv_cmd_time_, cv_cmd_timeout_sec_);

    const bool cv_mode = detection_fresh && cv_cmd_fresh;

    geometry_msgs::msg::Twist out;

    if (cv_mode) {
      // Modalità CV vera:
      // - detection valida class_id 0/2
      // - comando /cmd_vel_AI fresco
      out.angular.z = last_cv_cmd_.angular.z;  // yaw
      out.angular.y = last_cv_cmd_.angular.y;  // pitch
      out.angular.x = last_cv_cmd_.angular.x;  // shoot

      // marker debug: 1.0 = CV mode
      out.linear.z = 1.0;

      last_output_yaw_ = out.angular.z;
      last_output_pitch_ = out.angular.y;
    }

    else {
      // Nessuna detection valida:
      // freeze reale della canna.
      //
      // Non puntiamo più /turret/idle_target.
      // Non usiamo più il waypoint mappa.
      // Non usiamo stale /cmd_vel_AI.
      //
      // Il setpoint viene riportato alla posizione attuale letta dal micro,
      // così yaw/pitch comandati coincidono con yaw/pitch misurati.
      if (freeze_when_no_detection_ && have_micro_status_) {
        out.angular.z = micro_yaw_;
        out.angular.y = micro_pitch_;
      } else {
        out.angular.z = last_output_yaw_;
        out.angular.y = last_output_pitch_;
      }

      out.angular.x = 0.0;  // shoot off

      // marker debug: 0.0 = freeze / no CV
      out.linear.z = 0.0;

      last_output_yaw_ = out.angular.z;
      last_output_pitch_ = out.angular.y;
    }

    if (cv_mode != last_cv_mode_) {
      if (cv_mode) {
        RCLCPP_INFO(
          get_logger(),
          "Turret mode -> CV  detection_age=%.3f  cv_cmd_age=%.3f",
          (now() - last_valid_detection_time_).seconds(),
          (now() - last_cv_cmd_time_).seconds()
        );
      } else {
        RCLCPP_INFO(get_logger(), "Turret mode -> FREEZE");
      }

      last_cv_mode_ = cv_mode;
    }

    turret_cmd_pub_->publish(out);
  }

private:
  std::string cv_cmd_topic_;
  std::string detection_topic_;
  std::string micro_status_topic_;
  std::string output_topic_;

  double publish_rate_hz_{50.0};
  double detection_timeout_sec_{0.50};
  double cv_cmd_timeout_sec_{0.50};
  double min_detection_score_{0.12};

  bool freeze_when_no_detection_{true};

  std::vector<std::string> valid_class_ids_{"0", "2"};
  std::unordered_set<std::string> valid_class_set_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cv_cmd_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr micro_status_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr turret_cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  geometry_msgs::msg::Twist last_cv_cmd_;
  rclcpp::Time last_cv_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_valid_detection_time_{0, 0, RCL_ROS_TIME};

  bool have_cv_cmd_{false};
  bool have_valid_detection_{false};

  bool have_micro_status_{false};
  double micro_yaw_{0.0};
  double micro_pitch_{0.0};

  double last_output_yaw_{0.0};
  double last_output_pitch_{0.0};

  bool last_cv_mode_{false};
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TurretYawMux>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}