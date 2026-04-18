#ifndef AUTO_AIM_TARGETING__IO__OUTPUT_PUBLISHER_HPP_
#define AUTO_AIM_TARGETING__IO__OUTPUT_PUBLISHER_HPP_

#include <map>
#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/publisher.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "auto_aim_targeting/io/gimbal_command_controller.hpp"
#include "auto_aim_targeting/io/debug_publisher.hpp"
#include "auto_aim_targeting/types.hpp"
#include "auto_aim_targeting/config.hpp"
#include "auto_aim_targeting/tracking/tracker.hpp"

namespace rm_auto_aim
{

class TargetingOutputPublisher
{
public:
  TargetingOutputPublisher(
    const TargetingConfig & config,
    rclcpp::Publisher<auto_aim_interfaces::msg::TrackerInfo>::SharedPtr info_pub,
    rclcpp::Publisher<auto_aim_interfaces::msg::Targets>::SharedPtr targets_pub,
    rclcpp::Publisher<vision_msgs::msg::Detection2D>::SharedPtr optimal_bbox_pub,
    rclcpp::Publisher<auto_aim_interfaces::msg::GimbalCmd>::SharedPtr cmd_pub,
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub,
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_marker_pub,
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub);

  AimResult buildAimResult(
    const Observation & input,
    const std::map<int, std::unique_ptr<Tracker>> & trackers,
    int best_tracker_id,
    const std::optional<AimDecision> & decision,
    const FireGateResult & fire_result);

  void publish(const AimResult & output);

private:
  const Tracker * findTracker(
    const std::map<int, std::unique_ptr<Tracker>> & trackers,
    int tracker_id) const;

  TargetingDebugPublisher targeting_debug_publisher_;
  GimbalCommandController gimbal_command_controller_;
  rclcpp::Publisher<auto_aim_interfaces::msg::TrackerInfo>::SharedPtr info_pub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Targets>::SharedPtr targets_pub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2D>::SharedPtr optimal_bbox_pub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::GimbalCmd>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  std::string target_frame_;
  visualization_msgs::msg::Marker position_marker_;
  visualization_msgs::msg::Marker linear_v_marker_;
  visualization_msgs::msg::Marker angular_v_marker_;
  visualization_msgs::msg::Marker armor_marker_;
};

}  // namespace rm_auto_aim

#endif  // AUTO_AIM_TARGETING__IO__OUTPUT_PUBLISHER_HPP_
