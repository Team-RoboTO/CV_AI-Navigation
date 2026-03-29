#ifndef AUTO_AIM_TARGETING__IO__DEBUG_PUBLISHER_HPP_
#define AUTO_AIM_TARGETING__IO__DEBUG_PUBLISHER_HPP_

#include <string>
#include <vector>

#include <rclcpp/time.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/tracker_info.hpp"
#include "auto_aim_targeting/tracking/tracker.hpp"

namespace rm_auto_aim
{

class TargetingDebugPublisher
{
public:
  explicit TargetingDebugPublisher(std::string target_frame)
  : target_frame_(std::move(target_frame)) {}

  void fillTargetMsg(
    auto_aim_interfaces::msg::Target & msg,
    const Tracker & tracker,
    const rclcpp::Time & time) const;

  auto_aim_interfaces::msg::TrackerInfo makeTrackerInfo(const Tracker & tracker) const;

  vision_msgs::msg::Detection2D selectOptimalBBox(
    const auto_aim_interfaces::msg::Armors::SharedPtr & armors,
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr & detection_msg,
    const std::vector<size_t> & detection_indices,
    const Tracker * best_tracker) const;

  visualization_msgs::msg::MarkerArray buildMarkers(
    const auto_aim_interfaces::msg::Target & target_msg,
    const Tracker * best_tracker,
    const visualization_msgs::msg::Marker & position_template,
    const visualization_msgs::msg::Marker & linear_v_template,
    const visualization_msgs::msg::Marker & angular_v_template,
    const visualization_msgs::msg::Marker & armor_template) const;

private:
  void buildTrackingMarkers(
    const auto_aim_interfaces::msg::Target & target_msg,
    const Tracker * best_tracker,
    visualization_msgs::msg::Marker & position_marker,
    visualization_msgs::msg::Marker & linear_v_marker,
    visualization_msgs::msg::Marker & angular_v_marker,
    visualization_msgs::msg::Marker & armor_marker,
    visualization_msgs::msg::MarkerArray & marker_array) const;
  void buildDeleteMarkers(
    visualization_msgs::msg::Marker & position_marker,
    visualization_msgs::msg::Marker & linear_v_marker,
    visualization_msgs::msg::Marker & angular_v_marker,
    visualization_msgs::msg::Marker & armor_marker,
    visualization_msgs::msg::MarkerArray & marker_array) const;
  std::string target_frame_;
};

}  // namespace rm_auto_aim

#endif  // AUTO_AIM_TARGETING__IO__DEBUG_PUBLISHER_HPP_
