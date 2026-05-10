#ifndef AUTO_AIM__DEBUG_PUBLISHER_HPP_
#define AUTO_AIM__DEBUG_PUBLISHER_HPP_

#include <auto_aim/msg/auto_aim_debug.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <unordered_map>

#include "auto_aim/debug_frame.hpp"

namespace auto_aim
{

// Publishes the per-frame DebugFrame on /auto_aim/debug and emits a
// blocker-reason histogram every fire_log_period_s seconds.
class DebugPublisher
{
public:
  DebugPublisher(rclcpp::Node * node, const std::string & topic = "/auto_aim/debug",
                 double fire_log_period_s = 10.0,
                 bool publish_enabled = true,
                 bool histogram_log_enabled = true);

  // Copy fields from the DebugFrame into the ROS message and publish.
  void publish(const std_msgs::msg::Header & header, const DebugFrame & f);

private:
  rclcpp::Node * node_;
  rclcpp::Publisher<auto_aim::msg::AutoAimDebug>::SharedPtr pub_;
  rclcpp::Time last_log_time_;
  double fire_log_period_s_;
  bool publish_enabled_;
  bool histogram_log_enabled_;
  std::unordered_map<uint8_t, uint64_t> blocker_counts_;
};

}  // namespace auto_aim
#endif
