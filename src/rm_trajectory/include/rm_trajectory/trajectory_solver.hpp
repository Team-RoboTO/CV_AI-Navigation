#ifndef RM_TRAJECTORY__TRAJECTORY_SOLVER_HPP_
#define RM_TRAJECTORY__TRAJECTORY_SOLVER_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "auto_aim_interfaces/msg/gimbal_cmd.hpp"
#include "auto_aim_interfaces/msg/target.hpp"

namespace rm_auto_aim {
class TrajectorySolverNode : public rclcpp::Node {
public:
  explicit TrajectorySolverNode(const rclcpp::NodeOptions &options);

private:
  void targetCallback(const auto_aim_interfaces::msg::Target::SharedPtr msg);

  // Solves for the pitch angle given position and velocity
  // Returns pair<pitch, flight_time>
  std::pair<double, double> solveTrajectory(const double dist, const double z,
                                            const double v);

  rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::GimbalCmd>::SharedPtr cmd_pub_;

  // Debug visualization
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  // Parameters
  double bullet_speed_;
  double gravity_;   // Gravity acceleration
  double k_;         // Air resistance coefficient
  double time_bias_; // System latency compensation

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
};

} // namespace rm_auto_aim

#endif // RM_TRAJECTORY__TRAJECTORY_SOLVER_HPP_
