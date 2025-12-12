// Copyright 2022 Chen Jun

#include "rm_trajectory/trajectory_solver.hpp"

#include <cmath>
#include <memory>
#include <vector>

#include "rclcpp_components/register_node_macro.hpp"

namespace rm_auto_aim {
TrajectorySolverNode::TrajectorySolverNode(const rclcpp::NodeOptions &options)
    : Node("trajectory_solver", options) {
  RCLCPP_INFO(this->get_logger(), "Starting TrajectorySolverNode!");

  bullet_speed_ = this->declare_parameter("bullet_speed", 25.0);
  gravity_ = this->declare_parameter("gravity", 9.8);
  // Air resistance coefficient (simple linear approximation: a = -k*v)
  k_ = this->declare_parameter("k", 0.01);
  time_bias_ = this->declare_parameter("time_bias", 0.05);

  target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
      "/tracker/target", rclcpp::SensorDataQoS(),
      std::bind(&TrajectorySolverNode::targetCallback, this,
                std::placeholders::_1));

  cmd_pub_ = this->create_publisher<auto_aim_interfaces::msg::GimbalCmd>(
      "/tracker/cmd_gimbal", rclcpp::SensorDataQoS());

  // TODO: change topic name
  twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/tracker/cmd_vel", rclcpp::SensorDataQoS());

  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/trajectory/marker", 10);
}

void TrajectorySolverNode::targetCallback(
    const auto_aim_interfaces::msg::Target::SharedPtr msg) {
  if (!msg->tracking) {
    return;
  }

  // Get current state
  double x = msg->position.x;
  double y = msg->position.y;
  double z = msg->position.z;
  double vx = msg->velocity.x;
  double vy = msg->velocity.y;
  double vz = msg->velocity.z;

  // Simple iteration to find impact point
  // 1. Initial guess: t = dist / v
  double dist = std::sqrt(x * x + y * y);
  double t = dist / bullet_speed_;

  // Max iterations
  const int max_iter = 10;
  double pitch = 0.0;
  double yaw = 0.0;
  double final_x = 0, final_y = 0, final_z = 0;

  // TODO: maybe move implement analytical and not iterative solution to this
  // problem Predict target position at time t Linear prediction for target
  // motion (assuming constant velocity) Account for system latency (time_bias_)
  double prediction_time = t + time_bias_;
  double target_x = x + vx * prediction_time;
  double target_y = y + vy * prediction_time;
  double target_z = z + vz * prediction_time;

  // Calculate required yaw
  final_x = target_x;
  final_y = target_y;
  final_z = target_z;
  yaw = std::atan2(target_y, target_x);

  // Calculate horizontal distance to target
  double ground_dist = std::sqrt(target_x * target_x + target_y * target_y);

  // Calculate pitch angle to compensate for gravity and air resistance
  // Using simple physics: z = v*sin(theta)*t - 0.5*g*t^2
  // This is an approximation. A more accurate solver would integrate diff eq.
  // Here we use the approximation: theta = atan((z + 0.5*g*t^2)/dist)
  // But we should also account for air drag slowing down the bullet.gravity_ =
  // this->declare_parameter("gravity", 9.8);

  // For small k, range = v*cos(theta)/k * (1 - exp(-k*t))

  // Let's use a simpler gravity drop model:
  // dz = 1/2 * g * t^2
  double dz = 0.5 * gravity_ * t * t;
  pitch = std::atan2(target_z + dz, ground_dist);

  // Recalculate time of flight based on new path length
  // With drag: t = (exp(k*dist) - 1) / (k * v_0 * cos(pitch)) ?
  // Let's stick to standard kinematic update for the iteration:
  // Path length approx = ground_dist / cos(pitch)
  double path_len =
      ground_dist /
      std::cos(pitch); // assuming this is triangle as a first approximation
                       // TODO: CHECK IF THIS CAN BE IMPROVED

  // Update t: path / speed
  double new_t = path_len / bullet_speed_;

  if (std::abs(new_t - t) < 0.001) {
    t = new_t;
    break;
  }
  t = new_t;
}

// Publish command
auto_aim_interfaces::msg::GimbalCmd cmd;
cmd.header = msg->header;
cmd.pitch = pitch;
cmd.yaw = yaw;
cmd.distance =
    std::sqrt(final_x * final_x + final_y * final_y + final_z * final_z);
cmd.fire_cmd = true; // Simple logic: always fire if tracking
cmd_pub_->publish(cmd);

// Publish twist
geometry_msgs::msg::Twist twist;
twist.angular.y = pitch * 180 / M_PI;
twist.angular.z = yaw * 180 / M_PI;
twist_pub_->publish(twist);

// Visualize impact point
visualization_msgs::msg::Marker marker;
marker.header = msg->header;
marker.ns = "impact_point";
marker.type = visualization_msgs::msg::Marker::SPHERE;
marker.action = visualization_msgs::msg::Marker::ADD;
marker.pose.position.x = final_x;
marker.pose.position.y = final_y;
marker.pose.position.z = final_z;
marker.scale.x = 0.1;
marker.scale.y = 0.1;
marker.scale.z = 0.1;
marker.color.a = 1.0;
marker.color.r = 0.0;
marker.color.g = 1.0;
marker.color.b = 0.0;
marker_pub_->publish(marker);
}

} // namespace rm_auto_aim

RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::TrajectorySolverNode);
