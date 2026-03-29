#ifndef RM_TRAJECTORY__TRAJECTORY_SOLVER_NODE_HPP_
#define RM_TRAJECTORY__TRAJECTORY_SOLVER_NODE_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/header.hpp>

#include "auto_aim_interfaces/msg/gimbal_cmd.hpp"
#include "auto_aim_interfaces/msg/targets.hpp"
#include "rm_trajectory/engagement_selector.hpp"

namespace rm_auto_aim
{

class TrajectorySolverNode : public rclcpp::Node
{
public:
  explicit TrajectorySolverNode(const rclcpp::NodeOptions & options);

private:
  void targetsCallback(auto_aim_interfaces::msg::Targets::UniquePtr msg);
  void microPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void cameraImuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);

  void publishSafeCommand(const std_msgs::msg::Header & header);
  void publishShotPlan(const std_msgs::msg::Header & header, const ShotPlan & plan, bool fire);

  SolverContext buildContext(double transport_delay) const;
  bool poseIsFresh() const;

  BallisticsParams ballistics_params_;
  BallisticsSolver ballistics_solver_;
  ArmorPredictor predictor_;
  CostWeights cost_weights_;
  EngagementSelector selector_;

  rclcpp::Subscription<auto_aim_interfaces::msg::Targets>::SharedPtr targets_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr micro_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  rclcpp::Publisher<auto_aim_interfaces::msg::GimbalCmd>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  double bullet_speed_ = 25.0;
  double gravity_ = 9.8;
  double linear_drag_coeff_ = 0.01;
  double quadratic_drag_coeff_ = 0.01;
  bool use_quadratic_drag_ = false;
  double time_bias_ = 0.08;
  double time_bias_alpha_ = 0.35;
  double gimbal_response_delay_ = 0.0;
  double gimbal_height_ = 0.5;
  double min_fire_dist_ = 0.5;
  double max_fire_dist_ = 10.0;
  double angular_window_ = 0.09;
  double angular_window_ref_dist_ = 3.0;
  double max_measurement_age_ = 0.10;
  double max_gimbal_yaw_rate_ = 6.0;
  double max_gimbal_pitch_rate_ = 4.0;
  double indirect_vyaw_threshold_ = 3.0;
  double indirect_timing_tolerance_ = 0.02;
  int indirect_max_candidates_ = 8;
  double oblique_exponent_ = 2.0;
  double gimbal_pitch_max_ = 0.524;
  double gimbal_pitch_min_ = -0.524;
  double cmd_smooth_alpha_ = 0.4;
  double max_cmd_angle_ = 15.0;
  double latency_gate_sigma_ = 2.5;
  double time_bias_var_ = 0.0009;

  std::string pose_source_ = "none";
  double pose_timeout_ = 0.15;
  double yaw_sign_ = 1.0;
  double pitch_sign_ = 1.0;
  double current_yaw_ = 0.0;
  double current_pitch_ = 0.0;
  rclcpp::Time last_pose_time_{0, 0, RCL_ROS_TIME};

  double smoothed_rel_yaw_ = 0.0;
  double smoothed_rel_pitch_ = 0.0;
  bool cmd_smooth_initialized_ = false;
  int previous_tracker_id_ = -1;

  int latency_warmup_samples_ = 5;
  int latency_warmup_count_ = 0;
  bool indirect_mode_active_ = false;
};

}  // namespace rm_auto_aim

#endif  // RM_TRAJECTORY__TRAJECTORY_SOLVER_NODE_HPP_
