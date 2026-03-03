#ifndef RM_TRAJECTORY__TRAJECTORY_SOLVER_HPP_
#define RM_TRAJECTORY__TRAJECTORY_SOLVER_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>
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
  void targetCallback(auto_aim_interfaces::msg::Target::UniquePtr msg);

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
  double gravity_;         // Gravity acceleration
  double k_;               // Air resistance coefficient
  double time_bias_;       // Adaptive latency compensation (EMA)
  double time_bias_alpha_; // EMA smoothing factor for time_bias_
  double gimbal_height_;   // Barrel height above odom origin (metres)
  double min_fire_dist_;   // Minimum engagement range (metres)
  double max_fire_dist_;   // Maximum engagement range (metres)
  double angular_window_;             // Half-width of armor face window (rad)

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

  // Acceleration estimation from consecutive target messages
  bool has_prev_target_ = false;
  rclcpp::Time prev_target_time_;
  double prev_vx_ = 0.0, prev_vy_ = 0.0, prev_vz_ = 0.0;
  double ax_ema_ = 0.0, ay_ema_ = 0.0, az_ema_ = 0.0;
  double accel_ema_alpha_;
  double max_accel_;

  // Latency outlier rejection
  double time_bias_var_ = 0.01;
  double latency_gate_sigma_;

  // Oblique face scoring exponent (higher = more penalty for oblique faces)
  double oblique_exponent_;

  // Indirect aiming for fast spinners
  double indirect_vyaw_threshold_;
  double indirect_timing_tolerance_;
  int indirect_max_candidates_;
  bool indirect_mode_active_ = false;

  // Gimbal relative angle subscription
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr micro_pose_sub_;
  double current_pitch_ = 0.0;
  double current_yaw_ = 0.0;
  double yaw_sign_ = 1.0;
  double pitch_sign_ = 1.0;

  double nextAlignmentTime(double yaw, double v_yaw, double face_idx,
                           double face_spacing, double bearing) const;
};

} // namespace rm_auto_aim

#endif // RM_TRAJECTORY__TRAJECTORY_SOLVER_HPP_
