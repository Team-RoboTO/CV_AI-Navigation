#ifndef RM_TRAJECTORY__TRAJECTORY_SOLVER_HPP_
#define RM_TRAJECTORY__TRAJECTORY_SOLVER_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tuple>
#include <visualization_msgs/msg/marker.hpp>

#include "auto_aim_interfaces/msg/gimbal_cmd.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/targets.hpp"

namespace rm_auto_aim {

// ---------------------------------------------------------------------------
// TrajectorySolverNode — ballistic trajectory solver and fire-gate controller.
//
// Subscribes to /tracker/target (EKF output: robot center position, velocity,
// yaw, yaw-rate, radii, dz) and publishes gimbal pitch/yaw commands + fire
// trigger on /tracker/cmd_gimbal and /cmd_vel.
//
// Two aiming modes:
//
//   DIRECT (default, |v_yaw| < threshold):
//     Predicts which armor face will be closest + most face-on at bullet
//     impact time, solves the ballistic trajectory (iterative, with air drag),
//     and fires when the angular gap between barrel bearing and face normal
//     is within a configurable window.
//
//   INDIRECT (|v_yaw| > indirect_vyaw_threshold_):
//     For fast spinners.  Enumerates upcoming alignment windows (moments when
//     any face will be squarely facing the camera), solves trajectory to each
//     candidate impact point, and fires when timing residual < tolerance.
//
// Ballistic model (linear air drag):
//   Horizontal: x(t) = v·cos(θ) · (1 − e^(−kt)) / k
//   Vertical:   z(t) = v·sin(θ)·t − ½·g·t²
//   Solver iterates pitch ↔ flight-time until convergence (~3–5 passes).
//
// Pipeline latency is compensated via an adaptive EMA with outlier rejection.
// Acceleration is estimated from consecutive EKF velocity outputs.
// ---------------------------------------------------------------------------
class TrajectorySolverNode : public rclcpp::Node {
public:
  explicit TrajectorySolverNode(const rclcpp::NodeOptions &options);

private:
  void targetCallback(auto_aim_interfaces::msg::Target::UniquePtr msg);
  void microPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void cameraImuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void targetsCallback(auto_aim_interfaces::msg::Targets::UniquePtr msg);

  // Solves for the pitch angle given position and velocity
  // Returns tuple<pitch, flight_time, reachable>
  // reachable is false when the unclamped pitch exceeds gimbal limits
  std::tuple<double, double, bool> solveTrajectory(const double dist,
                                                   const double z,
                                                   const double v);

  rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;
  rclcpp::Subscription<auto_aim_interfaces::msg::Targets>::SharedPtr targets_sub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::GimbalCmd>::SharedPtr cmd_pub_;

  // Dedup: skip legacy /tracker/target if we already processed the same timestamp
  // via /tracker/targets
  rclcpp::Time last_targets_stamp_{0, 0, RCL_ROS_TIME};

  // Debug visualization
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  // Parameters
  double bullet_speed_;
  double gravity_;         // Gravity acceleration
  double k_;               // Air resistance coefficient
  double time_bias_;       // Adaptive latency compensation (EMA)
  double time_bias_alpha_; // EMA smoothing factor for time_bias_
  // ---------------------------------------------------------------------------
  // Gimbal response delay — constant offset added to the prediction horizon.
  //
  // PROBLEM: time_bias_ models camera→solver latency (measured via EMA), but
  // the additional delay from solver→serial→gimbal PID response is NOT
  // captured.  The gimbal doesn't instantly reach the commanded angle — the
  // STM32 reads the UART packet, the PID loop settles, and the motor moves.
  //
  // IMPACT: At 1 m/s target speed and 15ms unmodeled gimbal delay, the bullet
  // hits 15mm behind the target — a systematic bias that cannot be tuned away
  // with angular_window because it's directional (always behind, never ahead).
  //
  // SOLUTION: total_prediction_time = t_flight + time_bias_ + gimbal_response_delay_
  // Default 0.0 (no change until measured on real hardware).
  // TUNING: Start at 0, increase in 5ms steps while observing systematic aim
  // offset on a moving target.  Typical value: 10-20ms.
  // ---------------------------------------------------------------------------
  double gimbal_response_delay_;
  double gimbal_height_;   // Barrel height above odom origin (metres)
  double min_fire_dist_;   // Minimum engagement range (metres)
  double max_fire_dist_;   // Maximum engagement range (metres)
  double angular_window_;             // Half-width of armor face window (rad)
  double angular_window_ref_dist_;   // Reference distance for window scaling (metres)
  double max_measurement_age_;        // Maximum allowed measurement staleness (seconds)
  bool use_quadratic_drag_;          // Use quadratic drag model (F=-c*v^2) instead of linear
  double max_gimbal_yaw_rate_;       // Max gimbal yaw slew rate (rad/s)
  double max_gimbal_pitch_rate_;     // Max gimbal pitch slew rate (rad/s)

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

  // --- Acceleration estimation (2nd-order motion model) ---
  //
  // The solver uses acceleration to predict where the target will be at
  // bullet impact time: pos(t) = pos + vel·t + ½·accel·t².
  //
  // Two modes are available:
  //
  //   use_msg_acceleration_ = true (DEFAULT, RECOMMENDED):
  //     Read acceleration directly from msg->acceleration, which the tracker
  //     computes via the innovation-based method (v_posterior - v_prior) / dt.
  //     This is ~3× less noisy than finite-differencing (see Target.msg
  //     comments for the math) and has no additional phase lag because the
  //     EMA is applied at the source (in the tracker).
  //
  //   use_msg_acceleration_ = false (FALLBACK):
  //     Differentiates consecutive EKF velocity outputs and EMA-smooths.
  //     This is the legacy approach, kept as a safety net in case the
  //     tracker-published acceleration proves worse in real testing.
  //     To switch: ros2 param set /trajectory_solver use_msg_acceleration false
  //
  // When use_msg_acceleration_=true, the fallback members (prev_vx_, ax_ema_,
  // etc.) are still updated each frame so a runtime switch is seamless.
  bool use_msg_acceleration_;             // Use tracker-published acceleration
  bool has_prev_target_ = false;          // First frame flag (fallback mode)
  int32_t prev_tracker_id_ = -1;          // Track identity switches to reset accel EMA
  rclcpp::Time prev_target_time_;         // Timestamp of previous target message
  double prev_vx_ = 0.0, prev_vy_ = 0.0, prev_vz_ = 0.0;  // Previous velocities (fallback)
  double ax_ema_ = 0.0, ay_ema_ = 0.0, az_ema_ = 0.0;      // EMA-smoothed acceleration (fallback)
  double accel_ema_alpha_;                // EMA weight (higher = more responsive, noisier)
  double max_accel_;                      // Clamp on raw accel (rejects EKF jumps)
  double ref_frequency_;                  // [Hz] reference frame rate for time-normalization

  // --- Latency EMA with outlier rejection ---
  // Tracks pipeline delay (camera → YOLO → EKF → here) as an EMA of
  // measured_latency = now() − msg->header.stamp.  Outlier gate prevents
  // GPU throttle spikes from corrupting the estimate.
  double time_bias_var_ = 0.0009;  // Running variance of latency (σ² ≈ 30ms²)
  double latency_gate_sigma_;      // Reject samples > N·σ from mean
  int latency_warmup_count_ = 0;   // Accept first N samples unconditionally
  static constexpr int kLatencyWarmupSamples = 5;

  // Oblique face scoring exponent (higher = more penalty for oblique faces)
  double oblique_exponent_;
  double gimbal_pitch_max_;  // Physical gimbal pitch upper limit (rad)
  double gimbal_pitch_min_;  // Physical gimbal pitch lower limit (rad)

  // --- Indirect aiming for fast spinners ---
  // When |v_yaw| > threshold, switch from aiming at the current face to
  // pre-aiming at a future alignment window when any face will be face-on.
  double indirect_vyaw_threshold_;     // Activation threshold [rad/s] (hysteresis at 70%)
  double indirect_timing_tolerance_;   // Minimum timing residual tolerance [s]
  int indirect_max_candidates_;        // Max alignment windows to evaluate per frame
  bool indirect_mode_active_ = false;  // Current mode state (with hysteresis)

  // --- Current gimbal position (from /micro_pose serial feedback) ---
  // Used to compute relative commands: target_angle − current_angle
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr micro_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  double current_pitch_ = 0.0;   // Current gimbal pitch [rad]
  double current_yaw_ = 0.0;     // Current gimbal yaw [rad]
  double yaw_sign_ = 1.0;        // Sign convention correction for yaw
  double pitch_sign_ = 1.0;      // Sign convention correction for pitch
  rclcpp::Time last_pose_time_{0, 0, RCL_ROS_TIME};
  double pose_timeout_;     // Suppress fire when /micro_pose is stale [s]
  std::string pose_source_;               // Flag to enable/disable checking for real gimbal/imu data

  // --- Gimbal command smoothing (EMA) ---
  // Filters the relative pitch/yaw commands to suppress frame-to-frame jitter
  // caused by EKF measurement noise.  Resets on tracker ID change.
  double cmd_smooth_alpha_;                // EMA weight (0→max smooth, 1→no smooth)
  double max_cmd_angle_;                   // Hard clamp: max |relative cmd| per frame [deg]
  double smoothed_abs_yaw_ = 0.0;         // EMA state for absolute target yaw [rad]
  double smoothed_abs_pitch_ = 0.0;       // EMA state for absolute target pitch [rad]
  bool cmd_smooth_initialized_ = false;   // First-frame flag

  // Compute when a spinning armor face will next point at the camera.
  // Used by indirect mode to enumerate alignment windows.
  // Returns time [s] until next alignment (always >= 0).
  double nextAlignmentTime(double yaw, double v_yaw, double face_idx,
                           double face_spacing, double bearing) const;
};

} // namespace rm_auto_aim

#endif // RM_TRAJECTORY__TRAJECTORY_SOLVER_HPP_
