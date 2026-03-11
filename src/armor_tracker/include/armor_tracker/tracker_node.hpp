#ifndef ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_
#define ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_

// ROS
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// STD
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <map>

#include "armor_tracker/pnp_solver.hpp"
#include "armor_tracker/tracker.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/targets.hpp"
#include "auto_aim_interfaces/msg/tracker_info.hpp"

namespace rm_auto_aim
{
using tf2_filter = tf2_ros::MessageFilter<vision_msgs::msg::Detection2DArray>;

// ---------------------------------------------------------------------------
// ArmorTrackerNode — Multi-target tracker for RoboMaster auto-aim.
//
// Pipeline (called once per YOLO detection frame at ~30 Hz):
//
//   1. YOLO Detection2DArray arrives → PnP solve each bbox → 3D Armor poses
//   2. Transform armors: camera frame → odom frame (via dynamic gimbal TF)
//   3. Filter: remove impossible Z, beyond max range
//   4. For each existing Tracker: adaptive damping → predictStep()
//   5. Global data association: assign each detection to closest tracker
//      (Mahalanobis distance, with 4× penalty for DETECTING trackers)
//   6. Update each Tracker with its assigned detections (EKF update)
//   7. Spawn new Trackers for unassigned detections (if not too close)
//   8. Prune LOST trackers
//   9. Select best tracker (lowest: range × state_penalty × (1+v_yaw_var))
//  10. Publish: Targets (all), Target (best), TrackerInfo, optimal bbox, markers
//
// Each Tracker has its own EKF instance created via createTracker() factory.
// The EKF lambdas capture a raw pointer to their Tracker for per-tracker
// damping alpha access.
// ---------------------------------------------------------------------------
class ArmorTrackerNode : public rclcpp::Node
{
public:
  explicit ArmorTrackerNode(const rclcpp::NodeOptions & options);

private:
  // Main callback: receives YOLO detections, runs full pipeline (steps 1-10)
  void armorsCallback(const vision_msgs::msg::Detection2DArray::ConstSharedPtr armors_ptr);

  // Publish RViz visualization markers (center, velocity arrows, armor cubes)
  void publishMarkers(const auto_aim_interfaces::msg::Target & target_msg);

  // Factory: creates a Tracker with fresh EKF, configured with all ROS params.
  // Each call produces an independent Tracker with its own EKF lambdas.
  std::unique_ptr<Tracker> createTracker();

  // Fill a Target ROS message from a Tracker's current EKF state
  void fillTargetMsg(
    auto_aim_interfaces::msg::Target & msg,
    Tracker & tracker,
    const rclcpp::Time & time,
    const rclcpp::Time & last_meas_time);

  // Receive gimbal feedback from lower computer → update gimbal angles
  void microPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  // Broadcast odom → camera_color_optical_frame TF (includes gimbal + chassis rotation)
  void broadcastGimbalTF(const rclcpp::Time & stamp);

  // Maximum allowable armor distance in the XOY plane
  double max_armor_distance_;

  // YOLO class IDs to track (e.g. "3" = red_armor); grey/dead ("1") always excluded
  std::set<std::string> target_classes_;

  // The time when the last message was received
  rclcpp::Time last_time_;
  double ref_frequency_ = 30.0;          // [Hz] reference frame rate for time-normalization
  double dt_ = 1.0 / ref_frequency_;

  // --- EKF noise model parameters (used in createTracker lambdas) ---
  double s2qxyz_, s2qyaw_, s2qr_;    // Process noise σ² for xyz, yaw, radius
  double r_xyz_base_, r_xyz_slope_;   // R_pos = (base + slope × dist)² — range-dependent
  double r_yaw_base_, r_yaw_slope_;   // R_yaw = (base + slope × dist)² — range-dependent
  double r_yaw_angle_power_;          // Exponent n for obliquity inflation: 1/cos^n(face_angle)
  double max_yaw_oblique_deg_;        // Face angle [°] beyond which yaw noise → ∞
  bool use_secondary_face_fusion_;    // Fuse secondary armor face into 9D EKF
  double secondary_r_inflation_;      // Safety multiplier on secondary face R matrix
  double secondary_maha_threshold_;   // Mahalanobis gate for secondary face update
  double lost_time_thres_;            // Seconds in TEMP_LOST before transitioning to LOST

  // --- Multi-tracker infrastructure ---
  // Each tracker has a unique integer ID and its own EKF instance.
  // trackers_ maps ID → Tracker;  tracker_last_meas_times_ maps ID → last measurement time.
  std::map<int, std::unique_ptr<Tracker>> trackers_;
  std::map<int, rclcpp::Time> tracker_last_meas_times_;
  int next_tracker_id_ = 0;       // Monotonically increasing ID for new trackers
  int best_tracker_id_ = -1;      // ID of the current primary target (-1 = none)
  int max_trackers_;               // Maximum concurrent trackers (prevents runaway spawning)
  double new_tracker_min_dist_;    // Minimum Euclidean distance to existing trackers for spawning

  // Tracker construction parameters (stored for createTracker factory)
  double max_match_distance_;
  double max_track_range_;
  int tracking_thres_;
  double initial_r1_, initial_r2_;
  double r_kf_Q_, r_kf_R_, r_kf_P_init_;
  double r_adapt_max_dist_, dz_adapt_alpha_, r_yaw_uncertainty_scale_;
  double v_yaw_max_;
  double maha_match_threshold_, maha_jump_threshold_;

  // Camera info
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  cv::Point2f cam_center_;
  std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;
  std::unique_ptr<PnPSolver> pnp_solver_;
  // Reset tracker service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_tracker_srv_;

  // Subscriber with tf2 message_filter
  std::string target_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr armors_sub_;

  // Tracker info publisher
  rclcpp::Publisher<auto_aim_interfaces::msg::TrackerInfo>::SharedPtr info_pub_;

  // Publishers
  rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr target_pub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Targets>::SharedPtr targets_pub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2D>::SharedPtr optimal_bbox_pub_;

  // --- Adaptive velocity damping ---
  // Prevents EKF velocity overshoot oscillation.  When the innovation
  // (measurement − prediction) opposes the velocity, the filter is
  // overshooting.  We reduce the damping alpha to brake the velocity.
  //
  // alpha = 1.0: no damping (velocity unchanged)
  // alpha < 1.0: velocity decays each frame (braking)
  // alpha is time-adjusted: alpha^(dt/dt_nominal) for frame-rate independence
  double xyz_damping_alpha_;          // Current effective XYZ value (computed each frame)
  double yaw_damping_alpha_;          // Current effective yaw value (computed each frame)
  double xyz_damping_alpha_base_;     // ROS param: baseline alpha for XYZ (e.g. 0.95)
  double yaw_damping_alpha_base_;     // ROS param: baseline alpha for yaw (e.g. 0.98)
  double coast_damping_factor_;       // Multiplier for TEMP_LOST: base × factor (e.g. 0.85)
  double damping_innov_threshold_;    // Position innovation threshold for overshoot [m]
  double yaw_innov_threshold_;        // Yaw innovation threshold for overshoot [rad]

  // --- Innovation-based acceleration estimation (per-tracker) ---
  //
  // Each tracker maintains its own EMA-smoothed acceleration estimate, keyed
  // by tracker_id.  This is computed in fillTargetMsg() as:
  //   raw_accel = (v_posterior − v_prior) / dt
  // then EMA-smoothed with time-adjusted alpha for frame-rate independence.
  //
  // WHY PER-TRACKER: Different robots accelerate independently.  A single
  // global accel EMA would mix acceleration signals from different targets
  // after a best-target switch, causing transient prediction errors for
  // several frames.  Per-tracker EMAs preserve each robot's acceleration
  // history across target switches and resume cleanly when a robot is
  // re-selected as best target.
  //
  // WHY std::map<int, double>: The map auto-initializes to 0.0 via
  // operator[], matching the desired zero-acceleration prior for new trackers.
  // Entries are erased when their tracker is pruned (LOST state) or on reset.
  //
  // accel_ema_alpha_: EMA weight (0→max smooth, 1→no filter).  Default 0.3
  // matches the old trajectory solver parameter, but the input signal is
  // now ~3× cleaner, so the EMA adds less distortion.
  std::map<int, double> tracker_ax_ema_, tracker_ay_ema_, tracker_az_ema_;
  double accel_ema_alpha_;  // EMA weight for acceleration smoothing

  // PnP light-bar ratio (bbox → light-bar scaling)
  double light_ratio_;

  // Letterbox padding Y offset (pixels)
  double bbox_padding_y_;

  // Dynamic TF from gimbal feedback
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr micro_pose_sub_;
  double gimbal_yaw_, gimbal_pitch_;
  double gimbal_height_, yaw_sign_, pitch_sign_;
  rclcpp::Time last_micro_pose_time_{0, 0, RCL_ROS_TIME};
  double tracker_micro_pose_timeout_ = 0.2;

  // --- IMU-based chassis rotation compensation ---
  // The camera IMU measures total angular velocity (chassis + gimbal).
  // We subtract gimbal velocity (from /micro_pose) to isolate chassis rotation,
  // then integrate to estimate chassis yaw/pitch.  This is applied to the TF
  // so armor positions in odom frame aren't corrupted by chassis turning.
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);

  double chassis_yaw_ = 0.0;         // Integrated chassis yaw [rad]
  double chassis_pitch_ = 0.0;       // Integrated chassis pitch [rad]
  double imu_wx_filtered_ = 0.0;     // EMA-filtered gyro X (camera frame) [rad/s]
  double imu_wy_filtered_ = 0.0;     // EMA-filtered gyro Y [rad/s]
  double imu_wz_filtered_ = 0.0;     // EMA-filtered gyro Z [rad/s]
  double prev_gimbal_yaw_ = 0.0;     // Previous gimbal yaw (for finite-difference)
  double prev_gimbal_pitch_ = 0.0;   // Previous gimbal pitch
  rclcpp::Time prev_imu_time_;       // Timestamp of previous IMU message
  bool imu_initialized_ = false;     // First IMU message received
  bool imu_active_ = false;          // IMU data is being received
  rclcpp::Time last_imu_time_;       // Timestamp of most recent IMU message

  // IMU tuning parameters
  bool enable_imu_compensation_ = true;  // Master enable for chassis compensation
  double imu_gyro_alpha_ = 0.3;          // EMA weight for gyro low-pass filter
  double imu_timeout_ = 0.1;             // Staleness threshold for IMU data [s]
  // Gyro bias estimation: corrects IMU drift during low-motion periods
  double gyro_bias_wz_ = 0.0;
  double gyro_bias_wy_ = 0.0;
  double gyro_bias_alpha_;               // Slow EMA weight (default 0.005)
  double gyro_stationary_threshold_;     // Low-motion threshold [rad/s] (default 0.03)
  int gyro_stationary_count_ = 0;        // Consecutive low-motion frames
  int gyro_stationary_min_frames_;       // Frames before bias updates begin (default 30)

  // Visualization marker publisher
  visualization_msgs::msg::Marker position_marker_;
  visualization_msgs::msg::Marker linear_v_marker_;
  visualization_msgs::msg::Marker angular_v_marker_;
  visualization_msgs::msg::Marker armor_marker_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_
