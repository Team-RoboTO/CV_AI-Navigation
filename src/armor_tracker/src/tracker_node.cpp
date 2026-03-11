#include "armor_tracker/tracker_node.hpp"

#include "armor_tracker/armor.hpp"

// STD
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <opencv2/calib3d.hpp>

namespace rm_auto_aim
{
ArmorTrackerNode::ArmorTrackerNode(const rclcpp::NodeOptions & options)
: Node("armor_tracker", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting TrackerNode!");

  // Maximum allowable armor distance in the XOY plane
  max_armor_distance_ = this->declare_parameter("max_armor_distance", 10.0);

  // YOLO class IDs to track (0=blue, 1=grey/dead, 2=purple, 3=red)
  // Default: {"3"} = red only. Grey ("1") is always removed regardless of config.
  auto target_classes_vec = this->declare_parameter<std::vector<std::string>>(
    "target_classes", std::vector<std::string>{"3"});
  target_classes_ = std::set<std::string>(target_classes_vec.begin(), target_classes_vec.end());
  target_classes_.erase("1");  // grey/dead — never track
  std::string classes_str;
  for (const auto & c : target_classes_) {
    if (!classes_str.empty()) classes_str += ", ";
    classes_str += c;
  }
  RCLCPP_INFO(this->get_logger(),
    "Tracking %zu class(es): %s", target_classes_.size(), classes_str.c_str());

  // Ratio to scale YOLO bbox inward to approximate light-bar positions.
  // WHY SCALE: YOLO bounding boxes encompass the full visible armor plate
  // (including the dark body between light bars), but PnP requires the 4
  // light-bar CORNER positions.  Scaling inward by ~85% maps the bbox edges
  // to the actual light-bar locations.  The exact ratio depends on armor
  // geometry (SMALL vs LARGE plates).  0.85 is a compromise that works for both.
  light_ratio_ = this->declare_parameter("light_ratio", 0.85);

  // Letterbox padding: when DNN encoder pads image to square with keep_aspect_ratio,
  // YOLO bbox Y coordinates are shifted by (network_h - image_h) / 2 pixels.
  // Default 80 = (640 - 480) / 2 for 640x480 input padded to 640x640.
  bbox_padding_y_ = this->declare_parameter("bbox_padding_y", 80.0);

  // Tracker parameters (stored for createTracker factory)
  // - max_match_distance_: Euclidean gate (m) between predicted center and detection; blocks far mis-associations.
  // - max_track_range_: drop detections outside this planar radius (m) before tracker init/update.
  // - tracking_thres_: consecutive good updates required to promote DETECTING → TRACKING.
  // - lost_time_thres_: seconds tolerated without measurement before a tracker is considered TEMP_LOST/LOST.
  // - initial_r1_ / initial_r2_: initial inner/outer ring radii (m) used by yaw solver for 4-armor robots.
  // - r_kf_*: process (Q), measurement (R), and initial (P) covariance for the radius Kalman filter.
  // - r_adapt_max_dist_: maximum allowed jump (m) when adapting armor ring radius frame-to-frame.
  // - dz_adapt_alpha_: EMA factor for smoothing vertical drift compensation when armors hop faces.
  // - r_yaw_uncertainty_scale_: scales yaw covariance when radius estimate is poor (guards against overconfidence).
  // - v_yaw_max_: clamp for yaw rate (rad/s) to prevent numerical blow-ups on bad detections.
  // - maha_match_threshold_ / maha_jump_threshold_: Mahalanobis gates for normal matching vs armor-jump association.
  // - tracker_micro_pose_timeout_: max age (s) of gimbal feedback before warning and reusing stale pose.
  // - max_trackers_: upper bound on simultaneous tracks to keep CPU predictable.
  // - new_tracker_min_dist_: minimum 3D spacing (m) to spawn a new tracker near an existing one.
  max_match_distance_ = this->declare_parameter("tracker.max_match_distance", 0.15);
  max_track_range_ = this->declare_parameter("tracker.max_track_range", 6.0);
  tracking_thres_ = this->declare_parameter("tracker.tracking_thres", 5);
  lost_time_thres_ = this->declare_parameter("tracker.lost_time_thres", 0.3);
  initial_r1_         = this->declare_parameter("tracker.initial_r1", 0.22);
  initial_r2_         = this->declare_parameter("tracker.initial_r2", 0.30);
  r_kf_Q_             = this->declare_parameter("tracker.r_kf_Q", 3.3e-8);
  r_kf_R_             = this->declare_parameter("tracker.r_kf_R", 0.0004);
  r_kf_P_init_        = this->declare_parameter("tracker.r_kf_P_init", 0.0064);
  r_adapt_max_dist_   = this->declare_parameter("tracker.r_adapt_max_dist", 4.0);
  dz_adapt_alpha_     = this->declare_parameter("tracker.dz_adapt_alpha", 0.05);
  r_yaw_uncertainty_scale_ = this->declare_parameter("tracker.r_yaw_uncertainty_scale", 50.0);
  v_yaw_max_          = this->declare_parameter("tracker.v_yaw_max", 15.0);
  maha_match_threshold_ = this->declare_parameter("tracker.maha_match_threshold", 13.3);
  maha_jump_threshold_  = this->declare_parameter("tracker.maha_jump_threshold", 20.0);
  tracker_micro_pose_timeout_ = this->declare_parameter("tracker.micro_pose_timeout", 0.2);
  max_trackers_       = this->declare_parameter("tracker.max_trackers", 5);
  new_tracker_min_dist_ = this->declare_parameter("tracker.new_tracker_min_dist", 0.5);

  // EKF parameters
  xyz_damping_alpha_base_ = declare_parameter("ekf.xyz_damping_alpha", 0.95);
  yaw_damping_alpha_base_ = declare_parameter("ekf.yaw_damping_alpha", 0.95);
  // coast_damping_factor_: extra decay multiplier applied when coasting without
  // measurements (TEMP_LOST) or when we intentionally brake yaw; <1.0 increases
  // damping so velocity dies out faster during occlusion.
  coast_damping_factor_   = declare_parameter("ekf.coast_damping_factor", 0.85);
  damping_innov_threshold_ = declare_parameter("ekf.damping_innov_threshold", 0.10);
  yaw_innov_threshold_    = declare_parameter("ekf.yaw_innov_threshold", 0.15);
  xyz_damping_alpha_ = xyz_damping_alpha_base_;
  yaw_damping_alpha_ = yaw_damping_alpha_base_;
  s2qxyz_ = declare_parameter("ekf.sigma2_q_xyz", 5.0);
  s2qyaw_ = declare_parameter("ekf.sigma2_q_yaw", 10.0);
  s2qr_   = declare_parameter("ekf.sigma2_q_r",   1e-6);
  r_xyz_base_  = declare_parameter("ekf.r_xyz_base",  0.04);
  r_xyz_slope_ = declare_parameter("ekf.r_xyz_slope", 0.03);
  r_yaw_base_  = declare_parameter("ekf.r_yaw_base",  0.05);
  r_yaw_slope_ = declare_parameter("ekf.r_yaw_slope", 0.002);
  r_yaw_angle_power_    = declare_parameter("ekf.r_yaw_angle_power",    4.0);
  max_yaw_oblique_deg_  = declare_parameter("ekf.max_yaw_oblique_deg",  65.0);

  // Reset tracker service
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  reset_tracker_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "/tracker/reset", [this](
                        const std_srvs::srv::Trigger::Request::SharedPtr,
                        std_srvs::srv::Trigger::Response::SharedPtr response) {
      trackers_.clear();
      tracker_last_meas_times_.clear();
      best_tracker_id_ = -1;
      response->success = true;
      RCLCPP_INFO(this->get_logger(), "All trackers reset!");
      return;
    });

  // TF2 transform infrastructure.
  // WHY TF2: armor PnP poses are in camera frame, but the EKF tracks in odom frame.
  // TF2 provides the camera→odom transform (via gimbal + chassis rotation).
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  // Timer interface must be set before any waitForTransform call to avoid
  // tf2_ros::CreateTimerInterfaceException (ROS 2 Humble requirement).
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  // WHY "odom" as target frame: all EKF states must be in a fixed world frame.
  // "odom" is the standard ROS convention for a world-fixed frame with no
  // drift guarantee (unlike "map" which requires SLAM).  Since we broadcast
  // odom→camera TF ourselves, odom IS our world frame.
  target_frame_ = this->declare_parameter("target_frame", "odom");
  armors_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
    "/detector/armors", rclcpp::SensorDataQoS(),
    std::bind(&ArmorTrackerNode::armorsCallback, this, std::placeholders::_1));

  // Camera intrinsics: needed for PnP (2D→3D conversion).
  // WHY reset() after first message: camera intrinsics never change at runtime
  // (fixed lens), so we only need ONE message.  Keeping the subscription alive
  // wastes bandwidth and triggers unnecessary callbacks.
  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera_info", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
      cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
      cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
      pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
      cam_info_sub_.reset();  // unsubscribe — intrinsics won't change
    });

  // Measurement publisher (for debug usage)
  info_pub_ = this->create_publisher<auto_aim_interfaces::msg::TrackerInfo>("/tracker/info", 10);

  // Publisher — RELIABLE + SensorDataQoS.
  // WHY RELIABLE: the trajectory solver publishes a gimbal command for every
  // target update.  If the underlying DDS drops a message (BEST_EFFORT allows
  // this under CPU load), the trajectory solver skips a frame and the fire
  // gate may miss a valid shot window.  RELIABLE ensures delivery even at
  // the cost of slight latency (acceptable here — target rate is ~30 Hz).
  // WHY SensorDataQoS base: gives us keep_last depth, volatile durability,
  // and deadline/liveliness defaults suited for periodic sensor-like data.
  auto target_qos = rclcpp::SensorDataQoS()
    .reliability(rclcpp::ReliabilityPolicy::Reliable);
  target_pub_ = this->create_publisher<auto_aim_interfaces::msg::Target>(
    "/tracker/target", target_qos);

  // Multi-target publisher
  targets_pub_ = this->create_publisher<auto_aim_interfaces::msg::Targets>(
    "/tracker/targets", target_qos);

  // Optimal BBox Publisher
  optimal_bbox_pub_ = this->create_publisher<vision_msgs::msg::Detection2D>(
    "/detections_output/optimal_target", rclcpp::SensorDataQoS());

  // Visualization Marker Publisher
  // See http://wiki.ros.org/rviz/DisplayTypes/Marker
  position_marker_.ns = "position";
  position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  position_marker_.pose.orientation.w = 1.0;
  position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
  position_marker_.color.a = 1.0;
  position_marker_.color.g = 1.0;
  linear_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  linear_v_marker_.ns = "linear_v";
  linear_v_marker_.scale.x = 0.03;
  linear_v_marker_.scale.y = 0.05;
  linear_v_marker_.color.a = 1.0;
  linear_v_marker_.color.r = 1.0;
  linear_v_marker_.color.g = 1.0;
  angular_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  angular_v_marker_.ns = "angular_v";
  angular_v_marker_.scale.x = 0.03;
  angular_v_marker_.scale.y = 0.05;
  angular_v_marker_.color.a = 1.0;
  angular_v_marker_.color.b = 1.0;
  angular_v_marker_.color.g = 1.0;
  armor_marker_.ns = "armors";
  armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
  armor_marker_.scale.x = 0.03;
  armor_marker_.scale.z = 0.125;
  armor_marker_.color.a = 1.0;
  armor_marker_.color.r = 1.0;
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/tracker/marker", 10);

  // Dynamic TF from gimbal feedback
  tf2_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  gimbal_yaw_ = 0.0;
  gimbal_pitch_ = 0.0;
  gimbal_height_ = declare_parameter("gimbal.height", 0.5);
  // WHY sign parameters: different robot builds wire the gimbal encoder
  // with different polarity conventions.  Some STM32 firmwares report pitch
  // as negative-up, others positive-up.  Rather than reflashing firmware,
  // a sign flip in software adapts to the convention.  +1.0 = no flip.
  yaw_sign_      = declare_parameter("gimbal.yaw_sign", 1.0);
  pitch_sign_    = declare_parameter("gimbal.pitch_sign", 1.0);
  micro_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/micro_pose", rclcpp::SensorDataQoS(),
    std::bind(&ArmorTrackerNode::microPoseCallback, this, std::placeholders::_1));

  // IMU-based chassis rotation compensation
  enable_imu_compensation_ = declare_parameter("imu.enable", true);
  imu_gyro_alpha_ = declare_parameter("imu.gyro_alpha", 0.3);
  imu_timeout_ = declare_parameter("imu.timeout", 0.1);

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu", rclcpp::SensorDataQoS(),
    std::bind(&ArmorTrackerNode::imuCallback, this, std::placeholders::_1));
}



void ArmorTrackerNode::microPoseCallback(
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  // Extract gimbal angles from the lower computer.
  // cmd_vel_subscriber stores: position.x = -pitch, position.y = -yaw (radians)
  double p = pitch_sign_ * msg->pose.position.x;
  double y = yaw_sign_ * msg->pose.position.y;
  // Reject garbage serial data — gimbal angles physically can't exceed ~90°
  if (std::abs(p) < 1.6 && std::abs(y) < 1.6) {
    gimbal_yaw_   = y;
    gimbal_pitch_ = p;
  }
  last_micro_pose_time_ = this->now();
  broadcastGimbalTF(msg->header.stamp);
}

void ArmorTrackerNode::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  if (!enable_imu_compensation_) {
    return;
  }

  rclcpp::Time now = msg->header.stamp;
  last_imu_time_ = now;

  if (!imu_initialized_) {
    prev_imu_time_ = now;
    prev_gimbal_yaw_ = gimbal_yaw_;
    prev_gimbal_pitch_ = gimbal_pitch_;
    imu_initialized_ = true;
    imu_active_ = true;
    return;
  }

  double dt = (now - prev_imu_time_).seconds();
  if (dt <= 0.0 || dt > 0.5) {
    prev_imu_time_ = now;
    prev_gimbal_yaw_ = gimbal_yaw_;
    prev_gimbal_pitch_ = gimbal_pitch_;
    return;
  }

  // -----------------------------------------------------------------------
  // IMU-based chassis rotation estimation.
  //
  // PROBLEM: The camera is mounted on a gimbal that rotates independently
  // of the chassis.  When the CHASSIS rotates (robot driving/turning), the
  // entire world shifts in the camera's view.  Without compensation, the
  // EKF sees this as all targets suddenly moving — corrupting velocity
  // estimates and causing false armor jumps.
  //
  // SOLUTION: Use the D455's onboard IMU (which rotates WITH the camera) to
  // measure the camera's total angular velocity.  Then subtract the gimbal's
  // own rotation (known from /micro_pose feedback) to isolate the chassis
  // rotation.  This chassis rotation is applied to the TF transform so the
  // odom frame stays fixed in the world, not on the chassis.
  //
  // Flow: IMU gyro (camera frame) → rotate to chassis frame → subtract
  //       gimbal velocity → integrate → chassis yaw/pitch
  // -----------------------------------------------------------------------

  // EMA low-pass filter on raw gyro to suppress vibration noise.
  // D455 IMU axes: X-right, Y-down, Z-forward (camera_color_optical_frame).
  imu_wx_filtered_ = imu_gyro_alpha_ * msg->angular_velocity.x + (1.0 - imu_gyro_alpha_) * imu_wx_filtered_;
  imu_wy_filtered_ = imu_gyro_alpha_ * msg->angular_velocity.y + (1.0 - imu_gyro_alpha_) * imu_wy_filtered_;
  imu_wz_filtered_ = imu_gyro_alpha_ * msg->angular_velocity.z + (1.0 - imu_gyro_alpha_) * imu_wz_filtered_;

  // Build the rotation from camera optical frame → chassis frame.
  // This chains the gimbal joint angles + the optical frame convention:
  //   q_c2chassis = R_z(gimbal_yaw) × R_y(gimbal_pitch) × R_convention
  tf2::Quaternion q_yaw, q_pitch, q_convention;
  q_yaw.setRPY(0, 0, gimbal_yaw_);                    // Gimbal yaw around Z
  q_pitch.setRPY(0, gimbal_pitch_, 0);                // Gimbal pitch around Y
  q_convention.setRPY(-M_PI / 2.0, 0, -M_PI / 2.0);  // Camera optical → robotics convention

  tf2::Quaternion q_c2chassis = q_yaw * q_pitch * q_convention;
  q_c2chassis.normalize();

  // Rotate the IMU angular velocity from camera frame into chassis frame
  tf2::Vector3 omega_c_in_c(imu_wx_filtered_, imu_wy_filtered_, imu_wz_filtered_);
  tf2::Vector3 omega_c_in_chassis = tf2::quatRotate(q_c2chassis, omega_c_in_c);

  // Estimate gimbal's own angular velocity from finite differences of
  // gimbal feedback angles.  The gimbal yaws around chassis Z, pitches around Y.
  double gimbal_wz = (gimbal_yaw_ - prev_gimbal_yaw_) / dt;
  double gimbal_wy = (gimbal_pitch_ - prev_gimbal_pitch_) / dt;

  // Isolate chassis angular velocity:
  //   omega_total = omega_chassis + omega_gimbal
  //   omega_chassis = omega_total − omega_gimbal
  double chassis_wy = omega_c_in_chassis.y() - gimbal_wy;
  double chassis_wz = omega_c_in_chassis.z() - gimbal_wz;

  // Euler integration of chassis rotation.
  // Accumulates over time — note: no drift correction, so this will
  // slowly accumulate error over minutes of operation.
  chassis_yaw_ += chassis_wz * dt;
  chassis_pitch_ += chassis_wy * dt;

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
      "IMU chassis_yaw=%.4f chassis_pitch=%.4f gimbal_yaw=%.4f gimbal_pitch=%.4f wz=%.4f wy=%.4f",
      chassis_yaw_, chassis_pitch_, gimbal_yaw_, gimbal_pitch_, chassis_wz, chassis_wy);

  imu_active_ = true;
  prev_imu_time_ = now;
  prev_gimbal_yaw_ = gimbal_yaw_;
  prev_gimbal_pitch_ = gimbal_pitch_;
}

// ---------------------------------------------------------------------------
// broadcastGimbalTF — publish the transform from "odom" to "camera_color_optical_frame".
//
// This transform tells TF2 where the camera is in world coordinates so that
// armor positions measured in camera frame can be converted to odom frame.
//
// Decomposition (right-to-left multiplication order):
//
//   T(odom → camera) = Translate(0, 0, gimbal_height)   ← vertical offset
//     × R_z(chassis_yaw) × R_y(chassis_pitch)           ← chassis ego-motion (from IMU)
//     × R_z(gimbal_yaw) × R_y(gimbal_pitch)             ← gimbal joints (from lower computer)
//     × R_convention                                     ← optical frame convention
//
// R_convention = RPY(−π/2, 0, −π/2) converts from the ROS camera_color_optical_frame
// convention (X-right, Y-down, Z-forward) to the robotics convention (X-forward,
// Y-left, Z-up).  When both gimbal angles are 0, this matches the old static TF.
//
// The transform is written directly into tf2_buffer_ (via setTransform) so
// that lookupTransform() later in the same callback sees it immediately,
// even before the /tf topic message is received by other subscribers.
// ---------------------------------------------------------------------------
void ArmorTrackerNode::broadcastGimbalTF(const rclcpp::Time & stamp)
{
  // Purpose: keep odom as a world-fixed frame by publishing the live camera pose
  // derived from chassis rotation (IMU) + gimbal joints + fixed optical offset.
  // Without this, every gimbal movement would look like the target jumped in 3D.
  // Consequences of skipping it: TF2 would reuse a stale camera pose, so after
  // each gimbal pan/tilt the same physical robot would project to a different
  // place in odom. The tracker would see fake target motion, inflate Mahalanobis
  // distances, and either miss associations or spawn ghost tracks.

  // Check gimbal pose staleness
  if (last_micro_pose_time_.nanoseconds() > 0) {
    double micro_staleness = (this->now() - last_micro_pose_time_).seconds();
    if (micro_staleness > tracker_micro_pose_timeout_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Gimbal pose stale (%.3fs > %.3fs), using last-known angles",
        micro_staleness, tracker_micro_pose_timeout_);
    }
  }

  // Chassis rotation from IMU (identity if IMU inactive/stale)
  tf2::Quaternion q_chassis_yaw, q_chassis_pitch;
  q_chassis_yaw.setRPY(0, 0, 0);
  q_chassis_pitch.setRPY(0, 0, 0);

  if (enable_imu_compensation_ && imu_active_) {
    double staleness = (stamp - last_imu_time_).seconds();
    if (staleness < imu_timeout_) {
      q_chassis_yaw.setRPY(0, 0, chassis_yaw_);
      q_chassis_pitch.setRPY(0, chassis_pitch_, 0);
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "IMU data stale (%.3fs > %.3fs), falling back to gimbal-only TF",
        staleness, imu_timeout_);
    }
  }

  // Gimbal rotation from lower computer feedback
  tf2::Quaternion q_yaw, q_pitch, q_convention;
  q_yaw.setRPY(0, 0, gimbal_yaw_);
  q_pitch.setRPY(0, gimbal_pitch_, 0);
  // R_convention = RPY(-π/2, 0, -π/2) matches the old static TF when yaw=pitch=0
  q_convention.setRPY(-M_PI / 2.0, 0, -M_PI / 2.0);

  // Compose in order: chassis (IMU) → gimbal → optical-frame convention.
  tf2::Quaternion q_final = q_chassis_yaw * q_chassis_pitch * q_yaw * q_pitch * q_convention;
  q_final.normalize();

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = stamp;
  t.header.frame_id = "odom";
  t.child_frame_id = "camera_color_optical_frame";
  t.transform.translation.x = 0.0;
  t.transform.translation.y = 0.0;
  t.transform.translation.z = gimbal_height_;
  t.transform.rotation = tf2::toMsg(q_final);

  // Write directly into our own buffer so the transform is available immediately
  // for the lookupTransform() call later in this same callback.
  // The broadcast still publishes on /tf for other nodes (e.g. rviz, tf2_echo).
  tf2_buffer_->setTransform(t, "armor_tracker");
  tf2_broadcaster_->sendTransform(t);
}

// ---------------------------------------------------------------------------
// armorsCallback — main pipeline entry point, called once per YOLO detection frame.
//
// Processing steps:
//   1. Broadcast gimbal TF (odom → camera) so TF2 can transform this frame.
//   2. For each YOLO bounding box:
//      a. Subtract letterbox padding (YOLO pads images to square).
//      b. Filter by YOLO class ID (only track configured enemy color).
//      c. Scale bbox inward by light_ratio_ to approximate light-bar corners.
//      d. Run PnP solver → 3D position + orientation in camera frame.
//   3. Transform all armor poses from camera frame → odom frame via TF2.
//   4. Filter out armors with physically impossible Z or beyond max range.
//   5. Feed the Armors message into the Tracker (init or update).
//   6. Publish Target msg, TrackerInfo debug msg, optimal bbox, and RViz markers.
// ---------------------------------------------------------------------------
void ArmorTrackerNode::armorsCallback(
  const vision_msgs::msg::Detection2DArray::ConstSharedPtr detection_msg)
{
  // Always broadcast gimbal TF so this frame uses a fresh odom→camera pose.
  // WHY: PnP outputs armor poses in camera frame; the tracker works in odom.
  // A correct odom→camera transform is required to place every detection in
  // world coordinates before gating/association. If we skip this, a gimbal pan
  // shifts the camera but TF2 thinks the camera is still at its old pose—so a
  // stationary robot appears to jump, inflates Mahalanobis distance, and either
  // gets dropped or spawns ghost tracks. If /micro_pose is live, angles refresh
  // at 100 Hz; otherwise we reuse last-known angles to keep TF continuous (falls
  // back to static at startup).

  
  // TF primer: TF/TF2 is ROS's frame tree; broadcasting a transform inserts the
  // current odom→camera pose into that tree so lookupTransform can convert PnP
  // results (camera frame) into odom immediately. Without broadcasting, TF2 would
  // reuse stale transforms and every gimbal move would be misinterpreted as target
  // motion. "Broadcast" here means publish to /tf for others AND write into our
  // local buffer for same-callback lookups.

  // Frames here: "odom" = world-fixed frame we maintain with IMU compensation;
  // "camera_color_optical_frame" = RealSense optical frame (x-right, y-down, z-forward)
  // attached to the moving gimbal. We must transform detections from camera frame
  // into odom to compare across time as the gimbal and chassis move.
  broadcastGimbalTF(detection_msg->header.stamp);

  // Convert 2D detections to Armors message
  auto_aim_interfaces::msg::Armors armors_msg;
  armors_msg.header = detection_msg->header;

  if (armors_msg.header.frame_id.empty()) {
    armors_msg.header.frame_id = "camera_color_optical_frame";
  }
  
  std::vector<size_t> detection_indices;

  if (pnp_solver_ == nullptr) {
    return;
  }

  size_t detection_idx = 0;
  for (const auto & detection : detection_msg->detections) {
    auto_aim_interfaces::msg::Armor armor_msg;
    // Extract bbox (decoder outputs coordinates in padded network space)
    auto center_x = detection.bbox.center.position.x;
    auto center_y = detection.bbox.center.position.y - bbox_padding_y_;
    auto width = detection.bbox.size_x;
    auto height = detection.bbox.size_y;

    // Skip degenerate detections (avoids divide-by-zero below)
    if (height < 1.0 || width < 1.0) {
      detection_idx++;
      continue;
    }

    // Filter by YOLO class ID — skip teammates, dead robots, etc.
    if (!detection.results.empty()) {
      const auto & class_id = detection.results[0].hypothesis.class_id;
      if (target_classes_.find(class_id) == target_classes_.end()) {
        detection_idx++;
        continue;
      }
    }

    // Build an Armor object from the YOLO bbox for PnP solving.
    // WHY PnP FROM BBOXES?  YOLO gives us 2D bounding boxes, but the EKF
    // needs 3D positions.  PnP (Perspective-n-Point) recovers the 3D pose
    // from 2D-3D point correspondences.  We approximate the 4 light-bar
    // corners from the bbox, then PnP finds the rotation + translation
    // that best maps the known real-world armor dimensions to those 2D points.
    Armor armor_obj;
    armor_obj.center = cv::Point2f(center_x, center_y);

    // Heuristic for Armor Type (plate aspect: small~1.12, large~1.85)
    float ratio = width / height;
    armor_obj.type = (ratio > 1.5) ? ArmorType::LARGE : ArmorType::SMALL;
    armor_obj.number = detection.results.empty() ? "unknown" : detection.results[0].hypothesis.class_id;
    armor_obj.confidence = detection.results.empty() ? 0.0 : detection.results[0].hypothesis.score;
    // Map class_id to type if possible, but ratio heuristic is good fallback
    // Set classification result for debug
    armor_obj.classfication_result = armor_obj.number;

    // Scale bbox inward by light_ratio to approximate light-bar corners.
    // YOLO bbox covers the full armor plate, which is wider than the light bars.
    const float lr = static_cast<float>(light_ratio_);
    const float lx = center_x - lr * width / 2;
    const float rx = center_x + lr * width / 2;
    const float ty = center_y - lr * height / 2;
    const float by = center_y + lr * height / 2;

    armor_obj.left_light.center = cv::Point2f(lx, center_y);
    armor_obj.left_light.top    = cv::Point2f(lx, ty);
    armor_obj.left_light.bottom = cv::Point2f(lx, by);
    armor_obj.right_light.center = cv::Point2f(rx, center_y);
    armor_obj.right_light.top   = cv::Point2f(rx, ty);
    armor_obj.right_light.bottom = cv::Point2f(rx, by);

    // Solve PnP: turn 2D light corners into a 3D pose relative to the camera.
    // rvec: axis-angle rotation (Rodrigues form: a 3D vector whose direction is the
    //       rotation axis and whose length is the rotation angle in radians) telling
    //       how the armor is oriented.
    // tvec: position of the armor origin measured from the camera (meters) in the
    //       camera frame (x: right, y: down, z: forward for RealSense optical).
    cv::Mat rvec, tvec;
    bool success = pnp_solver_->solvePnP(armor_obj, rvec, tvec);
    if (success) {
      armor_msg.type = ARMOR_TYPE_STR[static_cast<int>(armor_obj.type)];
      armor_msg.number = armor_obj.number;

      // Fill pose
      armor_msg.pose.position.x = tvec.at<double>(0);
      armor_msg.pose.position.y = tvec.at<double>(1);
      armor_msg.pose.position.z = tvec.at<double>(2);

      // Convert rvec to a quaternion for ROS/TF2 message compatibility.
      cv::Mat rotation_matrix;
      cv::Rodrigues(rvec, rotation_matrix);  // Rodrigues vector → 3x3 rotation matrix
      tf2::Matrix3x3 tf2_rotation_matrix(
        rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
        rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0),
        rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
        rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
        rotation_matrix.at<double>(2, 2));
      tf2::Quaternion tf2_q;
      tf2_rotation_matrix.getRotation(tf2_q);  // matrix → quaternion for TF2/ROS msgs
      // TF2/ROS consumers expect quaternions: Pose/Transform messages and
      // tf2::doTransform all operate on (x,y,z,w), not Rodrigues vectors or raw
      // matrices. Converting here lets downstream transforms and publishers work.
      double q_len = tf2_q.length();
      // WHY THIS CHECK: PnP can fail silently (returns success=true but garbage
      // rotation) when the input points are nearly coplanar or extremely noisy.
      // A degenerate rotation matrix produces a zero-length or NaN quaternion.
      // Feeding this into TF2 transforms would propagate NaN to all downstream
      // states, crashing the EKF.
      if (!std::isfinite(q_len) || q_len < 1e-6) {
        detection_idx++;
        continue;  // skip degenerate PnP
      }
      tf2_q.normalize();
      armor_msg.pose.orientation = tf2::toMsg(tf2_q);

      armor_msg.distance_to_image_center = pnp_solver_->calculateDistanceToCenter(armor_obj.center);

      armors_msg.armors.emplace_back(armor_msg);
      detection_indices.push_back(detection_idx);
    }
    detection_idx++;
  }

  // Pointer to the converted message
  auto armors_ptr = std::make_shared<auto_aim_interfaces::msg::Armors>(armors_msg);

  // Transform armor positions from camera frame to odom (world) frame.
  // WHY: PnP gives positions relative to the camera.  But the EKF tracks the
  // robot in world coordinates (odom frame), so we need to know where the camera
  // is in the world.  The gimbal TF (broadcast above) provides this transform.
  // Without it, every time the gimbal moves, the target would appear to jump
  // in camera frame even though it's stationary in the world.
  if (!armors_ptr->armors.empty()) {
    // lookupTransform grabs the latest odom→camera transform (with a small timeout)
    // so we can re-express each armor pose in odom.
    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
      // target_frame_ is the world frame we track in (default "odom", overridable
      // via parameter). We pull the transform from camera frame into that frame.
      tf_stamped = tf2_buffer_->lookupTransform(
        target_frame_, armors_ptr->header.frame_id,
        armors_ptr->header.stamp, rclcpp::Duration::from_seconds(0.01));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(get_logger(), "TF2 error: %s", ex.what());
      return;
    }
    for (auto & armor : armors_ptr->armors) {
      // Wrap pose in a stamped message and apply the TF so orientation + position
      // move from camera frame into odom frame.
      geometry_msgs::msg::PoseStamped ps_in, ps_out;
      ps_in.header = armors_ptr->header;
      ps_in.pose = armor.pose;
      // tf2::doTransform composes the pose with tf_stamped (camera→target_frame_):
      //   p_world = R * p_cam + t, q_world = q_tf * q_cam.  It rotates then translates
      // the point and left-multiplies the orientation quaternion, producing the armor
      // pose in the world frame. Required so all armors are compared/tracked in one
      // coordinate system even while the camera moves.
      tf2::doTransform(ps_in, ps_out, tf_stamped);
      armor.pose = ps_out.pose;
    }
  }

  // Filter physically impossible detections.
  // WHY: PnP can produce wildly wrong 3D positions from noisy bboxes, especially
  // at frame edges or with partial occlusions.  An armor at Z=2.0m (2 meters
  // above the camera) or 15m away horizontally is obviously wrong — no robot
  // is that tall or that far.  Filtering here prevents these garbage measurements
  // from ever reaching the tracker, where they could corrupt the EKF state.
  auto & armors_vec = armors_ptr->armors;

  // Log yaw for all detections (debug)
  for (size_t i = 0; i < armors_vec.size(); i++) {
    const auto & a = armors_vec[i];
    tf2::Quaternion q;
    tf2::fromMsg(a.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    RCLCPP_DEBUG(get_logger(),
      "Det[%zu] id=%s yaw=%.3f pos=(%.2f, %.2f, %.2f)",
      i, a.number.c_str(), yaw,
      a.pose.position.x, a.pose.position.y, a.pose.position.z);
  }

  for (size_t i = 0; i < armors_vec.size(); /* no increment */) {
    auto & armor = armors_vec[i];
    if (
      std::abs(armor.pose.position.z) > 1.2 ||
      Eigen::Vector2d(armor.pose.position.x, armor.pose.position.y).norm() > max_armor_distance_) {
      // Drop detections that are unrealistically high or far; keep detection_indices
      // in sync so downstream mapping to YOLO detections remains correct.
      armors_vec.erase(armors_vec.begin() + i);
      detection_indices.erase(detection_indices.begin() + i);
    } else {
      i++;
    }
  }

  // Timestamp
  rclcpp::Time time = armors_ptr->header.stamp;

  // Compute timestep dt, clamped to [10ms, 100ms].
  // WHY CLAMP: The EKF process model (Q grows as t⁴) and damping (alpha^(dt/dt_nom))
  // are sensitive to dt.  Without a floor, a <1ms timer glitch produces near-zero Q
  // (filter becomes overconfident) and alpha ≈ 1 (no damping).  Without a ceiling,
  // a long gap (e.g. CPU stall) produces enormous Q (filter forgets everything) and
  // alpha → 0 (velocity zeroed instantly).  Both cause filter divergence.
  if (last_time_.nanoseconds() > 0) {
    dt_ = std::min(std::max((time - last_time_).seconds(), 0.01), 0.10);
  }

  // === STEP 1: Per-tracker adaptive damping + PREDICT ONLY ===
  // Predict all trackers first so we can use their predictions for global
  // data association (which tracker owns which detection).
  for (auto & kv : trackers_) {
    auto & tracker = *kv.second;
    if (tracker.tracker_state == Tracker::LOST) continue;

    tracker.lost_thres = static_cast<int>(lost_time_thres_ / dt_);

    // -----------------------------------------------------------------------
    // Adaptive velocity damping — prevents EKF overshoot oscillation.
    //
    // When a constant-velocity EKF tracks a robot that stops or
    // reverses, the predicted velocity overshoots reality.  The innovation
    // (measurement − prediction) then points OPPOSITE to the velocity,
    // pulling the state back — but the velocity is still large, so next
    // frame it overshoots again.  This creates a ringing/oscillation.
    //
    // SOLUTION: Detect when innovation opposes velocity (dot product < 0),
    // which signals overshoot, and reduce the velocity decay factor (alpha).
    //   alpha = 1.0  → no damping (velocity passes through unchanged)
    //   alpha = 0.95 → gentle decay (5% per frame, referenced to 30Hz)
    //   alpha = 0.8  → strong decay (aggressive braking)
    //
    // Three regimes per state:
    //   TRACKING + overshoot detected → scale alpha based on innovation size
    //   TRACKING + no overshoot       → alpha=1 (trust the velocity)
    //   TRACKING + slow (< 0.3m/s)    → base alpha (default gentle decay)
    //   TEMP_LOST                      → aggressive decay (no measurement
    //                                    to correct, so coast conservatively)
    //   DETECTING / other              → base alpha
    // -----------------------------------------------------------------------
    if (tracker.tracker_state == Tracker::TRACKING) {
      // Build the 3D velocity vector: (v_xc, v_yc, v_za)
      Eigen::Vector3d vel(
        tracker.target_state(1), tracker.target_state(3), tracker.target_state(5));
      double speed = vel.norm();

      // Overshoot test: if position innovation (measurement − prediction) points
      // opposite to the velocity, the EKF overshot.  dot < 0 means they oppose.
      double dot = tracker.info_position_innov.dot(vel);
      bool pos_overshoot = (dot < 0.0) && (speed > 0.3);

      if (pos_overshoot) {
        // Scale damping strength by how large the innovation is relative to
        // the threshold.  Larger innovation → stronger damping (lower alpha).
        double pos_scale = std::min(tracker.info_position_diff / damping_innov_threshold_, 1.0);
        // Interpolate: at scale=0 → alpha=1.0 (no damp), at scale=1 → alpha=base
        tracker.xyz_damping_alpha = 1.0 - pos_scale * (1.0 - xyz_damping_alpha_base_);
      } else if (speed > 0.3) {
        // Moving and NOT overshooting → trust the velocity, don't dampen
        tracker.xyz_damping_alpha = 1.0;
      } else {
        // Nearly stationary → apply baseline damping to prevent drift
        tracker.xyz_damping_alpha = xyz_damping_alpha_base_;
      }

      // Same logic for yaw rate: detect when yaw innovation opposes v_yaw
      double v_yaw = tracker.target_state(7);
      double yaw_innov = tracker.info_yaw_innov_signed;
      // Overshoot: innovation pulls yaw backward while v_yaw pushes forward
      bool overshoot = (yaw_innov * v_yaw < 0) && (std::abs(v_yaw) > 0.5);
      if (overshoot) {
        double yaw_scale = std::min(tracker.info_yaw_diff / yaw_innov_threshold_, 1.0);
        // Allow stronger damping than XYZ: multiply base by coast factor to brake harder
        double min_alpha = yaw_damping_alpha_base_ * coast_damping_factor_;
        tracker.yaw_damping_alpha = 1.0 - yaw_scale * (1.0 - min_alpha);
      } else if (std::abs(v_yaw) > 0.5) {
        // Spinning and NOT overshooting → let v_yaw pass through undamped
        tracker.yaw_damping_alpha = 1.0;
      } else {
        // Nearly not spinning → baseline damping
        tracker.yaw_damping_alpha = yaw_damping_alpha_base_;
      }
    } else if (tracker.tracker_state == Tracker::TEMP_LOST) {
      // No measurement to correct → aggressively decay velocity so the
      // prediction doesn't drift far from reality during occlusion. coast_damping_factor_
      // makes the decay stronger than the baseline.
      tracker.xyz_damping_alpha = xyz_damping_alpha_base_ * coast_damping_factor_;
      tracker.yaw_damping_alpha = yaw_damping_alpha_base_ * coast_damping_factor_;
    } else {
      // DETECTING: default gentle damping
      tracker.xyz_damping_alpha = xyz_damping_alpha_base_;
      tracker.yaw_damping_alpha = yaw_damping_alpha_base_;
    }

    // Predict only — update comes AFTER global assignment (Step 3)
    tracker.predictStep();
  }

  // === STEP 2: Global data association ===
  //
  // PROBLEM: Multiple trackers might want the same detection.  If we let each
  // tracker pick its own best detection independently, two trackers could fuse
  // the SAME armor, creating ghost tracks that mirror each other.
  //
  // SOLUTION: For each detection, find the ONE best tracker (lowest Mahalanobis
  // distance). Mahalanobis uses the tracker covariance, so it naturally gates by
  // the predicted uncertainty (anisotropic ellipsoid) instead of a fixed Euclidean
  // radius; this is more robust to stretched covariance along yaw/range. Each
  // detection goes to at most one tracker. Trackers that receive no detections
  // simply coast on their prediction (TEMP_LOST path).
  //
  // detection_owner[i] tracks which tracker (id) owns detection i; -1 means
  // unassigned. This prevents multiple trackers from fusing the same detection.
  // Unassigned detections may spawn new trackers in Step 4 below.
  std::vector<int> detection_owner(armors_ptr->armors.size(), -1);

  for (size_t i = 0; i < armors_ptr->armors.size(); i++) {
    const auto & armor = armors_ptr->armors[i];
    // Use the most permissive Mahalanobis gate (jump threshold) as the outer
    // bound — any detection beyond this is too far from ALL trackers.
    double best_maha = maha_jump_threshold_;
    int best_tid = -1;

    for (auto & kv : trackers_) {
      auto & tracker = *kv.second;
      // kv.first is tracker_id, kv.second is the shared_ptr<Tracker>
      if (tracker.tracker_state == Tracker::LOST) continue;
      // Class ID must match (e.g. "3" = red).  This prevents cross-team assignment.
      if (armor.number != tracker.tracked_id) continue;

      double maha = tracker.computeAssignmentCost(armor);

      // Penalize immature trackers (DETECTING state):
      //   New trackers have high P (large covariance) → the Mahalanobis
      //   distance to any detection is artificially low (wide acceptance
      //   ellipsoid).  Without the 4x penalty, a DETECTING tracker would
      //   steal detections from an established TRACKING tracker nearby.
      //   The 4x multiplier ensures TRACKING trackers win ties decisively.
      if (tracker.tracker_state == Tracker::DETECTING) {
        maha *= 4.0;
      }
      if (maha < best_maha) {
        best_maha = maha;
        best_tid = kv.first;
      }
    }
    detection_owner[i] = best_tid;
  }

  // === STEP 3: Update each tracker with ONLY its assigned detections ===
  for (auto & kv : trackers_) {
    auto & tracker = *kv.second;
    if (tracker.tracker_state == Tracker::LOST) continue;

    // Build per-tracker armors message with only this tracker's assigned detections
    auto tracker_armors = std::make_shared<auto_aim_interfaces::msg::Armors>();
    tracker_armors->header = armors_ptr->header;
    for (size_t i = 0; i < armors_ptr->armors.size(); i++) {
      if (detection_owner[i] == kv.first) {
        tracker_armors->armors.push_back(armors_ptr->armors[i]);
      }
    }

    tracker.update(tracker_armors);

    if (tracker.measurement_valid) {
      tracker_last_meas_times_[kv.first] = time;
    }
  }

  // === STEP 4: Spawn new trackers for unassigned detections ===
  for (size_t i = 0; i < armors_ptr->armors.size(); i++) {
    if (detection_owner[i] >= 0) continue;  // already assigned to a tracker
    const auto & armor = armors_ptr->armors[i];

    // Check distance to all existing tracker centers (Euclidean, not Mahalanobis,
    // because two robots can have very different yaw → high Mahalanobis even when
    // positionally close — we don't want to spawn a tracker on top of another)
    bool too_close = false;
    for (auto & kv : trackers_) {
      auto & tracker = *kv.second;
      if (tracker.tracker_state == Tracker::LOST) continue;
      double dx = armor.pose.position.x - tracker.target_state(0);
      double dy = armor.pose.position.y - tracker.target_state(2);
      double ddz = armor.pose.position.z - tracker.target_state(4);
      if (std::sqrt(dx * dx + dy * dy + ddz * ddz) < new_tracker_min_dist_) {
        too_close = true;
        break;
      }
    }
    if (too_close) continue;
    if (static_cast<int>(trackers_.size()) >= max_trackers_) continue;

    auto new_tracker = createTracker();
    new_tracker->initFromArmor(armor);
    int tid = next_tracker_id_++;
    new_tracker->tracker_id = tid;
    tracker_last_meas_times_[tid] = time;
    trackers_[tid] = std::move(new_tracker);
    RCLCPP_INFO(this->get_logger(), "New tracker %d for id=%s", tid, armor.number.c_str());
  }

  // === STEP 5: Prune LOST trackers ===
  for (auto it = trackers_.begin(); it != trackers_.end(); ) {
    if (it->second->tracker_state == Tracker::LOST) {
      RCLCPP_INFO(this->get_logger(), "Removing LOST tracker %d", it->first);
      tracker_last_meas_times_.erase(it->first);
      it = trackers_.erase(it);
    } else {
      ++it;
    }
  }

  // === STEP 6: Select best tracker for primary target output ===
  //
  // Among all TRACKING / TEMP_LOST trackers, pick the one most worth shooting.
  // Score = range × state_penalty × (1 + v_yaw_variance)
  //
  //   range:         closer targets are higher priority (lower score is better)
  //   state_penalty: TEMP_LOST targets score 2× worse (uncertain, might be lost)
  //   v_yaw_var:     high spin-rate uncertainty makes firing unreliable
  //                  (trajectory solver doesn't know where the face will be)
  //
  // DETECTING trackers are excluded — they haven't been confirmed yet.
  best_tracker_id_ = -1;
  double best_score = 1e9;
  for (auto & kv : trackers_) {
    auto & tracker = *kv.second;
    if (tracker.tracker_state != Tracker::TRACKING &&
        tracker.tracker_state != Tracker::TEMP_LOST) continue;
    double xc = tracker.target_state(0);
    double yc = tracker.target_state(2);
    double za = tracker.target_state(4);
    double range = std::sqrt(xc * xc + yc * yc + za * za);
    double state_penalty = (tracker.tracker_state == Tracker::TEMP_LOST) ? 2.0 : 1.0;
    double v_yaw_var = tracker.ekf.getVariance(7);  // P_post(7,7)
    double score = range * state_penalty * (1.0 + v_yaw_var);
    if (score < best_score) {
      best_score = score;
      best_tracker_id_ = kv.first;
    }
  }

  // === STEP 7: Publish Targets (all non-LOST trackers) ===
  auto_aim_interfaces::msg::Targets targets_msg;
  targets_msg.header.stamp = time;
  targets_msg.header.frame_id = target_frame_;
  targets_msg.best_target_idx = -1;

  int target_idx = 0;
  for (auto & kv : trackers_) {
    auto & tracker = *kv.second;
    if (tracker.tracker_state == Tracker::LOST) continue;

    auto_aim_interfaces::msg::Target t;
    rclcpp::Time meas_time = tracker_last_meas_times_.count(kv.first) ?
      tracker_last_meas_times_[kv.first] : rclcpp::Time(0, 0, RCL_ROS_TIME);
    fillTargetMsg(t, tracker, time, meas_time);

    targets_msg.targets.push_back(t);
    if (kv.first == best_tracker_id_) {
      targets_msg.best_target_idx = target_idx;
    }
    target_idx++;
  }
  targets_pub_->publish(targets_msg);

  // === STEP 8: Publish legacy single Target (best tracker) ===
  auto_aim_interfaces::msg::Target target_msg;
  target_msg.header.stamp = time;
  target_msg.header.frame_id = target_frame_;
  target_msg.tracking = false;
  target_msg.id = "";
  target_msg.armors_num = 0;

  if (best_tracker_id_ >= 0 && trackers_.count(best_tracker_id_)) {
    auto & best = *trackers_[best_tracker_id_];
    rclcpp::Time meas_time = tracker_last_meas_times_.count(best_tracker_id_) ?
      tracker_last_meas_times_[best_tracker_id_] : rclcpp::Time(0, 0, RCL_ROS_TIME);
    fillTargetMsg(target_msg, best, time, meas_time);

    // Publish debug info for best tracker
    auto_aim_interfaces::msg::TrackerInfo info_msg;
    info_msg.position_diff    = best.info_position_diff;
    info_msg.yaw_diff         = best.info_yaw_diff;
    info_msg.face_angle       = best.info_face_angle;
    if (best.measurement_valid) {
      info_msg.position.x     = best.measurement(0);
      info_msg.position.y     = best.measurement(1);
      info_msg.position.z     = best.measurement(2);
      info_msg.yaw            = best.measurement(3);
    }
    info_msg.xyz_damping_alpha = best.xyz_damping_alpha;
    info_msg.yaw_damping_alpha = best.yaw_damping_alpha;
    info_pub_->publish(info_msg);
  }

  // Publish optimal bbox (H2 fix: bounds check on detection_indices)
  vision_msgs::msg::Detection2D optimal_bbox;
  optimal_bbox.header = detection_msg->header;
  if (target_msg.tracking && best_tracker_id_ >= 0 && trackers_.count(best_tracker_id_)) {
    auto & best = *trackers_[best_tracker_id_];
    size_t matched_idx = static_cast<size_t>(-1);
    double min_dist = 0.2;
    for (size_t i = 0; i < armors_ptr->armors.size(); i++) {
      const auto & armor = armors_ptr->armors[i];
      double dx = armor.pose.position.x - best.tracked_armor.pose.position.x;
      double dy = armor.pose.position.y - best.tracked_armor.pose.position.y;
      double ddz = armor.pose.position.z - best.tracked_armor.pose.position.z;
      double dist = std::sqrt(dx * dx + dy * dy + ddz * ddz);
      if (dist < min_dist) {
        min_dist = dist;
        matched_idx = i;
      }
    }
    if (matched_idx != static_cast<size_t>(-1) && matched_idx < detection_indices.size()) {
      size_t original_detection_idx = detection_indices[matched_idx];
      if (original_detection_idx < detection_msg->detections.size()) {
        optimal_bbox = detection_msg->detections[original_detection_idx];
      }
    }
  }
  optimal_bbox_pub_->publish(optimal_bbox);

  last_time_ = time;

  target_pub_->publish(target_msg);
  publishMarkers(target_msg);
}

std::unique_ptr<Tracker> ArmorTrackerNode::createTracker()
{
  auto tracker = std::make_unique<Tracker>(max_match_distance_, max_track_range_);
  tracker->tracking_thres = tracking_thres_;
  tracker->initial_r1_ = initial_r1_;
  tracker->initial_r2_ = initial_r2_;
  tracker->r_kf_Q_ = r_kf_Q_;
  tracker->r_kf_R_ = r_kf_R_;
  tracker->r_kf_P_init_ = r_kf_P_init_;
  tracker->r_adapt_max_dist_ = r_adapt_max_dist_;
  tracker->dz_adapt_alpha_ = dz_adapt_alpha_;
  tracker->r_yaw_uncertainty_scale_ = r_yaw_uncertainty_scale_;
  tracker->v_yaw_max_ = v_yaw_max_;
  tracker->maha_match_threshold_ = maha_match_threshold_;
  tracker->maha_jump_threshold_ = maha_jump_threshold_;
  tracker->setMaxYawObliqueAngle(max_yaw_oblique_deg_ * M_PI / 180.0);

  // =========================================================================
  // EKF LAMBDA DEFINITIONS — the mathematical heart of the tracker.
  //
  // The EKF needs 6 pluggable functions:
  //   f    — process model:   how the state evolves over time
  //   j_f  — Jacobian of f:   linearized process model (for covariance propagation)
  //   h    — observation model: how to convert state → expected measurement
  //   j_h  — Jacobian of h:   linearized observation model (for Kalman gain)
  //   u_q  — process noise Q: how much uncertainty to add each timestep
  //   u_r  — measurement noise R: how noisy each measurement is
  //
  // All lambdas capture `this` for access to dt_, noise parameters, etc.
  // Damping alphas are read from tracker_ptr (raw pointer to the tracker
  // instance) so each tracker uses its OWN per-frame damping values, not
  // a shared node-level value.
  // =========================================================================
  Tracker * tracker_ptr = tracker.get();

  // -----------------------------------------------------------------------
  // f(x) — PROCESS MODEL: damped constant-velocity prediction.
  //
  // Each position component advances by velocity × dt:
  //   pos_new = pos + v_damped × dt
  //   v_new   = v × damping_factor
  //
  // Damping factor is time-adjusted: b = alpha^(dt / dt_nominal).
  //   At 30Hz (dt=33ms, nominal): b = alpha^1 = alpha
  //   At 60Hz (dt=16ms):          b = alpha^0.5 ≈ sqrt(alpha) — weaker per frame
  //   At 15Hz (dt=66ms):          b = alpha^2 — stronger per frame
  // This keeps the effective decay rate (per second) constant regardless of
  // frame rate.  Without time-adjustment, running at 60Hz would halve the
  // damping compared to 30Hz.
  //
  // State layout: [xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r]
  //                 0    1     2    3     4    5     6     7     8
  // -----------------------------------------------------------------------
  auto f = [this, tracker_ptr](const Eigen::VectorXd & x) {
    Eigen::VectorXd x_new = x;
    // Time-adjusted damping factors (see explanation above)
    double b = std::pow(tracker_ptr->xyz_damping_alpha, dt_ / (1.0 / 30.0));  // XYZ decay
    double a = std::pow(tracker_ptr->yaw_damping_alpha, dt_ / (1.0 / 30.0));  // Yaw decay
    // Apply velocity decay (models friction / resistance)
    double damped_vx   = x(1) * b;    // v_xc after damping
    double damped_vy   = x(3) * b;    // v_yc after damping
    double damped_vz   = x(5) * b;    // v_za after damping
    double damped_vyaw = x(7) * a;    // v_yaw after damping
    // Advance positions by damped velocity × dt
    x_new(0) += damped_vx * dt_;      // xc += v_xc × dt
    x_new(1) = damped_vx;             // v_xc (decayed)
    x_new(2) += damped_vy * dt_;      // yc += v_yc × dt
    x_new(3) = damped_vy;             // v_yc (decayed)
    x_new(4) += damped_vz * dt_;      // za += v_za × dt
    x_new(5) = damped_vz;             // v_za (decayed)
    x_new(6) += damped_vyaw * dt_;    // yaw += v_yaw × dt
    x_new(7) = damped_vyaw;           // v_yaw (decayed)
    // x_new(8) = x(8) — radius is constant (managed by ScalarKF, not the EKF)
    return x_new;
  };

  // -----------------------------------------------------------------------
  // j_f(x) — JACOBIAN OF f: partial derivatives ∂f/∂x.
  //
  // This is the linearized version of f() used by the EKF to propagate
  // the covariance matrix (P_pri = F · P_post · Fᵀ + Q).
  //
  // For the damped constant-velocity model, F is block-diagonal with
  // 2×2 blocks for each (pos, vel) pair, plus a 1×1 block for radius.
  //
  // Each 2×2 block looks like:
  //   [ 1   b·dt ]   where b = damping factor
  //   [ 0   b    ]
  // meaning: pos depends on old pos (1×) plus velocity (b·dt ×),
  //          vel depends only on old vel (b ×, decayed).
  // -----------------------------------------------------------------------
  auto j_f = [this, tracker_ptr](const Eigen::VectorXd &) {
    double b = std::pow(tracker_ptr->xyz_damping_alpha, dt_ / (1.0 / 30.0));
    double a = std::pow(tracker_ptr->yaw_damping_alpha, dt_ / (1.0 / 30.0));
    Eigen::MatrixXd f(9, 9);
    //              xc   v_xc  yc   v_yc  za   v_za  yaw  v_yaw  r
    // clang-format off
    f <<  1,   b*dt_, 0,   0,      0,   0,      0,   0,      0,   // xc
          0,   b,     0,   0,      0,   0,      0,   0,      0,   // v_xc
          0,   0,     1,   b*dt_,  0,   0,      0,   0,      0,   // yc
          0,   0,     0,   b,      0,   0,      0,   0,      0,   // v_yc
          0,   0,     0,   0,      1,   b*dt_,  0,   0,      0,   // za
          0,   0,     0,   0,      0,   b,      0,   0,      0,   // v_za
          0,   0,     0,   0,      0,   0,      1,   a*dt_,  0,   // yaw
          0,   0,     0,   0,      0,   0,      0,   a,      0,   // v_yaw
          0,   0,     0,   0,      0,   0,      0,   0,      1;   // r (constant)
    // clang-format on
    return f;
  };

  // -----------------------------------------------------------------------
  // h(x) — OBSERVATION MODEL: convert robot-center state → expected armor
  //         plate measurement [xa, ya, za, yaw].
  //
  // The EKF tracks the robot CENTER, but we MEASURE the ARMOR PLATE.
  // The armor plate is offset from center by radius r in the yaw direction:
  //
  //   Camera ────→ [Armor xa,ya] ←─ r ─→ (Center xc,yc)
  //                      ← face normal (yaw) ──┘
  //
  //   xa = xc − r·cos(yaw)     (armor is behind center from camera's view)
  //   ya = yc − r·sin(yaw)
  //   za = za                   (height passes through directly)
  //   yaw = yaw                 (heading passes through directly)
  // -----------------------------------------------------------------------
  auto h = [](const Eigen::VectorXd & x) {
    Eigen::VectorXd z(4);
    double xc = x(0), yc = x(2), yaw = x(6), r = x(8);
    z(0) = xc - r * cos(yaw);   // expected armor X position
    z(1) = yc - r * sin(yaw);   // expected armor Y position
    z(2) = x(4);                 // expected armor Z (= za, direct)
    z(3) = x(6);                 // expected yaw (= state yaw, direct)
    return z;
  };

  // -----------------------------------------------------------------------
  // j_h(x) — JACOBIAN OF h: partial derivatives ∂h/∂x.
  //
  // Rows = measurement components [xa, ya, za, yaw]
  // Cols = state components [xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r]
  //
  // Key nonlinear derivatives (from xa = xc − r·cos(yaw)):
  //   ∂xa/∂yaw = r·sin(yaw)    (yaw change moves armor tangentially)
  //   ∂xa/∂r   = −cos(yaw)     (radius change moves armor radially)
  //   ∂ya/∂yaw = −r·cos(yaw)
  //   ∂ya/∂r   = −sin(yaw)
  // All other derivatives are 0 or 1 (linear pass-through).
  // -----------------------------------------------------------------------
  auto j_h = [](const Eigen::VectorXd & x) {
    Eigen::MatrixXd h(4, 9);
    double yaw = x(6), r = x(8);
    //              xc   v_xc  yc   v_yc  za   v_za  yaw           v_yaw  r
    // clang-format off
    h <<  1,   0,   0,   0,   0,   0,   r*sin(yaw),  0,   -cos(yaw),  // ∂xa/∂(state)
          0,   0,   1,   0,   0,   0,  -r*cos(yaw),  0,   -sin(yaw),  // ∂ya/∂(state)
          0,   0,   0,   0,   1,   0,   0,            0,   0,          // ∂za/∂(state)
          0,   0,   0,   0,   0,   0,   1,            0,   0;          // ∂yaw/∂(state)
    // clang-format on
    return h;
  };

  // -----------------------------------------------------------------------
  // u_q() — PROCESS NOISE Q: uncertainty added per prediction step.
  //
  // Models unmodeled accelerations (the robot can change speed/direction).
  // Uses the "continuous white noise acceleration" model:
  //   For a (position, velocity) pair with acceleration variance σ²:
  //     Q_block = σ² × [ t⁴/4   t³/2 ]   where t = dt
  //                     [ t³/2   t²   ]
  //
  // Three noise levels:
  //   s2qxyz_ — for XYZ position/velocity (translational jerk)
  //   s2qyaw_ — for yaw/v_yaw (rotational jerk, typically higher)
  //   s2qr_   — for radius (very small: radius is quasi-static)
  //
  // Block-diagonal structure: XYZ, yaw, and radius are uncorrelated in Q.
  // -----------------------------------------------------------------------
  auto u_q = [this]() {
    Eigen::MatrixXd q(9, 9);
    double t = dt_, x = s2qxyz_, y = s2qyaw_, r = s2qr_;
    // Continuous white noise acceleration model coefficients:
    double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
    double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * y, q_vy_vy = pow(t, 2) * y;
    double q_r = pow(t, 4) / 4 * r;
    //              xc      v_xc    yc      v_yc    za      v_za    yaw     v_yaw   r
    // clang-format off
    q <<  q_x_x,  q_x_vx, 0,      0,      0,      0,      0,      0,      0,      // xc
          q_x_vx, q_vx_vx,0,      0,      0,      0,      0,      0,      0,      // v_xc
          0,      0,      q_x_x,  q_x_vx, 0,      0,      0,      0,      0,      // yc
          0,      0,      q_x_vx, q_vx_vx,0,      0,      0,      0,      0,      // v_yc
          0,      0,      0,      0,      q_x_x,  q_x_vx, 0,      0,      0,      // za
          0,      0,      0,      0,      q_x_vx, q_vx_vx,0,      0,      0,      // v_za
          0,      0,      0,      0,      0,      0,      q_y_y,  q_y_vy, 0,      // yaw
          0,      0,      0,      0,      0,      0,      q_y_vy, q_vy_vy,0,      // v_yaw
          0,      0,      0,      0,      0,      0,      0,      0,      q_r;    // r
    // clang-format on
    return q;
  };

  // -----------------------------------------------------------------------
  // u_r(z) — MEASUREMENT NOISE R: how noisy each measurement component is.
  //
  // R is DYNAMIC — it depends on two factors:
  //
  // 1. RANGE: PnP accuracy degrades with distance (fewer pixels on target).
  //    R_xyz = (base + slope × distance)²
  //    At 1m: (0.04 + 0.03)² = 0.0049 m²  (σ ≈ 7 cm)
  //    At 5m: (0.04 + 0.15)² = 0.036 m²   (σ ≈ 19 cm)
  //
  // 2. OBLIQUITY: When the armor plate is edge-on to the camera, PnP has
  //    very few pixels to work with and orientation is poorly constrained.
  //
  //    face_angle = angle between armor normal and camera bearing
  //      0°  = face-on (best PnP accuracy)
  //      90° = edge-on (PnP yaw is essentially random noise)
  //
  //    XYZ noise inflated by 1/cos²(face_angle):
  //      At 0°:  factor=1    (no penalty)
  //      At 60°: factor=4    (2× σ)
  //      At 80°: factor=33   (6× σ)
  //
  //    Yaw noise inflated by 1/cos^n(face_angle) with higher exponent n:
  //      Beyond max_yaw_oblique_deg_ (~65°), yaw noise → 1e6 (effectively
  //      infinite), so the Kalman gain for yaw → 0 and the bad yaw
  //      measurement is harmlessly ignored by the EKF.
  // -----------------------------------------------------------------------
  auto u_r = [this](const Eigen::VectorXd & z) {
    Eigen::DiagonalMatrix<double, 4> r;
    // Range to target (3D distance from camera at origin)
    double dist = std::sqrt(z[0]*z[0] + z[1]*z[1] + z[2]*z[2]);
    // Base noise standard deviations, scaled by range
    double ps = r_xyz_base_ + r_xyz_slope_ * dist;  // position σ (metres)
    double ys = r_yaw_base_ + r_yaw_slope_ * dist;  // yaw σ (radians)
    // Compute obliquity-dependent noise inflation
    double xyz_angle_factor;
    double yaw_angle_factor;
    if (z[0] * z[0] + z[1] * z[1] < 1e-12) {
      // Target at camera origin (degenerate) — use base noise
      xyz_angle_factor = 1.0;
      yaw_angle_factor = 1.0;
    } else {
      // bearing_opp = direction FROM armor TOWARD camera (opposite of bearing)
      double bearing_opp = std::atan2(-z[1], -z[0]);
      // face_angle = how much the plate is turned away from camera
      // z[3] is yaw (armor normal direction); bearing_opp is camera direction
      // 0 = face-on, π/2 = edge-on
      double face_angle = std::abs(std::remainder(z[3] - bearing_opp, 2.0 * M_PI));
      double cos_fa = std::cos(face_angle);
      // XYZ: inflate by 1/cos² (moderate penalty for oblique views)
      // Floor at 0.04 to prevent infinite noise at exactly 90°
      xyz_angle_factor = 1.0 / std::max(cos_fa * cos_fa, 0.04);
      // Yaw: inflate by 1/cos^n (steeper penalty, tunable exponent)
      double cos_pow = std::pow(std::abs(cos_fa), r_yaw_angle_power_);
      yaw_angle_factor = 1.0 / std::max(cos_pow, 1e-4);
      // Beyond the oblique threshold: yaw is unreliable, set noise to ~infinity
      // so the EKF ignores yaw entirely and only fuses XYZ position
      double max_oblique_rad = max_yaw_oblique_deg_ * M_PI / 180.0;
      if (face_angle > max_oblique_rad) {
        yaw_angle_factor = 1e6;
      }
    }
    // R is diagonal: [R_xa, R_ya, R_za, R_yaw]
    // Each entry = (base_σ)² × obliquity_factor
    r.diagonal() << ps*ps*xyz_angle_factor, ps*ps*xyz_angle_factor,
                     ps*ps*xyz_angle_factor, ys*ys*yaw_angle_factor;
    return r;
  };

  // -----------------------------------------------------------------------
  // P0 — Initial state covariance (diagonal).
  //
  // These values represent our uncertainty about each state variable at
  // tracker initialization (before any measurements are fused).
  //   Position (xc, yc, za) = 0.1 m²  (σ ≈ 0.32 m — PnP accuracy)
  //   Velocity (v_xc, v_yc) = 1.0 m²/s² (σ = 1 m/s — no motion info)
  //   Velocity (v_za)        = 0.2 m²/s² (robots don't move vertically fast)
  //   Yaw                    = 0.1 rad² (σ ≈ 18° — PnP yaw accuracy)
  //   V_yaw                  = 3.0 rad²/s² (σ ≈ 100°/s — could be spinning fast)
  //   Radius                 = 0.001 m² (σ ≈ 3 cm — managed by ScalarKF)
  // -----------------------------------------------------------------------
  Eigen::DiagonalMatrix<double, 9> p0;
  //                   xc   v_xc  yc   v_yc  za   v_za  yaw  v_yaw  r
  p0.diagonal() << 0.1, 1.0, 0.1, 1.0, 0.1, 0.2, 0.1, 3.0, 0.001;
  tracker->ekf = ExtendedKalmanFilter{f, h, j_f, j_h, u_q, u_r, p0};

  // -----------------------------------------------------------------------
  // max_covariance — per-state upper bounds for P diagonal elements.
  //
  // During TEMP_LOST (no measurements), P grows every frame via the Q term.
  // Without clamping, P would explode, and when a detection finally returns,
  // the huge Kalman gain would make the filter snap to a noisy measurement.
  //
  // These bounds limit how uncertain the filter can become:
  //   Position: 1.0 m²      (σ = 1m — beyond this the target is effectively lost)
  //   Velocity: 10.0 m²/s²  (σ ≈ 3 m/s)
  //   V_yaw:    30.0 rad²/s² (σ ≈ 5.5 rad/s — fast spinners need wide bound)
  //   Radius:   0.01 m²     (σ = 10 cm — clamped tighter since ScalarKF manages it)
  // -----------------------------------------------------------------------
  Eigen::VectorXd max_cov(9);
  //              xc    v_xc   yc    v_yc   za    v_za   yaw   v_yaw   r
  max_cov << 1.0, 10.0, 1.0, 10.0, 1.0, 2.0, 1.0, 30.0, 0.01;
  tracker->ekf.max_covariance = max_cov;
  // Condition number limit: if P becomes ill-conditioned (one eigenvalue
  // vastly larger than another), blend it toward P0 to restore stability.
  tracker->ekf.max_condition_number = 1e6;

  return tracker;
}

// ---------------------------------------------------------------------------
// fillTargetMsg — pack a tracker's EKF state into a Target ROS message.
//
// The Target message is the primary interface between the armor_tracker and
// the rm_trajectory (ballistic solver) node.  It carries everything the
// trajectory solver needs to compute a fire solution.
//
// EKF state vector → Target message field mapping:
//   state(0) → position.x   [m]    robot center X in odom frame
//   state(1) → velocity.x   [m/s]  robot center X velocity
//   state(2) → position.y   [m]    robot center Y in odom frame
//   state(3) → velocity.y   [m/s]  robot center Y velocity
//   state(4) → position.z   [m]    armor plate Z (height) in odom frame
//   state(5) → velocity.z   [m/s]  armor plate Z velocity
//   state(6) → yaw          [rad]  robot heading (continuously unwrapped)
//   state(7) → v_yaw        [rad/s] spin rate (positive = CCW)
//   state(8) → radius_1     [m]    orbit radius of the currently-tracked pair
//              radius_2     [m]    orbit radius of the OTHER pair (from r_other_kf_)
//              dz           [m]    height difference between the two armor pairs
//              v_yaw_variance [rad²/s²]  P_post(7,7) — EKF spin-rate uncertainty
//                            (used by trajectory solver to widen fire window)
//
// State machine behavior:
//   DETECTING  → tracking=false  — not yet confirmed; trajectory solver ignores
//   TRACKING   → tracking=true   — fully confirmed; trajectory solver fires
//   TEMP_LOST  → tracking=true   — coasting; trajectory solver suppresses fire
//   LOST       → tracking=false  — removed from the map; never reaches here
// ---------------------------------------------------------------------------
void ArmorTrackerNode::fillTargetMsg(
  auto_aim_interfaces::msg::Target & msg,
  Tracker & tracker,
  const rclcpp::Time & time,
  const rclcpp::Time & last_meas_time)
{
  msg.header.stamp = time;
  msg.header.frame_id = target_frame_;
  msg.tracker_id = tracker.tracker_id;
  // tracker_state as uint8 for the trajectory solver's fire suppression checks
  msg.tracker_state = static_cast<uint8_t>(tracker.tracker_state);

  if (tracker.tracker_state == Tracker::DETECTING) {
    // DETECTING: not yet confirmed (< tracking_thres consecutive matches).
    // Trajectory solver must not fire on an unconfirmed detection.
    msg.tracking = false;
    msg.id = "";
    msg.armors_num = 0;
  } else if (
    tracker.tracker_state == Tracker::TRACKING ||
    tracker.tracker_state == Tracker::TEMP_LOST) {
    // TRACKING / TEMP_LOST: confirmed target — publish the full EKF state.
    // TEMP_LOST is included so the trajectory solver keeps aiming during brief
    // occlusions; it suppresses fire via the tracker_state field check.
    msg.tracking = true;
    const auto & state = tracker.target_state;
    msg.id         = tracker.tracked_id;       // YOLO class ID (e.g. "3" = red)
    msg.armors_num = static_cast<int>(tracker.tracked_armors_num);  // 2, 3, or 4
    // Robot center position (odom frame):
    msg.position.x = state(0);   // xc
    msg.velocity.x = state(1);   // v_xc
    msg.position.y = state(2);   // yc
    msg.velocity.y = state(3);   // v_yc
    // Armor plate height (odom frame):
    msg.position.z = state(4);   // za  (height of currently tracked armor)
    msg.velocity.z = state(5);   // v_za
    // Robot orientation (continuously unwrapped from PnP yaw):
    msg.yaw        = state(6);   // heading [rad], unwrapped (can exceed ±π)
    msg.v_yaw      = state(7);   // spin rate [rad/s]; sign = direction of spin
    // Orbital geometry:
    msg.radius_1   = state(8);         // active pair radius (from ScalarKF)
    msg.radius_2   = tracker.another_r; // other pair radius (from r_other_kf_)
    msg.dz         = tracker.dz;       // height difference between armor pairs [m]
    // EKF covariance — used by trajectory solver to scale fire window:
    msg.v_yaw_variance = tracker.ekf.getVariance(7);  // P_post(7,7)
    // Last real detection timestamp — for trajectory solver's staleness check
    msg.last_measurement_stamp = last_meas_time;
  } else {
    // LOST: this branch should never execute because LOST trackers are pruned
    // before fillTargetMsg is called.  Guard against future logic changes.
    msg.tracking = false;
    msg.id = "";
    msg.armors_num = 0;
  }
}

// ---------------------------------------------------------------------------
// publishMarkers — publish RViz visualization for the best-target tracker.
//
// Published on /tracker/marker as a MarkerArray.  Consumed by RViz to overlay
// the EKF state on the live camera feed for debugging and tuning.
//
// Markers published when tracking=true:
//   position_marker_  (SPHERE, green)
//     Centre of the spinning robot (xc, yc, za + dz/2).
//     Positioned at the midpoint between the two armor-pair heights.
//
//   linear_v_marker_  (ARROW, yellow)
//     Linear velocity of the robot center (vx, vy, vz) as an arrow starting
//     at the center sphere.  Arrow tip = center + velocity → a 1-second
//     look-ahead of the robot's ground-truth motion.
//
//   angular_v_marker_ (ARROW, cyan)
//     Spin rate v_yaw represented as a vertical arrow (Z-direction only).
//     Length = v_yaw / π metres — one full rotation (2π rad/s) → 2m arrow.
//     Upward = CCW spin (positive v_yaw), downward = CW spin.
//
//   armor_marker_     (CUBE, red, repeated n times)
//     One cube per armor plate, positioned at the predicted plate locations:
//       face_i_position = center − r_i × [cos(yaw + i × 2π/n), sin(...)]
//     Scale matches actual plate dimensions (SMALL: 140×125 mm, LARGE: 235×127 mm).
//     Orientation: pitch=±0.26 rad (armor plate tilted slightly outward from
//     the robot center, as in the physical design).
//
// When tracking=false (no target): all markers are sent with DELETE action.
// ---------------------------------------------------------------------------
void ArmorTrackerNode::publishMarkers(const auto_aim_interfaces::msg::Target & target_msg)
{
  position_marker_.header = target_msg.header;
  linear_v_marker_.header = target_msg.header;
  angular_v_marker_.header = target_msg.header;
  armor_marker_.header = target_msg.header;

  visualization_msgs::msg::MarkerArray marker_array;
  if (target_msg.tracking) {
    double yaw = target_msg.yaw, r1 = target_msg.radius_1, r2 = target_msg.radius_2;
    double xc = target_msg.position.x, yc = target_msg.position.y, za = target_msg.position.z;
    double vx = target_msg.velocity.x, vy = target_msg.velocity.y, vz = target_msg.velocity.z;
    double dz = target_msg.dz;

    // Center sphere: positioned at the robot body center, vertically midway
    // between the two armor pair heights (za and za+dz).
    position_marker_.action = visualization_msgs::msg::Marker::ADD;
    position_marker_.pose.position.x = xc;
    position_marker_.pose.position.y = yc;
    position_marker_.pose.position.z = za + dz / 2;

    // Linear velocity arrow: from center to center + velocity.
    // A 1 m/s velocity produces a 1 m arrow (intuitive scale).
    linear_v_marker_.action = visualization_msgs::msg::Marker::ADD;
    linear_v_marker_.points.clear();
    linear_v_marker_.points.emplace_back(position_marker_.pose.position);
    geometry_msgs::msg::Point arrow_end = position_marker_.pose.position;
    arrow_end.x += vx;
    arrow_end.y += vy;
    arrow_end.z += vz;
    linear_v_marker_.points.emplace_back(arrow_end);

    // Spin rate arrow: vertical, length ∝ v_yaw.
    // v_yaw / π means: 1 full revolution/s → 2m arrow.
    angular_v_marker_.action = visualization_msgs::msg::Marker::ADD;
    angular_v_marker_.points.clear();
    angular_v_marker_.points.emplace_back(position_marker_.pose.position);
    arrow_end = position_marker_.pose.position;
    arrow_end.z += target_msg.v_yaw / M_PI;
    angular_v_marker_.points.emplace_back(arrow_end);

    // Armor cubes: one per face, using alternating r1/r2 and za/za+dz for 4-armor robots.
    armor_marker_.action = visualization_msgs::msg::Marker::ADD;
    bool is_small = false;
    if (best_tracker_id_ >= 0 && trackers_.count(best_tracker_id_)) {
      is_small = (trackers_[best_tracker_id_]->tracked_armor.type == "small");
    }
    armor_marker_.scale.y = is_small ? 0.140 : 0.235;  // armor width [m]
    armor_marker_.scale.z = is_small ? 0.125 : 0.127;  // armor height [m]
    bool is_current_pair = true;
    size_t a_n = target_msg.armors_num;
    geometry_msgs::msg::Point p_a;
    double r = 0;
    for (size_t i = 0; i < a_n; i++) {
      // Each face is angularly spaced by 2π/n from the previous
      double tmp_yaw = yaw + i * (2 * M_PI / a_n);
      // 4-armor robots alternate between two pairs with different r and z
      if (a_n == 4) {
        r = is_current_pair ? r1 : r2;
        p_a.z = za + (is_current_pair ? 0 : dz);
        is_current_pair = !is_current_pair;
      } else {
        r = r1;
        p_a.z = za;
      }
      // Face position = center − r × [cos, sin] (same formula as EKF h(x))
      p_a.x = xc - r * cos(tmp_yaw);
      p_a.y = yc - r * sin(tmp_yaw);

      armor_marker_.id = i;
      armor_marker_.pose.position = p_a;
      // Pitch of ±0.26 rad (~15°) matches the physical outward tilt of armor plates.
      // Outpost plates are tilted inward (negative pitch).
      tf2::Quaternion q;
      q.setRPY(0, target_msg.id == "outpost" ? -0.26 : 0.26, tmp_yaw);
      armor_marker_.pose.orientation = tf2::toMsg(q);
      marker_array.markers.emplace_back(armor_marker_);
    }
  } else {
    // No active tracking — delete all persistent markers
    position_marker_.action = visualization_msgs::msg::Marker::DELETE;
    linear_v_marker_.action = visualization_msgs::msg::Marker::DELETE;
    angular_v_marker_.action = visualization_msgs::msg::Marker::DELETE;

    // Delete all 4 armor slot markers (IDs 0-3) regardless of actual armors_num.
    // Using DELETE on a non-existent marker is harmless.
    armor_marker_.action = visualization_msgs::msg::Marker::DELETE;
    for (int i = 0; i < 4; i++) {
      armor_marker_.id = i;
      marker_array.markers.emplace_back(armor_marker_);
    }
  }

  marker_array.markers.emplace_back(position_marker_);
  marker_array.markers.emplace_back(linear_v_marker_);
  marker_array.markers.emplace_back(angular_v_marker_);
  marker_pub_->publish(marker_array);
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorTrackerNode)
