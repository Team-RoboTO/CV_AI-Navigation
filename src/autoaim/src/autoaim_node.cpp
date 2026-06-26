#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <autoaim/msg/armor_keypoint_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <angles/angles.h>

#include <opencv2/calib3d.hpp>
#include <cmath>
#include <algorithm>
#include <deque>
#include <iterator>
#include <set>
#include <string>
#include <array>
#include <vector>

#include "autoaim/pnp_solver.hpp"
#include "autoaim/tracker.hpp"

namespace autoaim
{

class AutoAimNode : public rclcpp::Node
{
public:
  explicit AutoAimNode(const rclcpp::NodeOptions & options)
  : Node("autoaim", options)
  {
    // ── All parameters in one place ──
    TrackerConfig cfg;
    cfg.light_ratio      = declare_parameter("light_ratio", 0.85);
    cfg.max_armor_dist   = declare_parameter("max_armor_distance", 8.0);
    cfg.max_armor_z      = declare_parameter("max_armor_z", 2.0);
    cfg.confirm_frames   = declare_parameter("confirm_frames", 3);
    cfg.lost_timeout     = declare_parameter("lost_timeout", 0.4);
    cfg.q_pos            = declare_parameter("q_pos", 5.0);
    cfg.q_z              = declare_parameter("q_z", 2.0);
    cfg.q_yaw            = declare_parameter("q_yaw", 10.0);
    cfg.q_r              = declare_parameter("q_r", 1e-6);
    cfg.r_pos_base       = declare_parameter("r_pos_base", 0.05);
    cfg.r_pos_slope      = declare_parameter("r_pos_slope", 0.04);
    cfg.r_yaw_base       = declare_parameter("r_yaw_base", 0.05);
    cfg.r_yaw_slope      = declare_parameter("r_yaw_slope", 0.005);
    cfg.max_oblique_deg  = declare_parameter("max_oblique_deg", 65.0);
    cfg.alpha_pos        = declare_parameter("alpha_pos", 0.99);
    cfg.alpha_yaw        = declare_parameter("alpha_yaw", 1.00);
    cfg.alpha_coast      = declare_parameter("alpha_coast", 0.95);
    cfg.initial_radius   = declare_parameter("initial_radius", 0.24);
    cfg.radius_ema_alpha = declare_parameter("radius_ema_alpha", 0.05);
    cfg.initial_dz       = declare_parameter("initial_dz", 0.0);
    cfg.bullet_speed     = declare_parameter("bullet_speed", 25.0);
    cfg.gravity          = declare_parameter("gravity", 9.8);
    cfg.gimbal_height    = declare_parameter("gimbal_height", 0.325);
    cfg.barrel_offset_x  = declare_parameter("barrel_offset_x", 0.0);
    cfg.barrel_offset_y  = declare_parameter("barrel_offset_y", 0.0);
    cfg.barrel_offset_z  = declare_parameter("barrel_offset_z", -0.10);
    cfg.angular_window   = declare_parameter("angular_window", 0.30);
    cfg.window_ref_dist  = declare_parameter("window_ref_dist", 3.0);
    cfg.min_fire_dist    = declare_parameter("min_fire_dist", 0.3);
    cfg.max_fire_dist    = declare_parameter("max_fire_dist", 6.0);
    cfg.fire_min_vis     = declare_parameter("fire_min_vis", 0.0);
    cfg.time_bias        = declare_parameter("time_bias", 0.10);
    // Measured-latency prediction horizon: the node measures
    // (now − capture stamp) per frame and the tracker adds actuation_latency
    // (serial TX + gimbal settle + muzzle exit). time_bias is only the
    // fallback for use_measured_latency = false.
    cfg.use_measured_latency = declare_parameter("use_measured_latency", true);
    cfg.actuation_latency    = declare_parameter("actuation_latency", 0.020);
    cfg.ref_freq         = declare_parameter("ref_freq", 100.0);
    cfg.max_match_dist   = declare_parameter("max_match_dist", 0.5);
    cfg.yaw_jump_thresh  = declare_parameter("yaw_jump_thresh", 0.50);
    cfg.maha_threshold   = declare_parameter("maha_threshold", 13.3);
    cfg.use_vyaw_from_timing = declare_parameter("use_vyaw_from_timing", true);
    cfg.vyaw_timing_min_dt   = declare_parameter("vyaw_timing_min_dt", 0.020);
    cfg.vyaw_timing_max_dt   = declare_parameter("vyaw_timing_max_dt", 0.400);
    cfg.vyaw_fire_threshold  = declare_parameter("vyaw_fire_threshold", 3.0);
    cfg.switch_range_ratio = declare_parameter("switch_range_ratio", 0.85);
    cfg.switch_cooldown  = declare_parameter("switch_cooldown", 10);
    cfg.same_target_identity_dist = declare_parameter("same_target_identity_dist", 1.0);

    tracker_ = std::make_unique<Tracker>(cfg);
    cfg_ = cfg;

    // YOLO26 keypoint model class IDs: "0"=blue armor, "1"=grey (ignored), "2"=red armor.
    // micro_color_target_classes[i] = YOLO class to shoot when micro sends color i:
    //   index 0 (micro: we are RED)  → shoot blue → "0"
    //   index 1 (micro: we are BLUE) → shoot red  → "2"
    auto tc = declare_parameter<std::vector<std::string>>(
      "target_classes", std::vector<std::string>{"0"});
    target_classes_ = std::set<std::string>(tc.begin(), tc.end());
    target_classes_.erase("1");  // never track grey
    target_classes_from_micro_status_ =
      declare_parameter("target_classes_from_micro_status", false);
    target_color_status_index_ = declare_parameter("target_color_status_index", 4);
    micro_color_target_classes_ = declare_parameter<std::vector<std::string>>(
      "micro_color_target_classes", std::vector<std::string>{"0", "2"});

    if (target_classes_from_micro_status_) {
      target_classes_.clear();
      RCLCPP_INFO(
        get_logger(),
        "Target class will be selected from /micro_status[%d]",
        target_color_status_index_);
    }

    // Per-axis command smoothing (EMA toward the destination). 1.0 = no
    // smoothing (pass-through). cmd_smooth_alpha is the legacy single knob and
    // acts as the default for both axes; the per-axis params override it so the
    // YAW can stay snappy (1.0) for fast movers while the PITCH is smoothed to
    // kill height jitter (e.g. 0.6) — the two no longer fight each other.
    smooth_alpha_       = declare_parameter("cmd_smooth_alpha", 1.0);
    smooth_alpha_yaw_   = declare_parameter("cmd_smooth_alpha_yaw",   smooth_alpha_);
    smooth_alpha_pitch_ = declare_parameter("cmd_smooth_alpha_pitch", smooth_alpha_);

    // Command stabilisation parameters.
    cmd_deadband_yaw_ = declare_parameter("cmd_deadband_yaw", 0.005);
    cmd_deadband_pitch_ = declare_parameter("cmd_deadband_pitch", 0.005);
    cmd_rate_limit_yaw_ = declare_parameter("cmd_rate_limit_yaw", 2.5);
    cmd_rate_limit_pitch_ = declare_parameter("cmd_rate_limit_pitch", 2.0);
    fire_lock_yaw_ = declare_parameter("fire_lock_yaw", 0.05);
    fire_lock_pitch_ = declare_parameter("fire_lock_pitch", 0.04);
    micro_pitch_feedback_opposite_sign_ = declare_parameter("micro_pitch_feedback_opposite_sign", true);
    // Separate flag for the fire-lock pitch comparison only.
    // With gimbal.pitch_sign=-1.0, set this to False (lock uses raw pitch directly).
    // With gimbal.pitch_sign=+1.0 and opposite firmware, set this to True.
    micro_pitch_lock_opposite_sign_ = declare_parameter("micro_pitch_lock_opposite_sign", false);

    // Safety when EKF/prediction briefly jumps off-screen or the target is lost.
    cmd_hold_time_ = declare_parameter("cmd_hold_time", 0.25);
    cmd_max_delta_yaw_ = declare_parameter("cmd_max_delta_yaw", 0.80);
    cmd_max_delta_pitch_ = declare_parameter("cmd_max_delta_pitch", 0.35);
    require_aim_inside_frame_ = declare_parameter("require_aim_inside_frame", false);

    use_ego_motion_compensation_ = declare_parameter("use_ego_motion_compensation", true);
    // ego_velocity_available: set to TRUE only when the micro is actually sending
    // valid vx/vy in /micro_status[2] and [3]. When FALSE, ego-motion compensation
    // is completely disabled regardless of use_ego_motion_compensation — robot_x_ and
    // robot_y_ stay at zero. This is the "I know I'm testing stationary" flag.
    // Once the micro firmware sends good velocity data, flip this to true.
    ego_velocity_available_ = declare_parameter("ego_velocity_available", false);
    ego_velocity_body_frame_ = declare_parameter("ego_velocity_body_frame", true);
    ego_velocity_scale_x_ = declare_parameter("ego_velocity_scale_x", 1.0);
    ego_velocity_scale_y_ = declare_parameter("ego_velocity_scale_y", 1.0);
    ego_velocity_max_ = declare_parameter("ego_velocity_max", 3.0);
    // Maximum drift [m] before dead-reckoning is reset to zero.
    // Pure velocity integration drifts over time. When the accumulated position
    // exceeds this cap, it is reset to zero. The tracker absorbs the jump via its
    // Mahalanobis gate. Set to ~half the arena diagonal for RoboMaster (4-5m).
    ego_position_max_drift_ = declare_parameter("ego_position_max_drift", 4.0);
    // Index in /micro_status where the CHASSIS heading [rad, world frame] is stored.
    // The chassis heading is needed to rotate body-frame velocity to world frame.
    // The gimbal IMU (imu_yaw_) is NOT the chassis heading — the gimbal is stabilised
    // and rotationally decoupled from the chassis which may be spinning at 200-300 RPM.
    // Set to -1 if the micro does not send chassis heading → falls back to gimbal yaw
    // (wrong for spinning-top mode, but usable until micro firmware is updated).
    // Once the micro sends chassis_yaw at index N, set this to N.
    chassis_heading_index_ = declare_parameter("chassis_heading_index", -1);

    // Static bore-sight correction [degrees].
    double pitch_offset_deg = declare_parameter("pitch_offset_deg", 0.0);
    double yaw_offset_deg   = declare_parameter("yaw_offset_deg", 0.0);
    pitch_offset_ = pitch_offset_deg * M_PI / 180.0;
    yaw_offset_   = yaw_offset_deg * M_PI / 180.0;

    // Gimbal sign corrections.
    yaw_sign_   = declare_parameter("gimbal.yaw_sign", 1.0);
    pitch_sign_ = declare_parameter("gimbal.pitch_sign", 1.0);

    // Detector input mode.
    use_keypoints_ = declare_parameter("use_keypoints", true);
    keypoint_topic_ = declare_parameter("keypoint_topic", std::string("/detector/armors_keypoints"));
    min_keypoint_score_ = declare_parameter("min_keypoint_score", 0.05);
    max_reproj_error_ = declare_parameter("max_reproj_error", 25.0);

    // Image <-> gimbal-angle time synchronization.
    // The image was captured 30-100 ms before the detection arrives here
    // (ZED capture + inference + transport). Projecting the armor into odom
    // with the LATEST gimbal angles is wrong by (gimbal slew rate x latency)
    // whenever the head is moving — at 3 rad/s and 80 ms that's 0.24 rad of
    // pure projection error, smeared in the direction of motion. The EKF then
    // chases an artifact of our own gimbal movement, which looks exactly like
    // "the tracker is slow / lags behind the robot".
    // When enabled, a ring buffer of time-stamped gimbal angles is kept and
    // the camera->odom transform uses the angles interpolated AT THE IMAGE
    // TIMESTAMP. Requires the detector to forward the camera frame stamp in
    // header.stamp (zed_detector.py now does — sl.TIME_REFERENCE.IMAGE).
    angle_sync_enable_ = declare_parameter("angle_sync_enable", true);

    // ── Subscribers ──
    if (use_keypoints_) {
      kpt_sub_ = create_subscription<autoaim::msg::ArmorKeypointArray>(
        keypoint_topic_, rclcpp::SensorDataQoS(),
        std::bind(&AutoAimNode::keypointCallback, this, std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "Using YOLO-pose keypoint detections on %s", keypoint_topic_.c_str());
    } else {
      det_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
        "/detector/armors", rclcpp::SensorDataQoS(),
        std::bind(&AutoAimNode::detectionCallback, this, std::placeholders::_1));
      RCLCPP_WARN(get_logger(), "Using legacy bbox-only detections on /detector/armors");
    }

    cam_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_info", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        image_width_ = static_cast<int>(msg->width);
        image_height_ = static_cast<int>(msg->height);

        if (pnp_.ready()) {
          return;
        }

        std::array<double, 9> K;
        std::copy(msg->k.begin(), msg->k.end(), K.begin());
        pnp_.init(K, msg->d);

        cam_fx_ = K[0];
        cam_fy_ = K[4];
        cam_cx_ = K[2];
        cam_cy_ = K[5];

        RCLCPP_INFO(
          get_logger(),
          "Camera intrinsics received (%dx%d) fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
          msg->width, msg->height, cam_fx_, cam_fy_, cam_cx_, cam_cy_);
      });

    micro_status_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/micro_status", rclcpp::SensorDataQoS(),
      std::bind(&AutoAimNode::microStatusCallback, this, std::placeholders::_1));

    // ── Publishers ──
    twist_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_AI", 10);
    aim_px_pub_ = create_publisher<geometry_msgs::msg::Twist>("/tracker/aim_pixels", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/tracker/marker", 10);

    RCLCPP_INFO(
      get_logger(),
      "Auto-aim node started: using /micro_status [yaw, pitch, vx, vy, ...], publishing /cmd_vel_AI");
  }

private:
  void microStatusCallback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg)
  {
    if (msg->data.size() < 2) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "/micro_status needs at least 2 floats: [yaw_rad, pitch_rad, vx, vy, ...]");
      return;
    }

    const double micro_yaw_rad = static_cast<double>(msg->data[0]);
    const double micro_pitch_rad = static_cast<double>(msg->data[1]);
    const double raw_vx = msg->data.size() > 2 ? static_cast<double>(msg->data[2]) : 0.0;
    const double raw_vy = msg->data.size() > 3 ? static_cast<double>(msg->data[3]) : 0.0;

    // Cache chassis heading if the micro provides it.
    cached_micro_data_size_ = msg->data.size();
    if (chassis_heading_index_ >= 0 &&
        static_cast<size_t>(chassis_heading_index_) < msg->data.size()) {
      chassis_yaw_ = static_cast<double>(msg->data[chassis_heading_index_]);
      last_chassis_yaw_valid_ = true;
    }

    // Raw microcontroller angles, exactly as reported by the micro.
    // Fire-lock and final deadband must compare against these raw values,
    // because /cmd_vel_AI is sent in this same micro convention.
    micro_yaw_raw_ = micro_yaw_rad;
    micro_pitch_raw_ = micro_pitch_rad;

    updateTargetClassesFromMicroStatus(*msg);

    // Internal ROS/gimbal convention used by PnP/tracker.
    // CRITICAL: if the micro reports pitch with opposite sign to the command
    // (e.g. commanded -0.10 → feedback +0.10), the GEOMETRY pitch used for
    // camera→odom and PnP must use the SAME corrected value, not the raw one.
    // Otherwise the gimbal physically points down but the code thinks it points
    // up, and odom-frame armor positions flip vertically.
    // Verify physically: tilt the gimbal up; the detected armor in RViz/odom
    // should rise. If it falls instead, this flag is wrong.
    const double micro_pitch_internal =
      micro_pitch_feedback_opposite_sign_ ? -micro_pitch_rad : micro_pitch_rad;

    imu_yaw_   = yaw_sign_ * micro_yaw_rad;
    imu_pitch_ = pitch_sign_ * micro_pitch_internal;
    imu_valid_ = true;

    // Ring buffer of time-stamped gimbal angles for image-time lookup.
    // Stamped at receipt time: the residual error is only the serial latency
    // (a few ms at 100 Hz / 500 kbaud), instead of the full vision-pipeline
    // latency we had before.
    angle_buffer_.push_back({this->now(), imu_yaw_, imu_pitch_});
    if (angle_buffer_.size() > kMaxAngleBuffer) {
      angle_buffer_.pop_front();
    }

    updateEgoPoseFromMicroVelocity(raw_vx, raw_vy);

    tf2::Quaternion q_gimbal;
    q_gimbal.setRPY(0.0, imu_pitch_, imu_yaw_);

    tf2::Quaternion q_conv;
    q_conv.setRPY(-M_PI / 2, 0, -M_PI / 2);

    imu_rotation_ = q_gimbal * q_conv;
    imu_translation_ = tf2::Vector3(robot_x_, robot_y_, cfg_.gimbal_height);
  }

  std::string targetClassesString() const
  {
    std::string out = "[";
    bool first = true;
    for (const auto & cls : target_classes_) {
      if (!first) {
        out += ", ";
      }
      out += cls;
      first = false;
    }
    out += "]";
    return out;
  }

  void resetTrackingForTargetClassChange()
  {
    tracker_ = std::make_unique<Tracker>(cfg_);
    last_target_generation_ = tracker_->targetGeneration();
    last_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    smooth_init_ = false;
    rate_init_ = false;
    has_safe_cmd_ = false;
    fire_hysteresis_active_ = false;

    if (twist_pub_) {
      geometry_msgs::msg::Twist twist;
      twist.angular.x = 0.0;
      twist.angular.y = micro_pitch_raw_;
      twist.angular.z = micro_yaw_raw_;
      twist_pub_->publish(twist);
    }
  }

  void updateTargetClassesFromMicroStatus(const std_msgs::msg::Float32MultiArray & msg)
  {
    if (!target_classes_from_micro_status_) {
      return;
    }

    if (target_color_status_index_ < 0 ||
        static_cast<size_t>(target_color_status_index_) >= msg.data.size())
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "target_color_status_index=%d is not available in /micro_status "
        "(size=%zu); not selecting a target color yet",
        target_color_status_index_, msg.data.size());
      return;
    }

    const double raw_color =
      static_cast<double>(msg.data[static_cast<size_t>(target_color_status_index_)]);

    if (!std::isfinite(raw_color)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "/micro_status[%d] target color is not finite: %.3f",
        target_color_status_index_, raw_color);
      return;
    }

    const int micro_color = static_cast<int>(std::lround(raw_color));
    if (std::abs(raw_color - static_cast<double>(micro_color)) > 0.25) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "/micro_status[%d] target color should be near an integer, got %.3f",
        target_color_status_index_, raw_color);
      return;
    }

    if (micro_color < 0 ||
        static_cast<size_t>(micro_color) >= micro_color_target_classes_.size())
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "/micro_status[%d]=%d has no entry in micro_color_target_classes "
        "(size=%zu)",
        target_color_status_index_, micro_color, micro_color_target_classes_.size());
      return;
    }

    const std::string & target_class =
      micro_color_target_classes_[static_cast<size_t>(micro_color)];
    if (target_class.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "micro_color_target_classes[%d] is empty; not selecting a target color",
        micro_color);
      return;
    }

    const std::set<std::string> next_classes{target_class};
    if (next_classes == target_classes_ && have_micro_target_color_ &&
        micro_color == last_micro_target_color_)
    {
      return;
    }

    target_classes_ = next_classes;
    have_micro_target_color_ = true;
    last_micro_target_color_ = micro_color;
    resetTrackingForTargetClassChange();

    RCLCPP_INFO(
      get_logger(),
      "Target class from /micro_status[%d]=%d -> %s",
      target_color_status_index_,
      micro_color,
      targetClassesString().c_str());
  }

  void updateEgoPoseFromMicroVelocity(double raw_vx, double raw_vy)
  {
    // Gate: both the master switch AND the "micro is actually sending vx/vy" flag
    // must be true. When ego_velocity_available_ is false, robot_x_/robot_y_ stay
    // at zero — safe for stationary testing without valid velocity data.
    if (!use_ego_motion_compensation_ || !ego_velocity_available_) {
      // Also clear cached velocity so computeAim future-barrel propagation is a no-op.
      robot_vx_world_ = 0.0;
      robot_vy_world_ = 0.0;
      return;
    }

    rclcpp::Time now = this->now();
    if (last_micro_status_time_.nanoseconds() == 0) {
      last_micro_status_time_ = now;
      return;
    }

    double dt = std::clamp((now - last_micro_status_time_).seconds(), 0.0, 0.05);
    last_micro_status_time_ = now;
    if (dt <= 0.0) {
      return;
    }

    double vx = raw_vx * ego_velocity_scale_x_;
    double vy = raw_vy * ego_velocity_scale_y_;
    const double vnorm = std::hypot(vx, vy);
    if (vnorm > ego_velocity_max_ && vnorm > 1e-6) {
      const double scale = ego_velocity_max_ / vnorm;
      vx *= scale;
      vy *= scale;
    }

    double vwx = vx;
    double vwy = vy;
    if (ego_velocity_body_frame_) {
      // IMPORTANT: rotate by CHASSIS heading, NOT gimbal heading.
      // imu_yaw_ is the stabilised gimbal head angle — it does NOT rotate
      // with the chassis (which may be spinning at 200-300 RPM).
      // To convert chassis body-frame velocity to world frame we need the
      // chassis absolute heading.
      //
      // chassis_heading_index_ >= 0  → use micro_status[chassis_heading_index_]
      //                                (requires micro firmware support)
      // chassis_heading_index_ == -1 → fall back to gimbal yaw (approximate;
      //                                only correct if chassis is not spinning)
      double chassis_yaw = imu_yaw_;  // fallback
      if (chassis_heading_index_ >= 0 &&
          last_chassis_yaw_valid_ &&
          chassis_heading_index_ < static_cast<int>(cached_micro_data_size_))
      {
        chassis_yaw = chassis_yaw_;
      } else if (chassis_heading_index_ >= 0) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "chassis_heading_index=%d set but not available in /micro_status "
          "(size=%zu). Falling back to gimbal yaw — ego-motion will be wrong "
          "during chassis spin. Add chassis heading to RX packet on the micro.",
          chassis_heading_index_, cached_micro_data_size_);
      }
      const double c = std::cos(chassis_yaw);
      const double ss = std::sin(chassis_yaw);
      vwx = c * vx - ss * vy;
      vwy = ss * vx + c * vy;
    }

    robot_x_ += vwx * dt;
    robot_y_ += vwy * dt;

    // Cache world-frame velocity so computeAim can propagate the barrel position
    // forward by the bullet flight time. When ego_velocity_available is false,
    // these stay at zero and the future-barrel correction reduces to a no-op.
    robot_vx_world_ = vwx;
    robot_vy_world_ = vwy;

    // Drift cap: pure dead-reckoning accumulates error over time.
    // If the integrated position exceeds the maximum allowed drift, reset to zero
    // — AND shift the tracker's world frame by the same amount so the target's
    // estimated position stays consistent. Without the shift, after reset the EKF
    // would think the target instantly teleported by `drift` meters and either
    // reject all detections via the Mahalanobis gate or fall into LOST.
    // Default ego_position_max_drift = 0.0 disables this entirely (safest until
    // ego-motion is fully validated).
    double drift = std::hypot(robot_x_, robot_y_);
    if (ego_position_max_drift_ > 0.0 && drift > ego_position_max_drift_) {
      RCLCPP_WARN(
        get_logger(),
        "Ego-motion drift %.2f m exceeds cap %.1f m — resetting origin and "
        "shifting tracker world frame.",
        drift, ego_position_max_drift_);
      double shift_dx = -robot_x_;
      double shift_dy = -robot_y_;
      robot_x_ = 0.0;
      robot_y_ = 0.0;
      // Translate the EKF target state by the same delta so the target doesn't
      // appear to jump in the tracker's frame.
      if (tracker_) tracker_->shiftWorldFrame(shift_dx, shift_dy);
    }
  }

  // ── Image <-> gimbal-angle time synchronization ────────────────────────
  struct AngleSample
  {
    rclcpp::Time t;
    double yaw;
    double pitch;
  };

  static constexpr size_t kMaxAngleBuffer = 500;  // ~5 s of history at 100 Hz

  /// Interpolate the gimbal angles at time t from the ring buffer.
  /// Clamps to the nearest sample when t is outside the buffered range.
  bool lookupAngles(const rclcpp::Time & t, double & yaw, double & pitch) const
  {
    if (angle_buffer_.empty()) {
      return false;
    }

    if (t <= angle_buffer_.front().t) {
      yaw = angle_buffer_.front().yaw;
      pitch = angle_buffer_.front().pitch;
      return true;
    }

    if (t >= angle_buffer_.back().t) {
      yaw = angle_buffer_.back().yaw;
      pitch = angle_buffer_.back().pitch;
      return true;
    }

    // Walk from the back: image stamps are normally close to the newest sample.
    for (auto it = angle_buffer_.rbegin(); std::next(it) != angle_buffer_.rend(); ++it) {
      const AngleSample & nb = *it;             // newer
      const AngleSample & ob = *std::next(it);  // older
      if (t >= ob.t && t <= nb.t) {
        const double span = (nb.t - ob.t).seconds();
        const double u = span > 1e-6 ? (t - ob.t).seconds() / span : 1.0;
        yaw = angles::normalize_angle(
          ob.yaw + u * angles::shortest_angular_distance(ob.yaw, nb.yaw));
        pitch = ob.pitch + u * (nb.pitch - ob.pitch);
        return true;
      }
    }

    return false;
  }

  /// Build the camera->odom transform for THIS image frame, using the gimbal
  /// angles at the image timestamp (when angle_sync_enable and the stamp are
  /// valid) instead of the latest angles. Falls back to the latest angles when
  /// sync is disabled, the stamp is zero, or the buffer is empty.
  void computeFrameTransform(const std_msgs::msg::Header & header)
  {
    double yaw = imu_yaw_;
    double pitch = imu_pitch_;

    if (angle_sync_enable_) {
      const rclcpp::Time t(header.stamp);
      if (t.nanoseconds() > 0) {
        double by = 0.0;
        double bp = 0.0;
        if (lookupAngles(t, by, bp)) {
          yaw = by;
          pitch = bp;
        }
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "angle_sync enabled but detection header.stamp is zero — the detector "
          "must forward the camera frame timestamp. Falling back to latest angles.");
      }
    }

    tf2::Quaternion q_gimbal;
    q_gimbal.setRPY(0.0, pitch, yaw);

    tf2::Quaternion q_conv;
    q_conv.setRPY(-M_PI / 2, 0, -M_PI / 2);

    frame_rotation_ = q_gimbal * q_conv;
    frame_translation_ = tf2::Vector3(robot_x_, robot_y_, cfg_.gimbal_height);
  }

  bool appendArmorFromPnP(
    const cv::Mat & rvec, const cv::Mat & tvec,
    const std::string & cid, double confidence,
    bool is_large, double reproj_error,
    std::vector<ArmorDetection> & armors)
  {
    if (tvec.empty() || rvec.empty()) {
      return false;
    }

    tf2::Vector3 p_cam(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));

    if (!std::isfinite(p_cam.x()) || !std::isfinite(p_cam.y()) || !std::isfinite(p_cam.z())) {
      return false;
    }

    if (p_cam.z() <= 0.05) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "REJECTED: PnP returned target behind/too close to camera z=%.3f",
        p_cam.z());
      return false;
    }

    cv::Mat rmat;
    cv::Rodrigues(rvec, rmat);

    tf2::Matrix3x3 tf_rm(
      rmat.at<double>(0, 0), rmat.at<double>(0, 1), rmat.at<double>(0, 2),
      rmat.at<double>(1, 0), rmat.at<double>(1, 1), rmat.at<double>(1, 2),
      rmat.at<double>(2, 0), rmat.at<double>(2, 1), rmat.at<double>(2, 2));

    tf2::Quaternion q_armor;
    tf_rm.getRotation(q_armor);
    if (q_armor.length() < 1e-6) {
      return false;
    }
    q_armor.normalize();

    // Use the transform built for THIS image frame (gimbal angles at the image
    // timestamp), not the latest angles — see computeFrameTransform().
    tf2::Vector3 p_odom = tf2::quatRotate(frame_rotation_, p_cam) + frame_translation_;
    tf2::Quaternion q_odom = frame_rotation_ * q_armor;

    double r, p, y;
    tf2::Matrix3x3(q_odom).getRPY(r, p, y);

    if (std::abs(p_odom.z()) > cfg_.max_armor_z) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "REJECTED: z=%.2f exceeds max_armor_z=%.1f",
        p_odom.z(), cfg_.max_armor_z);
      return false;
    }

    // Distance filter: range from CAMERA to armor, not from world origin.
    // With ego-motion active, the robot may have moved several meters from origin;
    // using world-frame distance would reject valid close targets.
    // p_cam is already the camera-to-armor vector — use it directly.
    const double rel_range_3d = std::sqrt(
      p_cam.x() * p_cam.x() + p_cam.y() * p_cam.y() + p_cam.z() * p_cam.z());
    if (rel_range_3d > cfg_.max_armor_dist) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "REJECTED: rel_range=%.2f exceeds max_armor_dist=%.1f",
        rel_range_3d, cfg_.max_armor_dist);
      return false;
    }

    // Tracker convention: yaw points from armor plate toward robot center.
    // The bearing must be from ROBOT to armor (not from world origin to armor),
    // otherwise the inward yaw correction is wrong when the robot has moved.
    double bearing_inward = std::atan2(p_odom.y() - robot_y_, p_odom.x() - robot_x_);
    double yaw_inward = y;

    if (std::abs(angles::shortest_angular_distance(bearing_inward, yaw_inward)) > M_PI / 2.0) {
      yaw_inward = angles::normalize_angle(yaw_inward + M_PI);
    }

    if (std::abs(angles::shortest_angular_distance(bearing_inward, yaw_inward)) >
        cfg_.max_oblique_deg * M_PI / 180.0)
    {
      yaw_inward = bearing_inward;
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "PnP OK [%s]: cam=(%.2f,%.2f,%.2f) odom=(%.2f,%.2f,%.2f) yaw=%.2f rel=%.2f large=%d err=%.2f",
      use_keypoints_ ? "keypoints" : "bbox",
      p_cam.x(), p_cam.y(), p_cam.z(),
      p_odom.x(), p_odom.y(), p_odom.z(),
      yaw_inward, rel_range_3d, is_large ? 1 : 0, reproj_error);

    ArmorDetection det{p_odom.x(), p_odom.y(), p_odom.z(), yaw_inward, cid, confidence, rel_range_3d};
    armors.push_back(det);
    return true;
  }

  void handleArmorMeasurements(
    const std_msgs::msg::Header & header,
    const std::vector<ArmorDetection> & armors)
  {
    rclcpp::Time now = header.stamp;
    double dt = 1.0 / cfg_.ref_freq;

    if (last_time_.nanoseconds() > 0) {
      dt = std::clamp((now - last_time_).seconds(), 0.01, 0.10);
    }

    last_time_ = now;

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Passing %zu armors to tracker (state=%d)",
      armors.size(), static_cast<int>(tracker_->state()));

    // Tell the tracker our current odom position. Used internally by
    // targetRange() / shouldSwitch() so the range comparison is consistent
    // with the detection's rel_range (both camera-relative).
    tracker_->setEgoPose(robot_x_, robot_y_);

    // Use the MEASUREMENT time for the tracker (capture stamp, fallback to now):
    // the vyaw-from-jump-timing estimator measures intervals between face jumps,
    // and capture intervals are jitter-free compared to processing time.
    rclcpp::Time meas_t = now;
    if (meas_t.nanoseconds() == 0) {
      meas_t = this->now();
    }
    tracker_->update(armors, dt, meas_t);

    // Detect target switch: if the tracker switched to a new target, reset
    // the node-side EMA and rate-limiter so the gimbal doesn't blend the
    // old target's angles with the new target's for 2-3 frames.
    int gen = tracker_->targetGeneration();
    if (gen != last_target_generation_) {
      smooth_init_ = false;
      rate_init_ = false;
      has_safe_cmd_ = false;
      fire_hysteresis_active_ = false;
      last_target_generation_ = gen;
    }

    // Measured pipeline latency: age of the EKF state (capture stamp) at the
    // moment the command is computed. The tracker adds actuation_latency on
    // top; setPipelineLatency clamps to [0, 0.25] so a zero or foreign-clock
    // stamp can never blow up the prediction horizon.
    double pipeline_latency = 0.0;
    if (now.nanoseconds() > 0) {
      pipeline_latency = (this->now() - now).seconds();
    }
    tracker_->setPipelineLatency(pipeline_latency);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "pipeline latency (capture->aim) = %.1f ms, + actuation_latency = %.0f ms "
      "(use_measured_latency=%d)",
      pipeline_latency * 1e3, cfg_.actuation_latency * 1e3,
      cfg_.use_measured_latency ? 1 : 0);

    auto aim = tracker_->computeAim(imu_yaw_, imu_pitch_,
                                    robot_x_, robot_y_,
                                    robot_vx_world_, robot_vy_world_);

    double aim_cam_yaw = 0.0;
    double aim_cam_pitch = 0.0;
    double imp_cam_yaw = 0.0;
    double imp_cam_pitch = 0.0;

    if (aim.tracking && aim.target_valid) {
      tf2::Vector3 target_odom(aim.target_x, aim.target_y, aim.target_z);

      tf2::Quaternion imu_inv = imu_rotation_.inverse();
      tf2::Vector3 target_cam = tf2::quatRotate(imu_inv, target_odom - imu_translation_);

      if (target_cam.z() > 0.1) {
        aim_cam_yaw   = std::atan2(target_cam.x(), target_cam.z());
        aim_cam_pitch = std::atan2(-target_cam.y(), target_cam.z());

        imp_cam_yaw = aim_cam_yaw;
        imp_cam_pitch = aim_cam_pitch;

        // Do NOT gate fire on camera-center error here. The barrel is below the
        // camera, so a correct ballistic aim point is not necessarily at the
        // optical center, especially at close range. Fire is gated later by
        // target validity, distance, face margin and micro/gimbal lock.
        aim.fire =
          aim.fire &&
          aim.distance >= cfg_.min_fire_dist &&
          aim.distance <= cfg_.max_fire_dist;
      } else {
        aim.fire = false;
      }
    }

    last_imp_yaw_ = imp_cam_yaw;
    last_imp_pitch_ = imp_cam_pitch;

    publishCommand(header, aim);
    publishMarkers(header, aim);
  }

  void keypointCallback(const autoaim::msg::ArmorKeypointArray::ConstSharedPtr msg)
  {
    if (!pnp_.ready()) {
      return;
    }

    if (!imu_valid_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for /micro_status: Float32MultiArray [yaw_rad, pitch_rad, vx, vy, ...]");
      return;
    }

    // Build the camera->odom transform for this image frame (time-synced angles).
    computeFrameTransform(msg->header);

    std::vector<ArmorDetection> armors;

    for (const auto & det : msg->detections) {
      const std::string cid = std::to_string(det.class_id);

      if (target_classes_.find(cid) == target_classes_.end()) {
        continue;
      }

      bool kpts_ok = true;
      std::array<cv::Point2f, 4> corners;

      for (size_t i = 0; i < 4; ++i) {
        const float x = det.keypoints_xy[2 * i + 0];
        const float y = det.keypoints_xy[2 * i + 1];
        const float score = det.keypoint_scores[i];
        const bool valid = det.keypoint_valid[i];

        if (!valid || !std::isfinite(x) || !std::isfinite(y) || score < min_keypoint_score_) {
          kpts_ok = false;
          break;
        }

        corners[i] = cv::Point2f(x, y);
      }

      if (!kpts_ok) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Skipping class=%s: invalid/low-score keypoints",
          cid.c_str());
        continue;
      }

      cv::Mat rvec, tvec;
      bool is_large = false;
      double reproj_error = 0.0;

      if (!pnp_.solveKeypoints(corners, rvec, tvec, is_large, &reproj_error, max_reproj_error_)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "PnP FAILED for keypoints class=%s reproj_err=%.2f",
          cid.c_str(), reproj_error);
        continue;
      }

      appendArmorFromPnP(
        rvec, tvec, cid, static_cast<double>(det.confidence),
        is_large, reproj_error, armors);
    }

    handleArmorMeasurements(msg->header, armors);
  }

  void detectionCallback(const vision_msgs::msg::Detection2DArray::ConstSharedPtr msg)
  {
    if (!pnp_.ready()) {
      return;
    }

    if (!imu_valid_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for /micro_status: Float32MultiArray [yaw_rad, pitch_rad, vx, vy, ...]");
      return;
    }

    // Build the camera->odom transform for this image frame (time-synced angles).
    computeFrameTransform(msg->header);

    std::vector<ArmorDetection> armors;

    for (const auto & det : msg->detections) {
      if (det.results.empty()) {
        continue;
      }

      const auto & cid = det.results[0].hypothesis.class_id;

      if (target_classes_.find(cid) == target_classes_.end()) {
        continue;
      }

      double cx = det.bbox.center.position.x;
      double cy = det.bbox.center.position.y;
      double w = det.bbox.size_x;
      double h = det.bbox.size_y;

      if (w < 1.0 || h < 1.0) {
        continue;
      }

      cv::Mat rvec, tvec;
      bool is_large = false;

      if (!pnp_.solve(cx, cy, w, h, cfg_.light_ratio, rvec, tvec, is_large)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "PnP FAILED for bbox (%.0f,%.0f,%.0f,%.0f)",
          cx, cy, w, h);
        continue;
      }

      appendArmorFromPnP(
        rvec, tvec, cid,
        static_cast<double>(det.results[0].hypothesis.score),
        is_large, -1.0, armors);
    }

    handleArmorMeasurements(msg->header, armors);
  }

  static double clampAbs(double v, double limit)
  {
    if (limit <= 0.0) {
      return v;
    }

    if (v > limit) {
      return limit;
    }

    if (v < -limit) {
      return -limit;
    }

    return v;
  }

  double rateLimitAngle(double target, double previous, double max_step)
  {
    if (max_step <= 0.0) {
      return target;
    }

    const double d = angles::shortest_angular_distance(previous, target);
    return angles::normalize_angle(previous + clampAbs(d, max_step));
  }

  double rateLimitLinear(double target, double previous, double max_step)
  {
    if (max_step <= 0.0) {
      return target;
    }

    return previous + clampAbs(target - previous, max_step);
  }

  void publishCommand(const std_msgs::msg::Header & header, const AimResult & aim)
  {
    double yaw_target = aim.abs_yaw;
    double pitch_target = aim.abs_pitch;
    double yaw_error = aim.rel_yaw;
    double pitch_error = aim.rel_pitch;

    // Static bore-sight correction.
    pitch_target -= pitch_offset_;
    yaw_target = angles::normalize_angle(yaw_target - yaw_offset_);
    pitch_error -= pitch_offset_;
    yaw_error -= yaw_offset_;

    // EMA smoothing of absolute destination in internal ROS/gimbal convention.
    if (!aim.tracking) {
      smooth_init_ = false;
      rate_init_ = false;
    } else if (!smooth_init_) {
      sm_y_ = yaw_target;
      sm_p_ = pitch_target;
      smooth_init_ = true;
    } else {
      sm_y_ = angles::normalize_angle(
        sm_y_ + smooth_alpha_yaw_ * angles::shortest_angular_distance(sm_y_, yaw_target));
      sm_p_ = smooth_alpha_pitch_ * pitch_target + (1.0 - smooth_alpha_pitch_) * sm_p_;
      yaw_target = sm_y_;
      pitch_target = sm_p_;
    }

    // Stabilization in internal convention.
    double cmd_yaw_err_internal = angles::shortest_angular_distance(imu_yaw_, yaw_target);
    double cmd_pitch_err_internal = pitch_target - imu_pitch_;

    if (aim.tracking) {
      if (std::abs(cmd_yaw_err_internal) < cmd_deadband_yaw_) {
        yaw_target = imu_yaw_;
        cmd_yaw_err_internal = 0.0;
      }

      if (std::abs(cmd_pitch_err_internal) < cmd_deadband_pitch_) {
        pitch_target = imu_pitch_;
        cmd_pitch_err_internal = 0.0;
      }

      rclcpp::Time cmd_t(header.stamp);
      double cmd_dt = 1.0 / cfg_.ref_freq;

      if (last_cmd_time_.nanoseconds() > 0) {
        cmd_dt = std::clamp((cmd_t - last_cmd_time_).seconds(), 0.005, 0.10);
      }

      last_cmd_time_ = cmd_t;

      if (!rate_init_) {
        rate_y_ = yaw_target;
        rate_p_ = pitch_target;
        rate_init_ = true;
      } else {
        yaw_target = rateLimitAngle(yaw_target, rate_y_, cmd_rate_limit_yaw_ * cmd_dt);
        pitch_target = rateLimitLinear(pitch_target, rate_p_, cmd_rate_limit_pitch_ * cmd_dt);
        rate_y_ = yaw_target;
        rate_p_ = pitch_target;
      }

      // Limit maximum command delta from current internal micro-corrected angle.
      if (cmd_max_delta_yaw_ > 0.0) {
        const double dy = clampAbs(
          angles::shortest_angular_distance(imu_yaw_, yaw_target),
          cmd_max_delta_yaw_);
          yaw_target = imu_yaw_ + dy;
      }

      if (cmd_max_delta_pitch_ > 0.0) {
        pitch_target = imu_pitch_ + clampAbs(
          pitch_target - imu_pitch_,
          cmd_max_delta_pitch_);
      }
    }

    // Convert final internal target to MICRO command convention.
    //
    // YAW FIX:
    // The micro yaw is incremental/unbounded. Therefore we must NOT send the
    // normalized absolute angle directly. We compute the shortest angular delta
    // from the current internal yaw to the desired internal target, then add that
    // delta to the raw micro yaw.
    //
    // Example:
    //   micro_yaw_raw_ = 34.0
    //   desired delta  = +0.5
    //   command sent   = 34.5
    //
    // Old wrong behavior:
    //   command sent = 0.5 or another normalized equivalent.
    const double yaw_delta_internal =
      angles::shortest_angular_distance(imu_yaw_, yaw_target);

    double yaw_target_micro =
      micro_yaw_raw_ + yaw_sign_ * yaw_delta_internal;

    // Pitch is usually bounded, so keep the previous convention.
    double pitch_target_micro = pitch_sign_ * pitch_target;


    // Debug projection and validity check.
    bool pixels_finite = false;
    bool pixels_inside = false;
    double aim_px_x = 0.0;
    double aim_px_y = 0.0;
    double imp_px_x = 0.0;
    double imp_px_y = 0.0;

    if (aim.tracking && pnp_.ready()) {
      last_aim_yaw_ = -angles::shortest_angular_distance(imu_yaw_, yaw_target);
      last_aim_pitch_ = pitch_target - imu_pitch_;

      aim_px_x = cam_cx_ + cam_fx_ * std::tan(last_aim_yaw_);
      aim_px_y = cam_cy_ - cam_fy_ * std::tan(last_aim_pitch_);
      imp_px_x = cam_cx_ + cam_fx_ * std::tan(last_imp_yaw_);
      imp_px_y = cam_cy_ - cam_fy_ * std::tan(last_imp_pitch_);

      pixels_finite =
        std::isfinite(aim_px_x) &&
        std::isfinite(aim_px_y) &&
        std::isfinite(imp_px_x) &&
        std::isfinite(imp_px_y);

      pixels_inside =
        pixels_finite &&
        aim_px_x >= 0.0 && aim_px_x < image_width_ &&
        aim_px_y >= 0.0 && aim_px_y < image_height_ &&
        imp_px_x >= 0.0 && imp_px_x < image_width_ &&
        imp_px_y >= 0.0 && imp_px_y < image_height_;
    }

    const bool finite_cmd =
      std::isfinite(yaw_target_micro) &&
      std::isfinite(pitch_target_micro);

    const bool command_valid =
      aim.tracking &&
      finite_cmd &&
      (!require_aim_inside_frame_ || pixels_inside);

    rclcpp::Time now_t(header.stamp);
    if (now_t.nanoseconds() == 0) {
      now_t = this->now();
    }

    bool holding_last_cmd = false;

    if (command_valid) {
      last_safe_cmd_yaw_micro_ = yaw_target_micro;
      last_safe_cmd_pitch_micro_ = pitch_target_micro;
      last_safe_cmd_time_ = now_t;
      has_safe_cmd_ = true;
    } else {
      const bool within_hold_time =
        has_safe_cmd_ &&
        (cmd_hold_time_ <= 0.0 ||
         (now_t - last_safe_cmd_time_).seconds() <= cmd_hold_time_);

      if (within_hold_time) {
        yaw_target_micro = last_safe_cmd_yaw_micro_;
        pitch_target_micro = last_safe_cmd_pitch_micro_;
      } else {
        // Use raw micro values because /cmd_vel_AI is in raw micro convention.
        yaw_target_micro = micro_yaw_raw_;
        pitch_target_micro = micro_pitch_raw_;
      }

      holding_last_cmd = true;

      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 500,
        "Holding last cmd: tracking=%d finite=%d pixels_inside=%d hold=%.2fs yaw=%.3f pitch=%.3f",
        static_cast<int>(aim.tracking),
        static_cast<int>(finite_cmd),
        static_cast<int>(pixels_inside),
        cmd_hold_time_,
        yaw_target_micro,
        pitch_target_micro);
    }

    // Fire-lock compares what we command with what the micro reports.
    // micro_pitch_lock_opposite_sign is SEPARATE from micro_pitch_feedback_opposite_sign:
    // - micro_pitch_feedback_opposite_sign: corrects the geometry/PnP pitch (imu_pitch_)
    // - micro_pitch_lock_opposite_sign: corrects the lock comparison only
    // With pitch_sign=-1.0, the geometry flip is already handled. The lock
    // comparison needs its own independent flag so the two don't fight each other.
    const double micro_pitch_for_lock =
      micro_pitch_lock_opposite_sign_ ? -micro_pitch_raw_ : micro_pitch_raw_;

    double yaw_err_micro =
      angles::shortest_angular_distance(micro_yaw_raw_, yaw_target_micro);

    double pitch_err_micro =
      pitch_target_micro - micro_pitch_for_lock;

    // Final micro-space deadband.
    if (aim.tracking && command_valid) {
      if (std::abs(yaw_err_micro) < cmd_deadband_yaw_) {
        yaw_target_micro = micro_yaw_raw_;
        yaw_err_micro = 0.0;
      }

      if (std::abs(pitch_err_micro) < cmd_deadband_pitch_) {
        pitch_target_micro = micro_pitch_for_lock;
        pitch_err_micro = 0.0;
      }
    }

    const bool command_locked =
      std::abs(yaw_err_micro) <= fire_lock_yaw_ &&
      std::abs(pitch_err_micro) <= fire_lock_pitch_;

    bool fire_decision = false;

    if (aim.fire && command_valid && command_locked && !holding_last_cmd) {
      fire_decision = true;
      fire_hysteresis_active_ = true;
    } else if (
      fire_hysteresis_active_ &&
      aim.fire &&
      aim.tracking &&
      command_valid &&
      command_locked &&
      !holding_last_cmd)
    {
      // Hysteresis: keep firing while aim.fire still agrees (face is still in window).
      // The extra aim.fire check prevents firing after the face has rotated past
      // the angular_window — without it the hysteresis would keep firing at edge-on plates.
      fire_decision = true;
    } else {
      fire_decision = false;
      fire_hysteresis_active_ = false;
    }

    if (!aim.tracking || holding_last_cmd) {
      fire_hysteresis_active_ = false;
      fire_decision = false;
    }

    geometry_msgs::msg::Twist twist;
    twist.angular.x = fire_decision ? 1.0 : 0.0;
    twist.angular.y = pitch_target_micro;
    twist.angular.z = yaw_target_micro;
    twist.linear.x = aim.distance;
    twist_pub_->publish(twist);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,
      "CMD micro: yaw=%.4f pitch=%.4f | micro raw yaw=%.4f pitch=%.4f | err yaw=%.4f pitch=%.4f | locked=%d fire=%d hold=%d",
      yaw_target_micro,
      pitch_target_micro,
      micro_yaw_raw_,
      micro_pitch_raw_,
      yaw_err_micro,
      pitch_err_micro,
      static_cast<int>(command_locked),
      static_cast<int>(fire_decision),
      static_cast<int>(holding_last_cmd));

    if (aim.tracking && pnp_.ready() && pixels_finite) {
      geometry_msgs::msg::Twist aim_px;
      aim_px.linear.x = aim_px_x;
      aim_px.linear.y = aim_px_y;
      aim_px.linear.z = holding_last_cmd ? 1.0 : 0.0;
      aim_px.angular.x = imp_px_x;
      aim_px.angular.y = imp_px_y;
      aim_px.angular.z = fire_decision ? 1.0 : 0.0;
      aim_px_pub_->publish(aim_px);
    }
  }

  void publishMarkers(const std_msgs::msg::Header & header, const AimResult & aim)
  {
    visualization_msgs::msg::MarkerArray arr;

    auto mk = [&](const std::string & ns, int id, int type) {
      visualization_msgs::msg::Marker m;
      m.header.stamp = header.stamp;
      m.header.frame_id = "odom";
      m.ns = ns;
      m.id = id;
      m.type = type;
      m.action = aim.tracking ?
        visualization_msgs::msg::Marker::ADD :
        visualization_msgs::msg::Marker::DELETE;
      m.scale.x = m.scale.y = m.scale.z = 0.1;
      m.color.a = 1.0;
      m.pose.orientation.w = 1.0;
      return m;
    };

    if (aim.tracking) {
      const auto & x = tracker_->ekfState();

      auto c = mk("center", 0, visualization_msgs::msg::Marker::SPHERE);
      c.pose.position.x = x(0);
      c.pose.position.y = x(2);
      c.pose.position.z = x(4);
      c.color.g = 1.0;
      arr.markers.push_back(c);

      for (int i = 0; i < 4; i++) {
        double fy = x(6) + i * M_PI / 2;
        double r = (i % 2 == 0) ? tracker_->radius() : tracker_->otherRadius();
        double dzo = (i % 2 == 1) ? tracker_->dz() : 0.0;

        auto m = mk("armor", i, visualization_msgs::msg::Marker::CUBE);
        m.pose.position.x = x(0) - r * std::cos(fy);
        m.pose.position.y = x(2) - r * std::sin(fy);
        m.pose.position.z = x(4) + dzo;

        tf2::Quaternion q;
        q.setRPY(0, 0.26, fy);
        m.pose.orientation = tf2::toMsg(q);

        m.scale.x = 0.03;
        m.scale.y = 0.14;
        m.scale.z = 0.125;
        m.color.r = 1.0;
        m.color.g = 0.0;
        arr.markers.push_back(m);
      }

      auto va = mk("velocity", 0, visualization_msgs::msg::Marker::ARROW);
      va.scale.x = 0.03;
      va.scale.y = 0.06;
      va.scale.z = 0.0;
      va.color.r = 1.0;
      va.color.g = 1.0;
      va.color.b = 0.0;

      geometry_msgs::msg::Point p_start, p_end;
      p_start.x = x(0);
      p_start.y = x(2);
      p_start.z = x(4);
      p_end.x = x(0) + x(1);
      p_end.y = x(2) + x(3);
      p_end.z = x(4) + x(5);

      va.points.push_back(p_start);
      va.points.push_back(p_end);
      arr.markers.push_back(va);

      {
        auto a = mk("aim", 0, visualization_msgs::msg::Marker::SPHERE);
        if (aim.target_valid) {
          double ray_dist = std::max(aim.distance, 0.5);
          // The aim ray emanates from the BARREL position in odom frame, which
          // is at the robot's current position plus the small fixed offset.
          // Without adding robot_x_/y_, the marker stays anchored to the world
          // origin and appears wrong in RViz once ego-motion is active.
          a.pose.position.x = robot_x_ + ray_dist * std::cos(aim.abs_pitch) * std::cos(aim.abs_yaw);
          a.pose.position.y = robot_y_ + ray_dist * std::cos(aim.abs_pitch) * std::sin(aim.abs_yaw);
          a.pose.position.z = cfg_.gimbal_height + ray_dist * std::sin(aim.abs_pitch);
        }
        a.scale.x = a.scale.y = a.scale.z = 0.08;
        a.color.r = 0.0;
        a.color.g = 1.0;
        a.color.b = 1.0;
        arr.markers.push_back(a);
      }

      {
        auto imp = mk("impact", 0, visualization_msgs::msg::Marker::SPHERE);
        if (aim.target_valid) {
          imp.pose.position.x = aim.target_x;
          imp.pose.position.y = aim.target_y;
          imp.pose.position.z = aim.target_z;
        }

        imp.scale.x = imp.scale.y = imp.scale.z = 0.12;
        imp.color.a = 0.6;

        if (aim.fire) {
          imp.color.r = 0.0;
          imp.color.g = 1.0;
          imp.color.b = 0.0;
        } else {
          imp.color.r = 1.0;
          imp.color.g = 0.2;
          imp.color.b = 0.0;
        }

        arr.markers.push_back(imp);
      }
    } else {
      arr.markers.push_back(mk("center", 0, visualization_msgs::msg::Marker::SPHERE));

      for (int i = 0; i < 4; i++) {
        arr.markers.push_back(mk("armor", i, visualization_msgs::msg::Marker::CUBE));
      }

      arr.markers.push_back(mk("velocity", 0, visualization_msgs::msg::Marker::ARROW));
      arr.markers.push_back(mk("aim", 0, visualization_msgs::msg::Marker::SPHERE));
      arr.markers.push_back(mk("impact", 0, visualization_msgs::msg::Marker::SPHERE));
    }

    marker_pub_->publish(arr);
  }

  // Members
  TrackerConfig cfg_;
  std::unique_ptr<Tracker> tracker_;
  PnPSolver pnp_;
  std::set<std::string> target_classes_;
  bool target_classes_from_micro_status_ = false;
  int target_color_status_index_ = 4;
  std::vector<std::string> micro_color_target_classes_;
  bool have_micro_target_color_ = false;
  int last_micro_target_color_ = -1;

  double imu_yaw_ = 0.0;
  double imu_pitch_ = 0.0;
  bool imu_valid_ = false;
  double yaw_sign_ = 1.0;
  double pitch_sign_ = 1.0;

  // Raw microcontroller angles, exactly as received from /micro_status.
  // These are used for fire-lock/deadband checks in the same convention as /cmd_vel_AI.
  double micro_yaw_raw_ = 0.0;
  double micro_pitch_raw_ = 0.0;

  tf2::Quaternion imu_rotation_{0, 0, 0, 1};
  tf2::Vector3 imu_translation_{0, 0, 0};

  // Image <-> gimbal-angle time sync.
  bool angle_sync_enable_ = true;
  std::deque<AngleSample> angle_buffer_;
  // Camera->odom transform for the CURRENT image frame (angles at image stamp).
  tf2::Quaternion frame_rotation_{0, 0, 0, 1};
  tf2::Vector3 frame_translation_{0, 0, 0};

  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};

  double smooth_alpha_ = 0.30;
  double smooth_alpha_yaw_ = 1.0;
  double smooth_alpha_pitch_ = 1.0;
  double cmd_deadband_yaw_ = 0.015;
  double cmd_deadband_pitch_ = 0.015;
  double cmd_rate_limit_yaw_ = 2.5;
  double cmd_rate_limit_pitch_ = 2.0;
  double fire_lock_yaw_ = 0.025;
  double fire_lock_pitch_ = 0.025;
  double cmd_hold_time_ = 0.25;
  double cmd_max_delta_yaw_ = 0.35;
  double cmd_max_delta_pitch_ = 0.25;
  bool require_aim_inside_frame_ = false;
  bool micro_pitch_feedback_opposite_sign_ = true;
  bool micro_pitch_lock_opposite_sign_ = false;

  bool use_ego_motion_compensation_ = true;
  bool ego_velocity_available_ = false;
  bool ego_velocity_body_frame_ = true;
  double ego_velocity_scale_x_ = 1.0;
  double ego_velocity_scale_y_ = 1.0;
  double ego_velocity_max_ = 3.0;
  double ego_position_max_drift_ = 4.0;
  double robot_x_ = 0.0;
  double robot_y_ = 0.0;
  // World-frame robot velocity (last value seen). Cached so computeAim can
  // propagate the barrel forward by flight_time. Zero when ego-motion off.
  double robot_vx_world_ = 0.0;
  double robot_vy_world_ = 0.0;
  rclcpp::Time last_micro_status_time_{0, 0, RCL_ROS_TIME};

  // Chassis heading — separate from gimbal heading because the head is stabilised.
  int chassis_heading_index_ = -1;
  double chassis_yaw_ = 0.0;
  bool last_chassis_yaw_valid_ = false;
  size_t cached_micro_data_size_ = 0;

  // Target switch detection.
  int last_target_generation_ = 0;

  bool has_safe_cmd_ = false;
  double last_safe_cmd_yaw_micro_ = 0.0;
  double last_safe_cmd_pitch_micro_ = 0.0;
  rclcpp::Time last_safe_cmd_time_{0, 0, RCL_ROS_TIME};

  double pitch_offset_ = 0.0;
  double yaw_offset_ = 0.0;

  bool fire_hysteresis_active_ = false;

  double last_aim_yaw_ = 0.0;
  double last_aim_pitch_ = 0.0;
  double last_imp_yaw_ = 0.0;
  double last_imp_pitch_ = 0.0;

  double sm_y_ = 0.0;
  double sm_p_ = 0.0;
  bool smooth_init_ = false;

  double rate_y_ = 0.0;
  double rate_p_ = 0.0;
  bool rate_init_ = false;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};

  bool use_keypoints_ = true;
  std::string keypoint_topic_ = "/detector/armors_keypoints";
  double min_keypoint_score_ = 0.05;
  double max_reproj_error_ = 25.0;

  rclcpp::Subscription<autoaim::msg::ArmorKeypointArray>::SharedPtr kpt_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr det_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr micro_status_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr aim_px_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  double cam_fx_ = 700.0;
  double cam_fy_ = 700.0;
  double cam_cx_ = 480.0;
  double cam_cy_ = 300.0;

  int image_width_ = 960;
  int image_height_ = 600;
};

}  // namespace autoaim

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoaim::AutoAimNode)
