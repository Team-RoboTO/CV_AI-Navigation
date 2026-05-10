// one ROS2 node for the whole auto-aim path.
// it takes YOLO-pose keypoint detections, camera intrinsics, and micro
// yaw/pitch, then runs PnP, the EKF tracker, aim prediction, and command
// publishing.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <auto_aim/msg/armor_keypoint_array.hpp>
#include <auto_aim/msg/auto_aim_debug.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <angles/angles.h>

#include <opencv2/calib3d.hpp>
#include <array>
#include <cmath>
#include <algorithm>
#include <set>
#include <string>

#include "auto_aim/config_validator.hpp"
#include "auto_aim/debug_frame.hpp"
#include "auto_aim/debug_publisher.hpp"
#include "auto_aim/fire_gate.hpp"
#include "auto_aim/frame_transformer.hpp"
#include "auto_aim/pnp_solver.hpp"
#include "auto_aim/tracker.hpp"

namespace auto_aim
{

namespace
{

// Validate that the four keypoints form a convex quadrilateral with
// clockwise winding 0->1->2->3 in image coordinates (x right, y down). The
// expected winding for TL,TR,BR,BL in image-y-down is clockwise.
//
// Returns true when the quad is well-formed. Robust to in-plane rotation
// (does not assume axis-aligned armor); only catches index swaps and
// bow-tie configurations that would silently feed PnP a wrong correspondence.
bool keypointGeometryOk(const std::array<cv::Point2f, 4> & c)
{
  auto cross = [](cv::Point2f a, cv::Point2f b, cv::Point2f d) {
    return (b.x - a.x) * (d.y - a.y) - (b.y - a.y) * (d.x - a.x);
  };
  // Triangulate the quad two different ways and require all four sub-triangles
  // to share the same sign. Convex + non-self-intersecting <=> all same sign.
  const double s1 = cross(c[0], c[1], c[2]);
  const double s2 = cross(c[0], c[2], c[3]);
  const double s3 = cross(c[1], c[2], c[3]);
  const double s4 = cross(c[1], c[3], c[0]);
  if (!(std::isfinite(s1) && std::isfinite(s2) && std::isfinite(s3) &&
        std::isfinite(s4))) {
    return false;
  }
  // Clockwise in image-y-down means positive cross product for TL,TR,BR.
  return (s1 > 0 && s2 > 0 && s3 > 0 && s4 > 0);
}

}  // namespace

class AutoAimNode : public rclcpp::Node
{
public:
  explicit AutoAimNode(const rclcpp::NodeOptions & options)
  : Node("auto_aim", options)
  {
    // all runtime parameters stay here for now.
    TrackerConfig cfg;
    cfg.light_ratio      = declare_parameter("light_ratio", 0.85);
    cfg.max_armor_dist   = declare_parameter("max_armor_distance", 8.0);
    cfg.max_armor_z      = declare_parameter("max_armor_z", 1.2);
    cfg.confirm_frames   = declare_parameter("confirm_frames", 3);
    cfg.lost_timeout     = declare_parameter("lost_timeout", 0.4);
    cfg.q_pos            = declare_parameter("q_pos", 5.0);
    cfg.q_yaw            = declare_parameter("q_yaw", 10.0);
    cfg.q_r              = declare_parameter("q_r", 1e-6);
    cfg.r_pos_base       = declare_parameter("r_pos_base", 0.05);
    cfg.r_pos_slope      = declare_parameter("r_pos_slope", 0.04);
    cfg.r_yaw_base       = declare_parameter("r_yaw_base", 0.05);
    cfg.r_yaw_slope      = declare_parameter("r_yaw_slope", 0.005);
    cfg.max_oblique_deg  = declare_parameter("max_oblique_deg", 65.0);
    cfg.alpha_pos        = declare_parameter("alpha_pos", 0.90);
    cfg.alpha_yaw        = declare_parameter("alpha_yaw", 0.90);
    cfg.alpha_coast      = declare_parameter("alpha_coast", 0.70);
    cfg.initial_radius   = declare_parameter("initial_radius", 0.24);
    cfg.radius_ema_alpha = declare_parameter("radius_ema_alpha", 0.05);
    cfg.bullet_speed     = declare_parameter("bullet_speed", 25.0);
    cfg.gravity          = declare_parameter("gravity", 9.8);
    cfg.gimbal_height    = declare_parameter("gimbal_height", 0.325);
    cfg.barrel_offset_x  = declare_parameter("barrel_offset_x", 0.0);
    cfg.barrel_offset_y  = declare_parameter("barrel_offset_y", 0.0);
    cfg.barrel_offset_z  = declare_parameter("barrel_offset_z", -0.10);
    cfg.angular_window   = declare_parameter("angular_window", 0.09);
    cfg.window_ref_dist  = declare_parameter("window_ref_dist", 3.0);
    cfg.min_fire_dist    = declare_parameter("min_fire_dist", 0.3);
    cfg.max_fire_dist    = declare_parameter("max_fire_dist", 8.0);
    cfg.time_bias        = declare_parameter("time_bias", 0.05);
    cfg.ref_freq         = declare_parameter("ref_freq", 30.0);
    cfg.max_match_dist   = declare_parameter("max_match_dist", 0.5);
    cfg.maha_threshold   = declare_parameter("maha_threshold", 13.3);
    cfg.switch_range_ratio = declare_parameter("switch_range_ratio", 0.85);
    cfg.switch_cooldown  = declare_parameter("switch_cooldown", 10);
    cfg.enable_tracking_switch =
      declare_parameter("enable_tracking_switch", false);
    cfg.tracking_switch_range_ratio =
      declare_parameter("tracking_switch_range_ratio", 0.80);

    // P8 adaptive Q + P9 anti-gyro tunables. The on/off flags are declared
    // here so Tracker can read them from cfg at construction time.
    cfg.enable_adaptive_q          = declare_parameter("enable_adaptive_q", false);
    cfg.stationary_speed_thresh    = declare_parameter("stationary_speed_thresh", 0.30);
    cfg.stationary_yawrate_thresh  = declare_parameter("stationary_yawrate_thresh", 0.50);
    cfg.stationary_innov_thresh    = declare_parameter("stationary_innov_thresh", 0.08);
    cfg.q_reduction_max            = declare_parameter("q_reduction_max", 0.90);
    cfg.yawrate_q_boost_per_rad    = declare_parameter("yawrate_q_boost_per_rad", 0.20);
    cfg.yawrate_q_boost_max        = declare_parameter("yawrate_q_boost_max", 2.00);
    cfg.enable_anti_gyro           = declare_parameter("enable_anti_gyro", false);
    cfg.anti_gyro_min_yawrate      = declare_parameter("anti_gyro_min_yawrate", 1.5);
    cfg.anti_gyro_hysteresis       = declare_parameter("anti_gyro_hysteresis", 0.5);

    // P5 adaptive R + Q smoothing/clamps. All flags default off so a fresh
    // launch is identical to the legacy fixed-R/Q behaviour.
    cfg.enable_adaptive_r          = declare_parameter("enable_adaptive_r", false);
    cfg.r_reproj_norm_soft         = declare_parameter("r_reproj_norm_soft", 0.04);
    cfg.r_reproj_norm_hard         = declare_parameter("r_reproj_norm_hard", 0.12);
    cfg.r_reproj_scaler_max        = declare_parameter("r_reproj_scaler_max", 16.0);
    cfg.r_confidence_floor         = declare_parameter("r_confidence_floor", 0.20);
    cfg.r_quality_alpha            = declare_parameter("r_quality_alpha", 0.30);
    cfg.q_stationary_alpha         = declare_parameter("q_stationary_alpha", 0.30);
    cfg.q_pos_eff_min              = declare_parameter("q_pos_eff_min", 0.05);
    cfg.q_pos_eff_max              = declare_parameter("q_pos_eff_max", 200.0);
    cfg.q_yaw_eff_min              = declare_parameter("q_yaw_eff_min", 0.10);
    cfg.q_yaw_eff_max              = declare_parameter("q_yaw_eff_max", 200.0);

    enable_adaptive_q_ = cfg.enable_adaptive_q;
    enable_anti_gyro_  = cfg.enable_anti_gyro;

    tracker_ = std::make_unique<Tracker>(cfg);
    cfg_ = cfg;
    frame_transformer_.setGimbalHeight(cfg_.gimbal_height);

    // Centralized fire-allow logic. Mirrors fire-related TrackerConfig values
    // so the gate, planner, and validator stay aligned without manual sync.
    FireGate::Config fg_cfg;
    fg_cfg.min_fire_dist          = cfg.min_fire_dist;
    fg_cfg.max_fire_dist          = cfg.max_fire_dist;
    fg_cfg.angular_window         = cfg.angular_window;
    fg_cfg.smoothing_lag_max      = declare_parameter("fire.smoothing_lag_max", 0.05);
    fg_cfg.stale_threshold_s      = declare_parameter("fire.stale_threshold_s", 0.20);
    fg_cfg.hysteresis_max_drift   = declare_parameter("fire.hysteresis_max_drift", 0.05);
    fg_cfg.anti_gyro_max_residual = declare_parameter("fire.anti_gyro_max_residual", 0.020);
    // Smoothing-lag gate is opt-in. EMA smoothing creates a steady phase lag
    // on a moving target, which would otherwise block fire forever. Use this
    // only for acquisition / static QA. See docs/auto_aim_debugging_guide.md.
    fg_cfg.enable_smoothing_gate  = declare_parameter("fire.enable_smoothing_gate", false);
    fg_cfg.enable_stale_gate      = declare_parameter("fire.enable_stale_gate", false);
    // anti-gyro gate auto-enables when the planner is in anti-gyro mode.
    fg_cfg.enable_anti_gyro_gate  = declare_parameter("fire.enable_anti_gyro_gate", enable_anti_gyro_);
    fire_gate_ = std::make_unique<FireGate>(fg_cfg);

    // target classes to track, usually "0" for blue or "3" for red.
    auto tc = declare_parameter<std::vector<std::string>>(
      "target_classes", std::vector<std::string>{"0"});
    target_classes_ = std::set<std::string>(tc.begin(), tc.end());
    target_classes_.erase("1");  // skip grey/dead detections.

    // YOLO-pose keypoint path is the only measurement model. There is no
    // longer a bbox fallback inside this node — the detector still publishes
    // /detector/armors for legacy debug consumers, but auto_aim does not
    // subscribe to it.
    keypoint_topic_ = declare_parameter(
      "keypoint_topic", std::string("/detector/armors_keypoints"));
    min_keypoint_score_ = declare_parameter("min_keypoint_score", 0.30);
    keypoint_max_reproj_error_ = declare_parameter("keypoint_max_reproj_error", 15.0);
    enable_keypoint_geometry_check_ =
      declare_parameter("enable_keypoint_geometry_check", true);

    smooth_alpha_ = declare_parameter("cmd_smooth_alpha", 0.4);

    // static bore-sight correction [deg].
    // aim the barrel at the armor center and put the HUD offsets here.
    double pitch_offset_deg = declare_parameter("pitch_offset_deg", 0.0);
    double yaw_offset_deg   = declare_parameter("yaw_offset_deg", 0.0);
    pitch_offset_ = pitch_offset_deg * M_PI / 180.0;
    yaw_offset_   = yaw_offset_deg * M_PI / 180.0;

    // gimbal sign corrections, flip if an axis is inverted.
    yaw_sign_   = declare_parameter("gimbal.yaw_sign", 1.0);
    pitch_sign_ = declare_parameter("gimbal.pitch_sign", 1.0);

    // optional feature flags consumed by later phases (default off so the
    // node behaves identically to the legacy code path). enable_adaptive_q
    // and enable_anti_gyro are declared earlier with the other tracker
    // config because Tracker reads them at construction time.
    enable_pnp_refine_ = declare_parameter("enable_pnp_refine", false);

    // P7: latency-compensated prediction. The default keeps the legacy
    // time_bias behaviour; setting use_measured_latency=true makes the
    // planner use a running EMA of capture-to-publish latency plus a fixed
    // gimbal_response_delay as the prediction lead.
    use_measured_latency_   = declare_parameter("use_measured_latency", false);
    gimbal_response_delay_  = declare_parameter("gimbal_response_delay_s", 0.030);
    latency_ema_alpha_      = declare_parameter("latency_ema_alpha", 0.10);
    publish_markers_enabled_ = declare_parameter("debug.publish_markers", true);
    clamp_aim_pixels_ = declare_parameter("debug.clamp_aim_pixels", true);

    // EMA delay compensation. When the published yaw/pitch is EMA-smoothed
    // with cmd_smooth_alpha < 1.0, the steady-state lag against a moving
    // signal is approximately frame_dt * (1 - alpha) / alpha seconds. When
    // this flag is on, we add that estimated delay to the prediction lead so
    // the planner aims slightly ahead, compensating for the smoother. Off by
    // default until robot tests confirm the reduction in lead-lag error
    // outweighs the loss of margin on stops/reversals.
    enable_ema_delay_compensation_ = declare_parameter("enable_ema_delay_compensation", false);
    ema_delay_max_s_               = declare_parameter("ema_delay_max_s", 0.120);

    // PnP health gate (P8 hardening). When on, PnP rejects results whose
    // normalized reprojection error or measured depth fall outside the
    // configured band. Default off so the node behaves like the legacy
    // absolute-error-only gate.
    enable_pnp_health_gate_   = declare_parameter("enable_pnp_health_gate", false);
    pnp_max_reproj_err_norm_  = declare_parameter("pnp_max_reproj_err_norm", 0.10);
    pnp_min_depth_m_          = declare_parameter("pnp_min_depth_m", 0.10);
    pnp_max_depth_m_          = declare_parameter("pnp_max_depth_m", 12.0);
    const std::string force_armor_size =
      declare_parameter("pnp.force_armor_size", std::string("auto"));
    if (force_armor_size == "auto") {
      pnp_force_size_mode_ = PnPSolver::ArmorSizeMode::AUTO;
    } else if (force_armor_size == "small") {
      pnp_force_size_mode_ = PnPSolver::ArmorSizeMode::SMALL;
    } else if (force_armor_size == "large") {
      pnp_force_size_mode_ = PnPSolver::ArmorSizeMode::LARGE;
    } else {
      RCLCPP_FATAL(get_logger(),
        "Invalid pnp.force_armor_size='%s' (expected auto|small|large)",
        force_armor_size.c_str());
      throw std::runtime_error("auto_aim: invalid pnp.force_armor_size");
    }

    // validate every parameter so a typo or wrong-units value fails loudly
    // at startup instead of silently mis-aiming the robot.
    auto vr = ConfigValidator::validate(
      cfg_, target_classes_, yaw_sign_, pitch_sign_,
      smooth_alpha_, pitch_offset_deg, yaw_offset_deg,
      min_keypoint_score_, keypoint_max_reproj_error_);
    if (!ConfigValidator::logResult(vr, get_logger())) {
      RCLCPP_FATAL(get_logger(),
        "Invalid auto_aim configuration: %zu error(s). Fix the YAML and re-launch.",
        vr.errors.size());
      throw std::runtime_error("auto_aim: invalid configuration");
    }

    // subscribers
    kpt_sub_ = create_subscription<auto_aim::msg::ArmorKeypointArray>(
      keypoint_topic_, rclcpp::SensorDataQoS(),
      std::bind(&AutoAimNode::keypointCallback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(),
      "Subscribed to YOLO-pose keypoints on %s (min_score=%.2f, max_reproj=%.1f, geom_check=%d)",
      keypoint_topic_.c_str(), min_keypoint_score_, keypoint_max_reproj_error_,
      enable_keypoint_geometry_check_ ? 1 : 0);

    cam_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_info", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        if (pnp_.ready()) return;
        std::array<double, 9> K;
        std::copy(msg->k.begin(), msg->k.end(), K.begin());
        pnp_.init(K, msg->d);
        // keep intrinsics for the aim pixel overlay.
        cam_fx_ = K[0]; cam_fy_ = K[4]; cam_cx_ = K[2]; cam_cy_ = K[5];
        cam_width_ = msg->width; cam_height_ = msg->height;
        RCLCPP_INFO(get_logger(), "Camera intrinsics received (%dx%d) fx=%.1f",
          msg->width, msg->height, cam_fx_);
      });

    // use the microcontroller yaw/pitch as the reference for absolute commands.
    // /micro_imu should publish Float32MultiArray with:
    //   data[0] = current yaw  [rad]
    //   data[1] = current pitch [rad]
    // this keeps ROS and the microcontroller in the same reference frame.
    imu_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/micro_imu", rclcpp::SensorDataQoS(),
      std::bind(&AutoAimNode::microImuCallback, this, std::placeholders::_1));

    // publishers
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/tracker/cmd_gimbal", rclcpp::SensorDataQoS());
    twist_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_AI", 10);
    aim_px_pub_ = create_publisher<geometry_msgs::msg::Twist>("/tracker/aim_pixels", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/tracker/marker", 10);

    // structured per-frame debug. /auto_aim/debug consumers can plot every
    // pipeline stage to diagnose where jitter or bias enters.
    debug_pub_ = std::make_unique<DebugPublisher>(
      this, "/auto_aim/debug",
      declare_parameter("debug.fire_log_period_s", 10.0),
      declare_parameter("debug.publish_enabled", true),
      declare_parameter("debug.histogram_log_enabled", true));

    RCLCPP_INFO(get_logger(), "Auto-aim node started: using /micro_imu [yaw_rad, pitch_rad], publishing absolute rad commands in micro convention");
  }

private:
  // store the current head/gimbal orientation and build the odom to camera rotation.
  //
  // /micro_imu convention:
  //   data[0] = yaw  [rad], absolute in the microcontroller reference
  //   data[1] = pitch [rad], absolute in the microcontroller reference
  //
  // /cmd_vel_AI sends absolute target yaw/pitch in radians, so this must use the
  // same reference as the microcontroller.
  //
  // convention rotation: ROS body to camera optical frame.
  void microImuCallback(const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg)
  {
    if (msg->data.size() < 2) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "/micro_imu needs at least 2 floats: [yaw_rad, pitch_rad]");
      return;
    }

    const double micro_yaw_rad = static_cast<double>(msg->data[0]);
    const double micro_pitch_rad = static_cast<double>(msg->data[1]);

    imu_yaw_   = yaw_sign_ * micro_yaw_rad;
    imu_pitch_ = pitch_sign_ * micro_pitch_rad;
    imu_valid_ = true;

    // FrameTransformer owns the rotation/translation; node members stay in
    // sync via accessors (also used by publishMarkers and the aim pixel path).
    frame_transformer_.updateFromImu(imu_yaw_, imu_pitch_);
    imu_rotation_ = frame_transformer_.rotation();
    imu_translation_ = frame_transformer_.translation();
  }

  // YOLO-pose path: direct PnP from the four predicted armor corners.
  void keypointCallback(const auto_aim::msg::ArmorKeypointArray::ConstSharedPtr msg)
  {
    if (!pnp_.ready()) return;
    if (!imu_valid_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Waiting for /micro_imu: Float32MultiArray [yaw_rad, pitch_rad]");
      return;
    }

    DebugFrame f;
    f.raw_kp_detection_count = static_cast<uint32_t>(msg->detections.size());
    rclcpp::Time t_proc_start = this->now();
    f.latency_capture_to_process_s = (t_proc_start - rclcpp::Time(msg->header.stamp)).seconds();

    rclcpp::Time now = msg->header.stamp;
    double dt = 1.0 / cfg_.ref_freq;
    if (last_time_.nanoseconds() > 0)
      dt = std::clamp((now - last_time_).seconds(), 0.01, 0.10);
    last_time_ = now;

    const double max_oblique_rad = cfg_.max_oblique_deg * M_PI / 180.0;
    const double pnp_norm_gate = enable_pnp_health_gate_ ? pnp_max_reproj_err_norm_ : 0.0;
    const double pnp_min_depth = enable_pnp_health_gate_ ? pnp_min_depth_m_         : 0.0;
    const double pnp_max_depth = enable_pnp_health_gate_ ? pnp_max_depth_m_         : 1.0e6;

    std::vector<ArmorDetection> armors;
    bool det_logged = false;   // detection-side debug populated for this frame?
    bool pnp_logged = false;   // PnP-side debug populated for this frame?
    for (const auto & det : msg->detections) {
      const std::string cid = std::to_string(det.class_id);
      if (target_classes_.find(cid) == target_classes_.end()) {
        f.class_reject_count++;
        continue;
      }

      // Per-detection keypoint extraction + sanity checks. The contract from
      // ArmorKeypoint.msg is index order TL,TR,BR,BL in image pixels.
      std::array<cv::Point2f, 4> corners;
      std::array<float, 4> scores{};
      std::array<float, 8> raw_kp{};
      bool kpts_finite = true;
      bool kpts_low_score = false;
      bool kpts_out_of_image = false;
      double sum_score = 0.0;
      double min_score = 1.0;
      for (size_t i = 0; i < corners.size(); ++i) {
        const float x = det.keypoints_xy[2 * i + 0];
        const float y = det.keypoints_xy[2 * i + 1];
        const float score = det.keypoint_scores[i];
        const bool valid = det.keypoint_valid[i];
        corners[i] = cv::Point2f(x, y);
        scores[i] = score;
        raw_kp[2 * i + 0] = x;
        raw_kp[2 * i + 1] = y;
        sum_score += static_cast<double>(score);
        if (score < min_score) min_score = score;
        if (!valid || !std::isfinite(x) || !std::isfinite(y)) {
          kpts_finite = false;
        }
        if (msg->image_width > 0 && msg->image_height > 0 &&
            (x < 0.0f || y < 0.0f ||
             x > static_cast<float>(msg->image_width) ||
             y > static_cast<float>(msg->image_height))) {
          kpts_out_of_image = true;
        }
        if (score < min_keypoint_score_) {
          kpts_low_score = true;
        }
      }
      const float mean_score = static_cast<float>(sum_score / 4.0);
      const bool geom_ok =
        kpts_finite && !kpts_out_of_image && (!enable_keypoint_geometry_check_ ||
                        keypointGeometryOk(corners));

      // Always populate the per-frame debug fields for the FIRST detection
      // that passed the class filter, even when we end up rejecting the
      // measurement. /auto_aim/debug consumers can see exactly why a frame
      // was dropped without re-running the detector.
      auto fillFirstDet = [&]() {
        if (det_logged) return;
        f.detection_present = true;
        f.bbox_cx = det.bbox_cx; f.bbox_cy = det.bbox_cy;
        f.bbox_w  = det.bbox_w;  f.bbox_h  = det.bbox_h;
        f.detect_confidence = det.confidence;
        f.class_id = cid;
        f.meas_source = auto_aim::msg::AutoAimDebug::MEAS_SOURCE_KEYPOINT;
        f.kp_image_points = raw_kp;
        f.kp_scores = scores;
        f.kp_min_score = static_cast<float>(min_score);
        f.kp_mean_score = mean_score;
        f.kp_geometry_valid = geom_ok;
        det_logged = true;
      };

      if (!kpts_finite || kpts_low_score || kpts_out_of_image || !geom_ok) {
        fillFirstDet();
        if (!pnp_logged) {
          if (!kpts_finite) {
            f.kp_reject_reason = auto_aim::msg::AutoAimDebug::KP_NONFINITE;
          } else if (kpts_low_score) {
            f.kp_reject_reason = auto_aim::msg::AutoAimDebug::KP_LOW_SCORE;
          } else if (kpts_out_of_image) {
            f.kp_reject_reason = auto_aim::msg::AutoAimDebug::KP_OUT_OF_IMAGE;
          } else {
            f.kp_reject_reason = auto_aim::msg::AutoAimDebug::KP_INVALID_GEOMETRY;
          }
          f.pnp_ok = false;
          f.pnp_image_points = raw_kp;
          f.pnp_reject_reason = (uint8_t)PnPResult::REASON_NONFINITE;
          pnp_logged = true;
        }
        if (kpts_low_score) {
          f.kp_low_score_reject_count++;
        } else {
          f.kp_geometry_reject_count++;
        }
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "Skipping class=%s: kp finite=%d low_score=%d out_of_image=%d geom_ok=%d min=%.2f mean=%.2f",
          cid.c_str(), kpts_finite ? 1 : 0, kpts_low_score ? 1 : 0,
          kpts_out_of_image ? 1 : 0,
          geom_ok ? 1 : 0, min_score, mean_score);
        continue;
      }

      // The keypoints passed every per-detection gate; record them as the
      // current "best" debug frame and then run PnP.
      fillFirstDet();

      auto pnp = pnp_.solveKeypoints(
        corners, keypoint_max_reproj_error_, enable_pnp_refine_,
        pnp_norm_gate, pnp_min_depth, pnp_max_depth, pnp_force_size_mode_);
      if (!pnp.ok) {
        f.pnp_failed_count++;
        if (!pnp_logged) {
          f.kp_reject_reason = auto_aim::msg::AutoAimDebug::KP_OK;
          f.pnp_ok = false;
          f.pnp_reproj_err = (float)pnp.reproj_err;
          f.pnp_reproj_err_norm = (float)pnp.reproj_err_norm;
          f.pnp_is_large = pnp.is_large;
          f.pnp_small_reproj_err = (float)pnp.small_reproj_err;
          f.pnp_large_reproj_err = (float)pnp.large_reproj_err;
          f.pnp_size_margin = (float)pnp.size_margin;
          f.pnp_image_points = pnp.image_points;
          f.pnp_reject_reason = (uint8_t)pnp.reject_reason;
          pnp_logged = true;
        }
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "Keypoint PnP FAILED reason=%u class=%s reproj=%.2f reproj_norm=%.3f",
          (unsigned)pnp.reject_reason, cid.c_str(),
          pnp.reproj_err, pnp.reproj_err_norm);
        continue;
      }

      auto td = frame_transformer_.apply(pnp.rvec, pnp.tvec, max_oblique_rad);
      if (!td.valid) continue;

      tf2::Vector3 p_cam(pnp.tvec.at<double>(0), pnp.tvec.at<double>(1),
                         pnp.tvec.at<double>(2));

      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
        "Keypoint PnP OK: cam=(%.2f,%.2f,%.2f) large=%d reproj=%.2f",
        p_cam.x(), p_cam.y(), p_cam.z(), pnp.is_large, pnp.reproj_err);
      RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
        "Odom: pos=(%.2f,%.2f,%.2f) yaw=%.2f", td.x, td.y, td.z, td.yaw);

      if (!pnp_logged) {
        f.kp_reject_reason = auto_aim::msg::AutoAimDebug::KP_OK;
        f.pnp_ok = true;
        f.pnp_is_large = pnp.is_large;
        f.pnp_reproj_err = (float)pnp.reproj_err;
        f.pnp_reproj_err_norm = (float)pnp.reproj_err_norm;
        f.pnp_small_reproj_err = (float)pnp.small_reproj_err;
        f.pnp_large_reproj_err = (float)pnp.large_reproj_err;
        f.pnp_size_margin = (float)pnp.size_margin;
        f.pnp_image_points = pnp.image_points;
        f.pnp_reject_reason = (uint8_t)pnp.reject_reason;
        f.pnp_tvec_x = (float)p_cam.x();
        f.pnp_tvec_y = (float)p_cam.y();
        f.pnp_tvec_z = (float)p_cam.z();
        f.odom_x = (float)td.x;
        f.odom_y = (float)td.y;
        f.odom_z = (float)td.z;
        f.odom_yaw = (float)td.yaw;
        pnp_logged = true;
      }

      if (std::abs(td.z) > cfg_.max_armor_z) {
        f.pose_z_reject_count++;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "REJECTED: z=%.2f exceeds max_armor_z=%.1f", td.z, cfg_.max_armor_z);
        continue;
      }
      if (std::hypot(td.x, td.y) > cfg_.max_armor_dist) {
        f.pose_range_reject_count++;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "REJECTED: range=%.2f exceeds max_armor_dist=%.1f",
          std::hypot(td.x, td.y), cfg_.max_armor_dist);
        continue;
      }

      ArmorDetection a;
      a.x = td.x; a.y = td.y; a.z = td.z; a.yaw = td.yaw;
      a.class_id = cid;
      a.confidence = static_cast<double>(det.confidence);
      a.pnp_reproj_err_norm = pnp.reproj_err_norm;
      armors.push_back(std::move(a));
    }
    f.armors_passed_to_tracker_count = static_cast<uint32_t>(armors.size());

    finalizeFrame(msg->header, f, armors, dt, t_proc_start);
  }

  void finalizeFrame(
    const std_msgs::msg::Header & header,
    DebugFrame & f,
    const std::vector<ArmorDetection> & armors,
    double dt,
    const rclcpp::Time & t_proc_start)
  {
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
      "Passing %zu armors to tracker (state=%d)", armors.size(), (int)tracker_->state());

    // update tracker. now_s is wall-clock seconds, used for measurement-age
    // logging in the debug frame; tracker.cpp ignores it when zero.
    const double now_s_wall = this->now().seconds();
    tracker_->update(armors, dt, now_s_wall);

    // populate EKF debug fields after update.
    {
      const auto & xs = tracker_->ekfState();
      const auto & P  = tracker_->ekfCovariance();
      for (int i = 0; i < 9; ++i) f.ekf_state[i] = (float)xs(i);
      f.tracker_state = (uint8_t)tracker_->state();
      f.tracker_target_id = tracker_->targetId();
      f.tracker_assigned_count = tracker_->lastAssignedCount();
      f.tracker_association_reject_count = tracker_->lastAssociationRejectCount();
      f.tracker_miss_reason = tracker_->lastTrackerMissReason();
      f.ekf_innovation_norm     = (float)tracker_->lastInnovationNorm();
      f.ekf_innovation_pos_norm = (float)tracker_->lastInnovationPosNorm();
      f.ekf_innovation_yaw_abs  = (float)tracker_->lastInnovationYawAbs();
      f.ekf_mahalanobis         = (float)tracker_->lastMahalanobis();
      f.ekf_q_pos_eff           = (float)tracker_->lastQPosEff();
      f.ekf_q_yaw_eff           = (float)tracker_->lastQYawEff();
      f.ekf_r_pos_eff           = (float)tracker_->lastRPosEff();
      f.ekf_r_yaw_eff           = (float)tracker_->lastRYawEff();
      const double pos_var = std::max(0.0, P(0,0) + P(2,2) + P(4,4));
      const double yaw_var = std::max(0.0, P(6,6));
      f.ekf_pos_sigma = (float)std::sqrt(pos_var);
      f.ekf_yaw_sigma = (float)std::sqrt(yaw_var);
      f.ekf_match_count = tracker_->consecutiveMatchCount();
      f.ekf_miss_count  = tracker_->consecutiveMissCount();
      f.ekf_measurement_age_s = (float)tracker_->measurementAgeSeconds(now_s_wall);
      f.ekf_measurement_quality = (uint8_t)tracker_->lastMeasurementQuality();
      f.best_match_mahalanobis = (float)tracker_->lastBestMatchMahalanobis();
      f.best_match_position_diff = (float)tracker_->lastBestMatchPositionDiff();
      f.best_match_yaw_diff = (float)tracker_->lastBestMatchYawDiff();
      f.match_reject_reason = tracker_->lastMatchRejectReason();
    }

    // ---- Prediction-lead composition ----
    //
    // Default path (all flags off):
    //   pred_lead_extra = cfg.time_bias
    //
    // With use_measured_latency=true:
    //   pred_lead_extra = latency_ema + gimbal_response_delay
    //
    // With enable_ema_delay_compensation=true (and cmd_smooth_alpha < 1):
    //   pred_lead_extra += dt * (1 - alpha) / alpha     (clamped)
    //
    // computeAim adds the bullet flight time on top of pred_lead_extra. The
    // override_pred_lead_s argument replaces cfg.time_bias entirely; we keep
    // the time_bias contribution implicit when no flag is on.
    double pred_lead_measured = 0.0;
    double pred_lead_gimbal   = 0.0;
    double pred_lead_time_bias = 0.0;
    double pred_lead_ema      = 0.0;
    double override_pred_lead = -1.0;

    if (use_measured_latency_ && latency_estimate_initialized_) {
      pred_lead_measured = latency_estimate_s_;
      pred_lead_gimbal   = gimbal_response_delay_;
    } else {
      pred_lead_time_bias = cfg_.time_bias;
    }

    if (enable_ema_delay_compensation_ &&
        smooth_alpha_ < (1.0 - 1e-6) && smooth_alpha_ > 1e-3) {
      pred_lead_ema = std::clamp(dt * (1.0 - smooth_alpha_) / smooth_alpha_,
                                 0.0, ema_delay_max_s_);
    }

    if (use_measured_latency_ || enable_ema_delay_compensation_) {
      override_pred_lead = pred_lead_measured + pred_lead_gimbal +
                           pred_lead_time_bias + pred_lead_ema;
    }
    auto aim = tracker_->computeAim(imu_yaw_, imu_pitch_, override_pred_lead);

    // populate aim debug fields.
    f.aim_target_valid = aim.target_valid;
    f.aim_face_index = aim.face_index;
    f.aim_abs_yaw = (float)aim.abs_yaw;
    f.aim_abs_pitch = (float)aim.abs_pitch;
    f.aim_distance = (float)aim.distance;
    f.aim_target_x = (float)aim.target_x;
    f.aim_target_y = (float)aim.target_y;
    f.aim_target_z = (float)aim.target_z;
    f.aim_flight_time = (float)aim.flight_time;
    f.aim_pred_t = (float)aim.pred_t;

    // Prediction-lead component breakdown (always logged; zero where the
    // corresponding flag was off, so the debug consumer can verify exactly
    // which terms are active).
    f.pred_lead_measured_s   = pred_lead_measured;
    f.pred_lead_gimbal_s     = pred_lead_gimbal;
    f.pred_lead_time_bias_s  = (override_pred_lead < 0.0) ? cfg_.time_bias : pred_lead_time_bias;
    f.pred_lead_ema_s        = pred_lead_ema;
    f.pred_flight_time_s     = aim.flight_time;
    f.pred_t_total_s         = aim.pred_t;

    // 2D overlay rays in the camera optical frame. aim_cam_total_angle is also
    // consumed by FireGate as the OFF_AXIS gate input.
    double aim_cam_yaw = 0, aim_cam_pitch = 0;
    double imp_cam_yaw = 0, imp_cam_pitch = 0;
    double aim_cam_total_angle = 0.0;

    if (aim.tracking && aim.target_valid) {
      tf2::Vector3 target_odom(aim.target_x, aim.target_y, aim.target_z);

      // selected armor face from odom to camera frame.
      tf2::Quaternion imu_inv = imu_rotation_.inverse();
      tf2::Vector3 target_cam = tf2::quatRotate(imu_inv, target_odom - imu_translation_);

      if (target_cam.z() > 0.1) {
        aim_cam_yaw   = std::atan2(target_cam.x(), target_cam.z());
        aim_cam_pitch = std::atan2(-target_cam.y(), target_cam.z());
        imp_cam_yaw   = aim_cam_yaw;
        imp_cam_pitch = aim_cam_pitch;
        aim_cam_total_angle = std::sqrt(aim_cam_yaw*aim_cam_yaw + aim_cam_pitch*aim_cam_pitch);
      } else {
        // the geometry says the selected face is behind the camera. mark the
        // target invalid so FireGate emits INVALID_TARGET instead of the
        // current frame silently dropping fire.
        aim.target_valid = false;
      }
    }

    // store the impact ray for pixel publishing.
    // the aim ray is computed later after offsets and smoothing.
    last_imp_yaw_ = imp_cam_yaw;
    last_imp_pitch_ = imp_cam_pitch;

    // publish outputs.
    publishCommand(header, aim, aim_cam_total_angle, f);
    if (publish_markers_enabled_) {
      publishMarkers(header, aim);
    }

    // latency: process end and publish stamp.
    rclcpp::Time t_proc_end = this->now();
    f.latency_process_s = (t_proc_end - t_proc_start).seconds();
    f.latency_total_s = (t_proc_end - rclcpp::Time(header.stamp)).seconds();
    f.latency_process_to_publish_s =
      f.latency_total_s - f.latency_capture_to_process_s - f.latency_process_s;

    // P7: update the EMA used as the prediction lead in the next frame.
    if (!latency_estimate_initialized_) {
      latency_estimate_s_ = f.latency_total_s;
      latency_estimate_initialized_ = true;
    } else {
      latency_estimate_s_ =
        (1.0 - latency_ema_alpha_) * latency_estimate_s_ +
        latency_ema_alpha_ * f.latency_total_s;
    }
    f.latency_estimate_ema_s = latency_estimate_s_;

    debug_pub_->publish(header, f);
  }

  // Gimbal command contract (full description in docs/gimbal_command_contract.md):
  //
  //   Topic:        /tracker/cmd_gimbal
  //   Message:      geometry_msgs/Twist
  //   angular.z:    absolute yaw   [rad], micro convention, after yaw_sign
  //   angular.y:    absolute pitch [rad], micro convention, after pitch_sign
  //   angular.x:    fire flag (1.0 = fire, 0.0 = hold)
  //   linear.x:     distance to selected armor face [m]
  //
  //   Compatibility publisher /cmd_vel_AI carries the same yaw/pitch/fire
  //   triplet without distance. /tracker/aim_pixels is a 2D HUD overlay only.
  //
  // This contract MUST NOT change without coordinated firmware updates.
  void publishCommand(
    [[maybe_unused]] const std_msgs::msg::Header & header,
    const AimResult & aim,
    double aim_cam_total_angle,
    DebugFrame & f)
  {
    // absolute target commands.
    // these are final destinations in the startup IMU/odom frame, not increments.
    double yaw_target = aim.abs_yaw;
    double pitch_target = aim.abs_pitch;

    // static bore-sight correction, camera to barrel angular offset. The
    // calibration guide (docs/calibration_guide.md) explains why these should
    // be left at zero in any new deployment.
    pitch_target -= pitch_offset_;
    yaw_target = angles::normalize_angle(yaw_target - yaw_offset_);

    // capture the pre-smoothing target so FireGate can compute smoothing lag.
    const double yaw_target_pre_smooth = yaw_target;
    const double pitch_target_pre_smooth = pitch_target;

    // pre-smoothing values are useful for diagnosing EMA lag in /auto_aim/debug.
    f.cmd_yaw_pre_smooth = (float)(yaw_sign_ * yaw_target_pre_smooth);
    f.cmd_pitch_pre_smooth = (float)(pitch_sign_ * pitch_target_pre_smooth);

    // EMA smoothing on the absolute destination, with yaw wrap handling.
    if (!aim.tracking) {
      smooth_init_ = false;
    } else if (!smooth_init_) {
      sm_y_ = yaw_target; sm_p_ = pitch_target; smooth_init_ = true;
    } else {
      sm_y_ = angles::normalize_angle(
        sm_y_ + smooth_alpha_ * angles::shortest_angular_distance(sm_y_, yaw_target));
      sm_p_ = smooth_alpha_*pitch_target + (1-smooth_alpha_)*sm_p_;
      yaw_target = sm_y_; pitch_target = sm_p_;
    }

    // smoothing-lag input for FireGate: how far the published command is from
    // the target the planner asked for. Yaw uses shortest_angular_distance to
    // handle wrap. The lag is also published on /auto_aim/debug regardless of
    // whether the smoothing gate is enabled — it is a useful diagnostic.
    const double yaw_lag = aim.tracking ?
      angles::shortest_angular_distance(yaw_target, yaw_target_pre_smooth) : 0.0;
    const double pitch_lag = aim.tracking ?
      (pitch_target_pre_smooth - pitch_target) : 0.0;
    const double smoothing_lag_rad = std::sqrt(yaw_lag*yaw_lag + pitch_lag*pitch_lag);
    f.smoothing_lag_rad = (float)smoothing_lag_rad;

    // FireGate is the single source of truth for fire/hold. The previous
    // three-way split (Tracker geometry, callback off-axis check, publish
    // hysteresis) is now one call.
    FireGate::Inputs in;
    in.tracking            = aim.tracking;
    in.target_valid        = aim.target_valid;
    in.ballistic_valid     = aim.target_valid;
    in.planner_margin_ok   = aim.fire;
    in.distance            = aim.distance;
    in.aim_cam_total_angle = aim_cam_total_angle;
    in.smoothing_lag_rad   = smoothing_lag_rad;
    in.stale_seconds       = 0.0;  // P7 wires the latency tracker
    in.anti_gyro_residual  = aim.anti_gyro_residual;
    auto fd = fire_gate_->evaluate(in);
    bool fire_decision = fd.fire;

    // convert back to the micro convention before publishing.
    const double yaw_target_micro   = yaw_sign_   * yaw_target;
    const double pitch_target_micro = pitch_sign_ * pitch_target;

    // command as Twist on /tracker/cmd_gimbal.
    // angular.z = absolute yaw target in micro convention [rad]
    // angular.y = absolute pitch target in micro convention [rad]
    // angular.x = fire (1.0 = fire, 0.0 = hold)
    // linear.x = distance [m]
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = aim.tracking ? yaw_target_micro : 0.0;
    cmd.angular.y = aim.tracking ? pitch_target_micro : 0.0;
    cmd.angular.x = fire_decision ? 1.0 : 0.0;
    cmd.linear.x  = aim.distance;
    cmd_pub_->publish(cmd);

    f.cmd_yaw_published = (float)cmd.angular.z;
    f.cmd_pitch_published = (float)cmd.angular.y;
    f.cmd_fire = fire_decision;
    f.fire_blocker = (uint8_t)fd.blocker;
    f.fire_blocker_reason = fd.reason;

    // same command on /cmd_vel_AI for the legacy interface.
    geometry_msgs::msg::Twist twist;
    twist.angular.x = fire_decision ? 1.0 : 0.0;
    twist.angular.y = cmd.angular.y;
    twist.angular.z = cmd.angular.z;
    twist_pub_->publish(twist);

    // publish aim and impact as 2d pixels for the viewer.
    // aim is the commanded ballistic ray, impact is the selected armor center.
    if (aim.tracking && aim.target_valid && pnp_.ready()) {
      // Convert the absolute commanded ray back into the current camera
      // optical frame. Keep this on the same TF path as impact projection;
      // scalar yaw/pitch subtraction is wrong once pitch is nonzero.
      tf2::Vector3 aim_dir_odom(
        std::cos(pitch_target) * std::cos(yaw_target),
        std::cos(pitch_target) * std::sin(yaw_target),
        std::sin(pitch_target));
      tf2::Vector3 aim_dir_cam =
        tf2::quatRotate(imu_rotation_.inverse(), aim_dir_odom);
      if (aim_dir_cam.z() > 1e-6 &&
          std::isfinite(aim_dir_cam.x()) &&
          std::isfinite(aim_dir_cam.y()) &&
          std::isfinite(aim_dir_cam.z())) {
        last_aim_yaw_ = std::atan2(aim_dir_cam.x(), aim_dir_cam.z());
        last_aim_pitch_ = std::atan2(-aim_dir_cam.y(), aim_dir_cam.z());
      } else {
        last_aim_yaw_ = last_imp_yaw_;
        last_aim_pitch_ = last_imp_pitch_;
      }

      double aim_px_x = cam_cx_ + cam_fx_ * std::tan(last_aim_yaw_);
      double aim_px_y = cam_cy_ - cam_fy_ * std::tan(last_aim_pitch_);

      double imp_px_x = cam_cx_ + cam_fx_ * std::tan(last_imp_yaw_);
      double imp_px_y = cam_cy_ - cam_fy_ * std::tan(last_imp_pitch_);

      if (clamp_aim_pixels_ && cam_width_ > 0 && cam_height_ > 0 &&
          std::isfinite(aim_px_x) && std::isfinite(aim_px_y)) {
        const double raw_x = aim_px_x;
        const double raw_y = aim_px_y;
        const double margin = 4.0;
        aim_px_x = std::clamp(aim_px_x, margin, static_cast<double>(cam_width_) - margin);
        aim_px_y = std::clamp(aim_px_y, margin, static_cast<double>(cam_height_) - margin);
        if (std::abs(raw_x - aim_px_x) > 1e-3 || std::abs(raw_y - aim_px_y) > 1e-3) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "Aim pixel off-screen raw=(%.1f, %.1f) clamped=(%.1f, %.1f)",
            raw_x, raw_y, aim_px_x, aim_px_y);
        }
      }
      if (!std::isfinite(aim_px_x) || !std::isfinite(aim_px_y)) {
        aim_px_x = imp_px_x;
        aim_px_y = imp_px_y;
      }

      geometry_msgs::msg::Twist aim_px;
      aim_px.linear.x = aim_px_x;
      aim_px.linear.y = aim_px_y;
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
      m.ns = ns; m.id = id; m.type = type;
      m.action = aim.tracking ?
        visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
      m.scale.x = m.scale.y = m.scale.z = 0.1;
      m.color.a = 1.0;
      m.pose.orientation.w = 1.0;
      return m;
    };

    if (aim.tracking) {
      const auto & x = tracker_->ekfState();

      // robot center.
      auto c = mk("center", 0, visualization_msgs::msg::Marker::SPHERE);
      c.pose.position.x = x(0); c.pose.position.y = x(2); c.pose.position.z = x(4);
      c.color.g = 1.0;
      arr.markers.push_back(c);

      // armor plates.
      for (int i = 0; i < 4; i++) {
        double fy = x(6) + i * M_PI / 2;
        double r = (i%2==0) ? tracker_->radius() : tracker_->otherRadius();
        double dzo = (i%2==1) ? tracker_->dz() : 0.0;
        auto m = mk("armor", i, visualization_msgs::msg::Marker::CUBE);
        m.pose.position.x = x(0) - r*cos(fy);
        m.pose.position.y = x(2) - r*sin(fy);
        m.pose.position.z = x(4) + dzo;
        tf2::Quaternion q; q.setRPY(0, 0.26, fy);
        m.pose.orientation = tf2::toMsg(q);
        m.scale.x = 0.03; m.scale.y = 0.14; m.scale.z = 0.125;
        m.color.r = 1.0; m.color.g = 0.0;
        arr.markers.push_back(m);
      }

      // velocity arrow.
      auto va = mk("velocity", 0, visualization_msgs::msg::Marker::ARROW);
      va.scale.x = 0.03; va.scale.y = 0.06; va.scale.z = 0.0;
      va.color.r = 1.0; va.color.g = 1.0; va.color.b = 0.0;
      geometry_msgs::msg::Point p_start, p_end;
      p_start.x = x(0); p_start.y = x(2); p_start.z = x(4);
      p_end.x = x(0) + x(1); p_end.y = x(2) + x(3); p_end.z = x(4) + x(5);
      va.points.push_back(p_start);
      va.points.push_back(p_end);
      arr.markers.push_back(va);

      // aim marker, approximate point on the commanded ballistic ray.
      {
        auto a = mk("aim", 0, visualization_msgs::msg::Marker::SPHERE);
        if (aim.target_valid) {
          double ray_dist = std::max(aim.distance, 0.5);
          a.pose.position.x = ray_dist * std::cos(aim.abs_pitch) * std::cos(aim.abs_yaw);
          a.pose.position.y = ray_dist * std::cos(aim.abs_pitch) * std::sin(aim.abs_yaw);
          a.pose.position.z = cfg_.gimbal_height + ray_dist * std::sin(aim.abs_pitch);
        }
        a.scale.x = a.scale.y = a.scale.z = 0.08;
        a.color.r = 0.0; a.color.g = 1.0; a.color.b = 1.0;
        arr.markers.push_back(a);
      }

      // impact point, selected armor face center.
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
          imp.color.r = 0.0; imp.color.g = 1.0; imp.color.b = 0.0;
        } else {
          imp.color.r = 1.0; imp.color.g = 0.2; imp.color.b = 0.0;
        }
        arr.markers.push_back(imp);
      }
    } else {
      // clear old markers.
      arr.markers.push_back(mk("center", 0, visualization_msgs::msg::Marker::SPHERE));
      for (int i = 0; i < 4; i++)
        arr.markers.push_back(mk("armor", i, visualization_msgs::msg::Marker::CUBE));
      arr.markers.push_back(mk("velocity", 0, visualization_msgs::msg::Marker::ARROW));
      arr.markers.push_back(mk("aim", 0, visualization_msgs::msg::Marker::SPHERE));
      arr.markers.push_back(mk("impact", 0, visualization_msgs::msg::Marker::SPHERE));
    }
    marker_pub_->publish(arr);
  }

  // node state
  TrackerConfig cfg_;
  std::unique_ptr<Tracker> tracker_;
  PnPSolver pnp_;
  std::set<std::string> target_classes_;

  double imu_yaw_ = 0, imu_pitch_ = 0;
  bool imu_valid_ = false;
  double yaw_sign_ = 1.0, pitch_sign_ = 1.0;
  tf2::Quaternion imu_rotation_{0, 0, 0, 1};
  tf2::Vector3 imu_translation_{0, 0, 0};
  FrameTransformer frame_transformer_;

  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};

  double smooth_alpha_ = 0.4;
  double pitch_offset_ = 0.0;  // bore-sight pitch correction [rad]
  double yaw_offset_ = 0.0;    // bore-sight yaw correction [rad]

  // feature flags consumed by later phases. Default false means the runtime
  // behaviour is identical to the original monolithic node.
  bool enable_pnp_refine_ = false;
  bool enable_anti_gyro_  = false;
  bool enable_adaptive_q_ = false;

  // P7 latency tracking
  bool   use_measured_latency_   = false;
  double gimbal_response_delay_  = 0.030;
  double latency_ema_alpha_      = 0.10;
  double latency_estimate_s_     = 0.0;
  bool   latency_estimate_initialized_ = false;
  bool   publish_markers_enabled_ = true;
  bool   clamp_aim_pixels_ = true;

  // P3 EMA delay compensation
  bool   enable_ema_delay_compensation_ = false;
  double ema_delay_max_s_        = 0.120;

  // P8 PnP health gate
  bool   enable_pnp_health_gate_   = false;
  double pnp_max_reproj_err_norm_  = 0.10;
  double pnp_min_depth_m_          = 0.10;
  double pnp_max_depth_m_          = 12.0;
  PnPSolver::ArmorSizeMode pnp_force_size_mode_ = PnPSolver::ArmorSizeMode::AUTO;

  // YOLO-pose keypoint input
  std::string keypoint_topic_ = "/detector/armors_keypoints";
  double min_keypoint_score_ = 0.30;
  double keypoint_max_reproj_error_ = 15.0;
  bool   enable_keypoint_geometry_check_ = true;

  std::unique_ptr<FireGate> fire_gate_;
  double last_aim_yaw_ = 0, last_aim_pitch_ = 0;   // camera-frame ballistic command angles
  double last_imp_yaw_ = 0, last_imp_pitch_ = 0;   // camera-frame selected plate/impact angles
  double sm_y_ = 0, sm_p_ = 0;
  bool smooth_init_ = false;

  rclcpp::Subscription<auto_aim::msg::ArmorKeypointArray>::SharedPtr kpt_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr imu_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr aim_px_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  std::unique_ptr<DebugPublisher> debug_pub_;

  // camera intrinsics for aim pixel projection.
  double cam_fx_ = 700, cam_fy_ = 700, cam_cx_ = 480, cam_cy_ = 300;
  uint32_t cam_width_ = 0, cam_height_ = 0;
};

}  // namespace auto_aim

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(auto_aim::AutoAimNode)
