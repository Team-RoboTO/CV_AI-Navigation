#include "auto_aim/config_validator.hpp"

#include <cmath>
#include <rclcpp/logging.hpp>
#include <sstream>

namespace auto_aim
{

namespace
{

// Helper: build a "<name> = <value> outside [<lo>, <hi>]" message.
std::string outOfRangeMsg(const std::string & name, double v, double lo, double hi)
{
  std::ostringstream os;
  os << name << "=" << v << " is outside the valid range [" << lo << ", " << hi << "]";
  return os.str();
}

// Helper: warn on a soft range, error on a hard range.
void checkRange(
  const std::string & name, double v, double soft_lo, double soft_hi,
  double hard_lo, double hard_hi,
  std::vector<std::string> & errors, std::vector<std::string> & warnings)
{
  if (v < hard_lo || v > hard_hi || !std::isfinite(v)) {
    errors.push_back(outOfRangeMsg(name, v, hard_lo, hard_hi));
  } else if (v < soft_lo || v > soft_hi) {
    warnings.push_back(name + "=" + std::to_string(v) +
      " is outside the recommended range [" + std::to_string(soft_lo) + ", " +
      std::to_string(soft_hi) + "]");
  }
}

}  // namespace

ValidationResult ConfigValidator::validate(
  const TrackerConfig & cfg,
  const std::set<std::string> & target_classes,
  double yaw_sign,
  double pitch_sign,
  double smooth_alpha,
  double pitch_offset_deg,
  double yaw_offset_deg,
  double min_keypoint_score,
  double keypoint_max_reproj_error)
{
  ValidationResult r;

  // Target classes must contain at least one entry, or no target will ever
  // match. This is silent without validation.
  if (target_classes.empty()) {
    r.errors.push_back("target_classes is empty: tracker will never match any detection");
  }

  // Bullet speed in [5, 50] m/s is the realistic competition envelope.
  // Outside [1, 100] is wrong-units or a typo.
  checkRange("bullet_speed", cfg.bullet_speed, 5.0, 50.0, 1.0, 100.0, r.errors, r.warnings);

  // Gravity is positive. We accept Earth-ish range loosely.
  checkRange("gravity", cfg.gravity, 9.0, 11.0, 1.0, 30.0, r.errors, r.warnings);

  // Gimbal height: 10 cm to 1 m on a typical RoboMaster chassis.
  checkRange("gimbal_height", cfg.gimbal_height, 0.10, 1.00, 0.01, 2.00, r.errors, r.warnings);

  // Barrel offsets are usually within +/- 30 cm. Anything beyond is a
  // measurement error (axes confused).
  checkRange("barrel_offset_x", cfg.barrel_offset_x, -0.30, 0.30, -1.00, 1.00, r.errors, r.warnings);
  checkRange("barrel_offset_y", cfg.barrel_offset_y, -0.30, 0.30, -1.00, 1.00, r.errors, r.warnings);
  checkRange("barrel_offset_z", cfg.barrel_offset_z, -0.30, 0.30, -1.00, 1.00, r.errors, r.warnings);

  // Smoothing damping coefficients are within [0, 1].
  checkRange("alpha_pos",   cfg.alpha_pos,   0.50, 1.00, 0.0, 1.0, r.errors, r.warnings);
  checkRange("alpha_yaw",   cfg.alpha_yaw,   0.50, 1.00, 0.0, 1.0, r.errors, r.warnings);
  checkRange("alpha_coast", cfg.alpha_coast, 0.10, 1.00, 0.0, 1.0, r.errors, r.warnings);

  // Command EMA alpha is bounded [0, 1]. Zero means no command ever changes
  // and one means no smoothing.
  checkRange("cmd_smooth_alpha", smooth_alpha, 0.05, 1.00, 0.0, 1.0, r.errors, r.warnings);

  // Light bar ratio is typically 0.4..1.0. Above 1 means the synthetic
  // rectangle would extend past the bbox (geometrically wrong).
  checkRange("light_ratio", cfg.light_ratio, 0.4, 1.0, 0.05, 1.5, r.errors, r.warnings);

  // Distances and ranges are positive.
  checkRange("max_armor_distance", cfg.max_armor_dist, 1.0, 15.0, 0.1, 50.0, r.errors, r.warnings);
  checkRange("max_armor_z",        cfg.max_armor_z,    0.3,  3.0, 0.05, 5.0, r.errors, r.warnings);
  checkRange("min_fire_dist",      cfg.min_fire_dist,  0.1,  3.0, 0.0, 10.0, r.errors, r.warnings);
  checkRange("max_fire_dist",      cfg.max_fire_dist,  1.0, 15.0, 0.1, 50.0, r.errors, r.warnings);
  if (cfg.max_fire_dist <= cfg.min_fire_dist) {
    r.errors.push_back("max_fire_dist must be strictly greater than min_fire_dist");
  }

  // Frequency reference and timeouts.
  checkRange("ref_freq", cfg.ref_freq, 10.0, 200.0, 1.0, 1000.0, r.errors, r.warnings);
  checkRange("lost_timeout", cfg.lost_timeout, 0.05, 2.0, 0.0, 10.0, r.errors, r.warnings);
  if (cfg.confirm_frames < 1) {
    r.errors.push_back("confirm_frames must be >= 1");
  }
  if (cfg.confirm_frames > 100) {
    r.warnings.push_back("confirm_frames > 100 may make the tracker too slow to lock");
  }

  // Match gates and Mahalanobis threshold.
  checkRange("max_match_dist", cfg.max_match_dist, 0.05, 2.0, 0.0, 10.0, r.errors, r.warnings);
  checkRange("maha_threshold", cfg.maha_threshold, 1.0, 50.0, 0.1, 1000.0, r.errors, r.warnings);

  // Angular window in radians. 0.09 rad is ~5 degrees.
  checkRange("angular_window", cfg.angular_window, 0.01, 0.30, 0.0, 1.5, r.errors, r.warnings);
  checkRange("window_ref_dist", cfg.window_ref_dist, 0.5, 10.0, 0.1, 50.0, r.errors, r.warnings);

  // Radius and ema.
  checkRange("initial_radius",   cfg.initial_radius,   0.10, 0.40, 0.01, 1.00, r.errors, r.warnings);
  checkRange("radius_ema_alpha", cfg.radius_ema_alpha, 0.001, 0.5, 0.0, 1.0, r.errors, r.warnings);

  // Process noise should be positive but not extreme.
  checkRange("q_pos", cfg.q_pos, 0.1, 100.0, 0.0, 1e6, r.errors, r.warnings);
  checkRange("q_yaw", cfg.q_yaw, 0.1, 100.0, 0.0, 1e6, r.errors, r.warnings);
  checkRange("q_r",   cfg.q_r,   1e-9, 1.0,  0.0, 1.0, r.errors, r.warnings);

  // Measurement noise.
  checkRange("r_pos_base",  cfg.r_pos_base,  0.001, 1.0, 0.0, 10.0, r.errors, r.warnings);
  checkRange("r_pos_slope", cfg.r_pos_slope, 0.0,   1.0, 0.0, 10.0, r.errors, r.warnings);
  checkRange("r_yaw_base",  cfg.r_yaw_base,  0.001, 1.0, 0.0, 10.0, r.errors, r.warnings);
  checkRange("r_yaw_slope", cfg.r_yaw_slope, 0.0,   1.0, 0.0, 10.0, r.errors, r.warnings);
  checkRange("max_oblique_deg", cfg.max_oblique_deg, 30.0, 85.0, 0.0, 90.0, r.errors, r.warnings);

  // Time bias is a positive (or zero) prediction lead used by the planner
  // before P7 replaces it with measured latency.
  checkRange("time_bias", cfg.time_bias, 0.0, 0.20, -0.10, 1.00, r.errors, r.warnings);

  // Adaptive R parameters (P5).
  checkRange("r_reproj_norm_soft",  cfg.r_reproj_norm_soft, 0.005, 0.20, 0.0, 1.0, r.errors, r.warnings);
  checkRange("r_reproj_norm_hard",  cfg.r_reproj_norm_hard, 0.02,  0.50, 0.0, 1.0, r.errors, r.warnings);
  if (cfg.r_reproj_norm_hard <= cfg.r_reproj_norm_soft) {
    r.errors.push_back("r_reproj_norm_hard must be strictly greater than r_reproj_norm_soft");
  }
  checkRange("r_reproj_scaler_max", cfg.r_reproj_scaler_max, 1.0, 64.0,  1.0, 1e6, r.errors, r.warnings);
  checkRange("r_confidence_floor",  cfg.r_confidence_floor,  0.0, 0.80,  0.0, 1.0, r.errors, r.warnings);
  checkRange("r_quality_alpha",     cfg.r_quality_alpha,     0.05, 1.0,  0.0, 1.0, r.errors, r.warnings);

  // Adaptive Q smoothing + clamps (P8 cleanup).
  checkRange("q_stationary_alpha",  cfg.q_stationary_alpha,  0.05, 1.0,  0.0, 1.0, r.errors, r.warnings);
  checkRange("q_pos_eff_min",       cfg.q_pos_eff_min,       0.001, 5.0, 0.0, 1e6, r.errors, r.warnings);
  checkRange("q_pos_eff_max",       cfg.q_pos_eff_max,       1.0,  500.0, 0.0, 1e6, r.errors, r.warnings);
  if (cfg.q_pos_eff_max <= cfg.q_pos_eff_min) {
    r.errors.push_back("q_pos_eff_max must be strictly greater than q_pos_eff_min");
  }
  checkRange("q_yaw_eff_min",       cfg.q_yaw_eff_min,       0.001, 5.0, 0.0, 1e6, r.errors, r.warnings);
  checkRange("q_yaw_eff_max",       cfg.q_yaw_eff_max,       1.0,  500.0, 0.0, 1e6, r.errors, r.warnings);
  if (cfg.q_yaw_eff_max <= cfg.q_yaw_eff_min) {
    r.errors.push_back("q_yaw_eff_max must be strictly greater than q_yaw_eff_min");
  }

  // Target switching ratio in (0, 1].
  checkRange("switch_range_ratio", cfg.switch_range_ratio, 0.5, 1.0, 0.0, 1.0, r.errors, r.warnings);
  checkRange("tracking_switch_range_ratio", cfg.tracking_switch_range_ratio,
             0.5, 1.0, 0.0, 1.0, r.errors, r.warnings);
  if (cfg.switch_cooldown < 0) {
    r.errors.push_back("switch_cooldown must be >= 0");
  }

  // Sign flips must be exactly +/- 1.
  if (std::abs(yaw_sign - 1.0) > 1e-6 && std::abs(yaw_sign + 1.0) > 1e-6) {
    r.errors.push_back("gimbal.yaw_sign must be +1.0 or -1.0");
  }
  if (std::abs(pitch_sign - 1.0) > 1e-6 && std::abs(pitch_sign + 1.0) > 1e-6) {
    r.errors.push_back("gimbal.pitch_sign must be +1.0 or -1.0");
  }

  // YOLO-pose keypoint gates. min_keypoint_score < 0.10 lets near-noise
  // keypoints into PnP and silently degrades aim. Hard error outside [0,1].
  checkRange("min_keypoint_score", min_keypoint_score,
             0.10, 0.95, 0.0, 1.0, r.errors, r.warnings);
  // keypoint_max_reproj_error is in pixels; competition values typically
  // 5..20 px. Anything above 50 px effectively disables the gate.
  checkRange("keypoint_max_reproj_error", keypoint_max_reproj_error,
             3.0, 25.0, 0.0, 200.0, r.errors, r.warnings);

  // Bore-sight offsets: warn loudly. The brief explicitly forbids using these
  // as a substitute for calibration.
  if (std::abs(pitch_offset_deg) > 0.01 || std::abs(yaw_offset_deg) > 0.01) {
    r.warnings.push_back(
      "Non-zero bore-sight offsets are set (pitch_offset_deg=" +
      std::to_string(pitch_offset_deg) + ", yaw_offset_deg=" +
      std::to_string(yaw_offset_deg) +
      "). These mask calibration errors. See docs/calibration_guide.md.");
  }

  return r;
}

bool ConfigValidator::logResult(const ValidationResult & r, rclcpp::Logger logger)
{
  for (const auto & w : r.warnings) {
    RCLCPP_WARN(logger, "[ConfigValidator] %s", w.c_str());
  }
  for (const auto & e : r.errors) {
    RCLCPP_FATAL(logger, "[ConfigValidator] %s", e.c_str());
  }
  return r.ok();
}

}  // namespace auto_aim
