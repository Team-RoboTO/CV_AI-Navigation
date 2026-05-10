#include "auto_aim/fire_gate.hpp"

#include <cmath>

namespace auto_aim
{

FireGate::FireGate(const Config & cfg) : cfg_(cfg) {}

void FireGate::reset() { hysteresis_active_ = false; }

FireGate::Decision FireGate::evaluate(const Inputs & in)
{
  Decision d;

  // 1) Tracker must be locked. Hysteresis cannot save us if there is no
  //    target.
  if (!in.tracking) {
    hysteresis_active_ = false;
    d.blocker = NOT_TRACKING;
    d.reason = "tracker is not in TRACKING state";
    return d;
  }

  // 2) Target geometry must be valid.
  if (!in.target_valid) {
    hysteresis_active_ = false;
    d.blocker = INVALID_TARGET;
    d.reason = "AimPlanner returned target_valid=false";
    return d;
  }

  if (!in.ballistic_valid) {
    hysteresis_active_ = false;
    d.blocker = INVALID_BALLISTIC;
    d.reason = "ballistic solver could not converge";
    return d;
  }

  // 3) Range gate.
  if (in.distance < cfg_.min_fire_dist || in.distance > cfg_.max_fire_dist) {
    hysteresis_active_ = false;
    d.blocker = OUT_OF_RANGE;
    d.reason = "distance " + std::to_string(in.distance) + " outside [" +
               std::to_string(cfg_.min_fire_dist) + ", " +
               std::to_string(cfg_.max_fire_dist) + "]";
    return d;
  }

  // 4) Stale measurement gate (P7+).
  if (cfg_.enable_stale_gate && in.stale_seconds > cfg_.stale_threshold_s) {
    hysteresis_active_ = false;
    d.blocker = STALE_MEASUREMENT;
    d.reason = "last detection " + std::to_string(in.stale_seconds) + "s old";
    return d;
  }

  // 5) Anti-gyro timing gate (P9+).
  if (cfg_.enable_anti_gyro_gate &&
      std::abs(in.anti_gyro_residual) > cfg_.anti_gyro_max_residual) {
    hysteresis_active_ = false;
    d.blocker = ANTI_GYRO_TIMING;
    d.reason = "anti-gyro residual " + std::to_string(in.anti_gyro_residual) + "s too large";
    return d;
  }

  // 6) Geometry gates: planner margin AND camera-frame total angle.
  //    These are the gates that hysteresis can paper over for a few frames.
  bool margin_ok = in.planner_margin_ok;
  bool off_axis  = in.aim_cam_total_angle >= cfg_.angular_window;

  // 7) Smoothing-lag gate (diagnostic / opt-in, default OFF).
  //    EMA smoothing creates a near-constant phase lag against a moving
  //    target, so this gate would permanently block fire while the gimbal
  //    chases. Use it for acquisition / static QA only; in competition,
  //    leave it off and rely on geometry (margin/off-axis) + plan validity.
  //    smoothing_lag_rad is still logged on /auto_aim/debug regardless.
  bool smoothing_ok = true;
  if (cfg_.enable_smoothing_gate && in.smoothing_lag_rad > cfg_.smoothing_lag_max) {
    smoothing_ok = false;
  }

  // Primary path: all gates pass, fire and arm hysteresis.
  if (margin_ok && !off_axis && smoothing_ok) {
    hysteresis_active_ = true;
    d.fire = true;
    d.blocker = ALLOWED;
    d.reason = "all gates passed";
    return d;
  }

  // Hysteresis path: the previous frame fired, the tracker is still locked,
  // and the angular drift is small. Keep firing through the noisy frame.
  if (hysteresis_active_) {
    if (in.aim_cam_total_angle < cfg_.hysteresis_max_drift && smoothing_ok) {
      d.fire = true;
      d.blocker = ALLOWED;
      d.reason = "hysteresis hold (drift " +
                 std::to_string(in.aim_cam_total_angle) + " rad)";
      return d;
    }
    // Drift outside hysteresis: drop to non-firing and fall through to the
    // matching blocker reason.
    hysteresis_active_ = false;
  }

  // Pick the most informative blocker. Order matters: if smoothing is the
  // problem we want to see SMOOTHING_LAG, not MARGIN_NEGATIVE.
  if (!smoothing_ok) {
    d.blocker = SMOOTHING_LAG;
    d.reason = "smoothing lag " + std::to_string(in.smoothing_lag_rad) +
               " > " + std::to_string(cfg_.smoothing_lag_max);
  } else if (off_axis) {
    d.blocker = OFF_AXIS;
    d.reason = "camera-frame angle " + std::to_string(in.aim_cam_total_angle) +
               " >= " + std::to_string(cfg_.angular_window);
  } else {
    d.blocker = MARGIN_NEGATIVE;
    d.reason = "AimPlanner margin negative";
  }
  return d;
}

}  // namespace auto_aim
