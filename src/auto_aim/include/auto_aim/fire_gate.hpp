#ifndef AUTO_AIM__FIRE_GATE_HPP_
#define AUTO_AIM__FIRE_GATE_HPP_

#include <cstdint>
#include <string>

namespace auto_aim
{

// Centralized fire-allow logic. Replaces the old three-way split between
// Tracker::computeAim, the angular-window check in the detection callback,
// and the hysteresis branch in publishCommand. One call site, one decision,
// one reason.
//
// Hysteresis is internal: once fire has been granted and the tracker is
// still TRACKING, fire can stay on across small angular drifts so the
// firmware does not see flicker on noisy frames. The hysteresis is dropped
// the moment any non-margin gate fails (range, stale measurement, etc.) or
// the tracker loses lock.
class FireGate
{
public:
  enum Blocker : uint8_t
  {
    ALLOWED              = 0,
    NOT_TRACKING         = 1,
    OUT_OF_RANGE         = 2,
    MARGIN_NEGATIVE      = 3,
    OFF_AXIS             = 4,
    INVALID_TARGET       = 5,
    INVALID_BALLISTIC    = 6,
    SMOOTHING_LAG        = 7,
    STALE_MEASUREMENT    = 8,
    ANTI_GYRO_TIMING     = 9,
  };

  struct Decision
  {
    bool fire = false;
    Blocker blocker = NOT_TRACKING;
    std::string reason;  // human-readable, used by /auto_aim/debug
  };

  struct Inputs
  {
    bool   tracking            = false;
    bool   target_valid        = false;
    bool   ballistic_valid     = true;
    bool   planner_margin_ok   = false;  // best.margin >= 0 from AimPlanner
    double distance            = 0.0;
    double aim_cam_total_angle = 0.0;    // sqrt(yaw_cam^2 + pitch_cam^2)
    double smoothing_lag_rad   = 0.0;    // |published - target_pre_smooth|
    double stale_seconds       = 0.0;    // time since last detection
    double anti_gyro_residual  = 0.0;    // |t_impact - t_face_aligned| (P9)
  };

  struct Config
  {
    double min_fire_dist          = 0.3;
    double max_fire_dist          = 8.0;
    double angular_window         = 0.09;  // camera frame, rad
    double smoothing_lag_max      = 0.05;  // rad, allow fire when smooth catches up
    double stale_threshold_s      = 0.20;  // s
    double hysteresis_max_drift   = 0.05;  // rad, keep firing if smaller
    double anti_gyro_max_residual = 0.020; // s, P9
    // Smoothing-lag gate is OFF by default. EMA smoothing on a moving target
    // produces a near-constant phase lag (~v*(1-alpha)/alpha frames), so a gate
    // on |pre_smooth - published| would permanently block fire while tracking
    // a steady-moving target. Keep it as an opt-in diagnostic for acquisition
    // and static QA only; the physical fire decision should rely on actual
    // gimbal/aim error and plan validity instead.
    bool   enable_smoothing_gate  = false; // diagnostic only, see fire_gate.cpp
    bool   enable_stale_gate      = false; // P7 enables when latency wired
    bool   enable_anti_gyro_gate  = false; // P9 enables
  };

  explicit FireGate(const Config & cfg);

  void setConfig(const Config & cfg) { cfg_ = cfg; }

  // Evaluate the decision for one frame. Updates the internal hysteresis
  // state.
  Decision evaluate(const Inputs & in);

  // Reset hysteresis (e.g. when the tracker is forced to LOST).
  void reset();

private:
  Config cfg_;
  bool hysteresis_active_ = false;
};

}  // namespace auto_aim
#endif
