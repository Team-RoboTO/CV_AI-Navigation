#ifndef AUTO_AIM__TRACKER_HPP_
#define AUTO_AIM__TRACKER_HPP_

#include <Eigen/Dense>
#include <cmath>
#include <string>
#include <vector>

namespace auto_aim
{

// armor detection in odom, after the TF transform.
struct ArmorDetection
{
  double x = 0, y = 0, z = 0;       // position in odom [m]
  double yaw = 0;                    // face normal direction [rad]
  std::string class_id;              // YOLO class, usually "0" blue or "3" red.
  double confidence = 0.0;           // detector score
  double pnp_reproj_err_norm = 0.0;  // mean pixel reproj err / bbox diag (P5)
  double range() const { return std::sqrt(x*x + y*y + z*z); }
};

// gimbal command output.
struct AimResult
{
  // absolute target angles in the IMU/odom startup frame [rad].
  // these are final destinations for the microcontroller, not increments.
  double abs_yaw   = 0;
  double abs_pitch = 0;

  // current angular error to the desired target [rad], only used for fire/debug.
  double rel_yaw   = 0;
  double rel_pitch = 0;

  double distance  = 0;  // range to the selected armor face [m]

  // selected armor face center in odom, used for debug projection and markers.
  bool   target_valid = false;
  double target_x = 0;
  double target_y = 0;
  double target_z = 0;

  bool   tracking  = false;
  bool   fire      = false;

  // debug-only fields (P1+).
  int    face_index   = -1;   // 0..3, or -1 if no aim
  double flight_time  = 0.0;  // ballistic flight time [s]
  double pred_t       = 0.0;  // prediction lead used by the planner [s]

  // P9 anti-gyro residual: signed seconds between bullet impact time and the
  // moment the selected face is normal-aligned with the bullet line. Zero
  // when the planner is in direct-aim mode.
  double anti_gyro_residual = 0.0;
  bool   anti_gyro_active   = false;
};

// all tracker config in one struct.
struct TrackerConfig
{
  // detection filters
  double light_ratio       = 0.85;
  double max_armor_dist    = 8.0;
  double max_armor_z       = 1.2;

  // tracker state machine
  int    confirm_frames    = 3;     // frames before DETECTING becomes TRACKING.
  double lost_timeout      = 0.4;   // seconds before TEMP_LOST becomes LOST.

  // EKF process noise, higher means faster response to measurements.
  double q_pos             = 5.0;
  double q_yaw             = 10.0;
  double q_r               = 1e-6;

  // EKF measurement noise, higher means smoother but slower.
  double r_pos_base        = 0.05;
  double r_pos_slope       = 0.04;
  double r_yaw_base        = 0.05;
  double r_yaw_slope       = 0.005;
  double max_oblique_deg   = 65.0;

  // velocity damping: v *= alpha each frame at ref_freq.
  double alpha_pos         = 0.90;
  double alpha_yaw         = 0.90;
  double alpha_coast       = 0.70;  // stronger damping when there is no measurement.

  // armor geometry
  double initial_radius    = 0.24;
  double radius_ema_alpha  = 0.05;

  // ballistics and physical geometry
  double bullet_speed      = 25.0;  // [m/s], measure this on the robot.
  double gravity           = 9.8;
  double gimbal_height     = 0.325; // camera/gimbal pivot above ground [m]

  // camera-to-barrel offset in the gimbal body frame [m].
  // measured from the camera optical center to the barrel exit point.
  //
  //   barrel_offset_x: forward (positive = barrel is in front of camera)
  //   barrel_offset_y: left    (positive = barrel is left of camera)
  //   barrel_offset_z: up      (positive = barrel is above camera)
  //
  // this matters most at close range: 10cm of offset is several degrees at 1-2m.
  // measure from the camera lens center to the barrel center.
  double barrel_offset_x   = 0.0;   // typically ~0.0 to 0.05
  double barrel_offset_y   = 0.0;   // typically ~0.0 (centered)
  double barrel_offset_z   = -0.10; // typically -0.05 to -0.15 (barrel below camera)

  // fire gate
  double angular_window    = 0.09;
  double window_ref_dist   = 3.0;
  double min_fire_dist     = 0.3;
  double max_fire_dist     = 8.0;

  // timing
  double time_bias         = 0.05;
  double ref_freq          = 30.0;

  // match gates
  double max_match_dist    = 0.5;
  double yaw_jump_thresh   = M_PI / 3.0;
  double maha_threshold    = 13.3;

  // target switching.
  // 0.85 means switch if the new target is closer than 85% of current range.
  double switch_range_ratio = 0.85;
  // short cooldown so two similar targets do not make the tracker bounce.
  int    switch_cooldown    = 10;  // ~0.33s at 30Hz

  // P8 adaptive Q. When enable_adaptive_q is true, the EKF reduces Q in the
  // stationary regime (small innovation, low velocity, low yaw rate) and
  // increases yaw Q in the high-yaw-rate regime. Default false leaves the
  // legacy fixed-Q behaviour intact.
  bool   enable_adaptive_q  = false;
  double stationary_speed_thresh   = 0.30;  // m/s, soft
  double stationary_yawrate_thresh = 0.50;  // rad/s, soft
  double stationary_innov_thresh   = 0.08;  // m, soft
  double q_reduction_max           = 0.90;  // up to 90% Q reduction when stationary
  double yawrate_q_boost_per_rad   = 0.20;  // per rad/s of |v_yaw|
  double yawrate_q_boost_max       = 2.00;  // capped multiplier

  // P9 anti-gyro. When enable_anti_gyro is true and |v_yaw| exceeds
  // anti_gyro_min_yawrate, the planner fills aim.anti_gyro_residual with the
  // signed-time alignment error between the selected face and the impact
  // moment. FireGate then enforces |residual| < anti_gyro_max_residual.
  bool   enable_anti_gyro       = false;
  double anti_gyro_min_yawrate  = 1.5;   // rad/s, threshold to enter anti-gyro mode
  double anti_gyro_hysteresis   = 0.5;   // rad/s, deadband on the threshold

  // Adaptive R. When enable_adaptive_r is true, the EKF inflates measurement
  // noise based on PnP reprojection quality and detector confidence. Default
  // false: legacy R uses only range and obliquity. The adaptive scaler is
  // additive on top, so range/obliquity behaviour is preserved.
  bool   enable_adaptive_r      = false;
  // Reprojection-norm above which R inflation begins. Below this value the
  // measurement is fully trusted. The scaler grows linearly (clamped) as the
  // norm crosses the threshold.
  double r_reproj_norm_soft     = 0.04;  // start to distrust above this (P4 setpoint)
  double r_reproj_norm_hard     = 0.12;  // fully untrusted (max scaler) at this
  double r_reproj_scaler_max    = 16.0;  // cap on R inflation factor
  double r_confidence_floor     = 0.20;  // detector score below this triggers full distrust
  // EMA on the per-frame R scaler to avoid jitter caused by single bad frames
  // dominating an otherwise good track. Set to 1.0 to disable smoothing.
  double r_quality_alpha        = 0.30;

  // Adaptive Q smoothing alpha. EMA on the stationary confidence so Q does
  // not chatter when the soft saturations cross thresholds. Set to 1.0 to
  // disable smoothing.
  double q_stationary_alpha     = 0.30;
  // Effective Q clamps. Prevent the adaptive schedule from collapsing Q to
  // zero (which freezes the EKF) or blowing up.
  double q_pos_eff_min          = 0.05;
  double q_pos_eff_max          = 200.0;
  double q_yaw_eff_min          = 0.10;
  double q_yaw_eff_max          = 200.0;
};

// spinning-top EKF, aim solver, and simple target switching.
// state: [xc, vxc, yc, vyc, za, vza, yaw, vyaw, r]
// measurement: [xa, ya, za, yaw]
class Tracker
{
public:
  explicit Tracker(const TrackerConfig & cfg);

  /// process one frame, with detections already transformed into odom.
  void update(const std::vector<ArmorDetection> & detections, double dt,
              double now_s = 0.0);

  /// compute gimbal angles from the current EKF state.
  ///
  /// pred_t = pred_lead_extra + flight_time, where pred_lead_extra is either
  /// `override_pred_lead_s` (when >= 0) or `cfg.time_bias` (legacy default).
  /// The override is honoured across both refinement iterations of the face
  /// search: previously, iter-1 silently fell back to time_bias which dropped
  /// any measured-latency contribution.
  AimResult computeAim(double current_yaw, double current_pitch,
                       double override_pred_lead_s = -1.0) const;

  enum State { LOST, DETECTING, TRACKING, TEMP_LOST };
  enum MeasurementQuality : uint8_t {
    MQ_NONE = 0,
    MQ_ACCEPTED = 1,
    MQ_DEGRADED = 2,
    MQ_REJECTED = 3,
  };
  State state() const { return state_; }
  const Eigen::VectorXd & ekfState() const { return x_; }
  const Eigen::MatrixXd & ekfCovariance() const { return P_; }
  double radius() const { return radius_; }
  double otherRadius() const { return other_radius_; }
  double dz() const { return dz_; }
  std::string targetId() const { return target_id_; }
  /// current tracked target range, used for debug and visualization.
  double targetRange() const;

  /// last innovation norm from the most recent EKF update, P1+.
  /// Mixes meters and radians; prefer the separated norms below.
  double lastInnovationNorm() const { return last_innovation_norm_; }
  /// last position innovation norm (sqrt of summed squared meters).
  double lastInnovationPosNorm() const { return last_innovation_pos_norm_; }
  /// last yaw innovation absolute value [rad].
  double lastInnovationYawAbs() const { return last_innovation_yaw_abs_; }
  /// last Mahalanobis distance of the matched detection, P1+.
  double lastMahalanobis() const { return last_mahalanobis_; }
  double lastBestMatchMahalanobis() const { return last_best_match_mahalanobis_; }
  double lastBestMatchPositionDiff() const { return last_best_match_position_diff_; }
  double lastBestMatchYawDiff() const { return last_best_match_yaw_diff_; }
  const std::string & lastMatchRejectReason() const { return last_match_reject_reason_; }
  const std::string & lastTrackerMissReason() const { return last_tracker_miss_reason_; }
  uint32_t lastAssignedCount() const { return last_assigned_count_; }
  uint32_t lastAssociationRejectCount() const { return last_association_reject_count_; }
  /// last effective Q values used by ekfPredict (P8 logging).
  double lastQPosEff() const { return last_q_pos_eff_; }
  double lastQYawEff() const { return last_q_yaw_eff_; }
  /// last effective sqrt(R) values used by ekfUpdate (P5 logging).
  double lastRPosEff() const { return last_r_pos_eff_; }
  double lastRYawEff() const { return last_r_yaw_eff_; }
  /// counters maintained by update().
  uint32_t consecutiveMatchCount() const { return match_count_; }
  uint32_t consecutiveMissCount() const { return miss_count_; }
  /// classification of the last measurement seen by update().
  MeasurementQuality lastMeasurementQuality() const { return last_meas_quality_; }
  /// time since the last accepted measurement [s], measured from `now`.
  double measurementAgeSeconds(double now_s) const {
    return last_match_time_s_ > 0.0 ? std::max(0.0, now_s - last_match_time_s_) : 0.0;
  }

private:
  void ekfPredict(double dt);
  Eigen::VectorXd ekfUpdate(const Eigen::Vector4d & z, const ArmorDetection & meta);
  double ekfMahalanobis(const Eigen::Vector4d & z) const;
  Eigen::Vector3d armorFromState(const Eigen::VectorXd & x) const;
  void initFromDetection(const ArmorDetection & det);
  void handleArmorJump(const ArmorDetection & det);
  double unwrapYaw(double raw_yaw);

  struct BallisticResult { double pitch, flight_time; bool valid; };
  BallisticResult solveBallistic(double ground_dist, double dz) const;

  /// check if a closer target is worth switching to.
  bool shouldSwitch(const ArmorDetection & candidate) const;

  TrackerConfig cfg_;
  Eigen::VectorXd x_;
  Eigen::MatrixXd P_, P0_;

  State state_ = LOST;
  int detect_count_ = 0;
  int lost_count_ = 0;
  int lost_thresh_ = 12;
  int switch_cooldown_counter_ = 0;

  double radius_ = 0.24;
  double other_radius_ = 0.28;
  double dz_ = 0.0;
  bool dz_initialized_ = false;
  double last_yaw_ = 0.0;
  std::string target_id_;

  // populated by ekfUpdate, exposed via the lastInnovation* / lastMahalanobis() accessors.
  mutable double last_innovation_norm_     = 0.0;  // legacy mixed norm
  mutable double last_innovation_pos_norm_ = 0.0;  // [m]
  mutable double last_innovation_yaw_abs_  = 0.0;  // [rad]
  mutable double last_mahalanobis_         = 0.0;
  mutable double last_best_match_mahalanobis_ = 0.0;
  mutable double last_best_match_position_diff_ = 0.0;
  mutable double last_best_match_yaw_diff_ = 0.0;
  std::string last_match_reject_reason_;
  std::string last_tracker_miss_reason_;
  uint32_t last_assigned_count_ = 0;
  uint32_t last_association_reject_count_ = 0;
  // Snapshots of the effective Q/R values from the most recent predict/update
  // step. Used by /auto_aim/debug; not used by the runtime decision path.
  mutable double last_q_pos_eff_ = 0.0;
  mutable double last_q_yaw_eff_ = 0.0;
  mutable double last_r_pos_eff_ = 0.0;
  mutable double last_r_yaw_eff_ = 0.0;
  // EMA state for adaptive Q stationary confidence (P8 cleanup).
  mutable double q_stationary_ema_ = 0.0;
  // EMA state for adaptive R quality scaler (P5).
  mutable double r_quality_ema_ = 1.0;
  // Track-quality counters and timing (P7 logging).
  uint32_t match_count_ = 0;
  uint32_t miss_count_  = 0;
  double   last_match_time_s_ = -1.0;   // <0 means "never"
  MeasurementQuality last_meas_quality_ = MQ_NONE;
};

}  // namespace auto_aim
#endif
