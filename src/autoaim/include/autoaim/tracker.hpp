#ifndef AUTOAIM__TRACKER_HPP_
#define AUTOAIM__TRACKER_HPP_

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include <rclcpp/time.hpp>

namespace autoaim
{

// ── Armor detection in odom frame (after TF transform) ──
struct ArmorDetection
{
  double x, y, z;       // position in odom [m]
  double yaw;           // face normal direction [rad]
  std::string class_id; // YOLO class ("0"=blue, "1"=grey, "2"=red)
  double confidence;
  // Range from CAMERA/ROBOT to armor [m]. Set during PnP from the camera-frame
  // translation vector. Use this for distance filtering and target selection.
  // DO NOT use sqrt(x*x+y*y+z*z) — those are world coords, and once ego-motion
  // is active the robot is no longer at the world origin.
  double rel_range = 0.0;
  double reproj_error = -1.0;
  bool yaw_replaced_by_bearing = false;
  // ── Planar-PnP yaw disambiguation ──
  // A single armor plate is planar, so IPPE returns TWO pose solutions whose
  // YAW differs (the classic flip ambiguity) while position is the same. The
  // node fills yaw_alt with the second solution's odom yaw (inward-corrected,
  // same convention as `yaw`). The tracker then picks whichever of {yaw, yaw_alt}
  // is closer to its predicted yaw — this kills the frame-to-frame yaw flicker
  // that otherwise pollutes the vyaw/spin estimate. has_yaw_alt is false when
  // there is only one solution or the primary yaw was already replaced by the
  // bearing (too oblique to trust either solution's yaw).
  double yaw_alt = 0.0;
  bool yaw_replaced_by_bearing_alt = false;
  bool has_yaw_alt = false;
  /// Backwards-compatible accessor — returns the camera-relative range.
  double range() const { return rel_range; }
};

// ── Gimbal command output ──
struct AimResult
{
  // Absolute target angles in the IMU/odom startup reference frame [rad].
  // These are final destinations for the microcontroller, not increments.
  double abs_yaw   = 0;
  double abs_pitch = 0;

  // Current angular error from the measured head/gimbal orientation to the
  // desired target [rad]. Kept for fire gating/debug only.
  double rel_yaw   = 0;
  double rel_pitch = 0;

  double distance  = 0;  // range to selected armor face [m]

  // Selected armor face center in odom frame. This is used only for debug
  // projection/markers, so the overlay stays on the plate center while the
  // command angles can still include barrel offset and gravity compensation.
  bool   target_valid = false;
  double target_x = 0;
  double target_y = 0;
  double target_z = 0;

  bool   tracking  = false;
  bool   fire      = false;

  // Debug-only details for the selected predicted face.
  int    best_face_idx    = -1;
  int    faces_checked    = 0;
  double phase_error      = 0;  // signed bearing-vs-face error [rad]
  double fire_window      = 0;  // selected face fire window [rad]
  double flight_time      = 0;  // bullet flight time [s]
  double prediction_time  = 0;  // flight_time + latency/bias [s]
  double fire_margin      = 0;  // positive means inside fire window [rad]

  // Phase-aware face scheduling debug. selected_face_idx mirrors
  // best_face_idx for newer tooling; the remaining fields explain whether the
  // aim point was chosen for immediate fire or to pre-aim an incoming face.
  int    selected_face_idx      = -1;
  bool   face_lookahead_active  = false;
  double face_time_to_window    = -1.0;  // seconds until selected face is fire-valid; -1 = n/a
  double face_margin            = 0.0;
  double face_phase_error       = 0.0;
  std::string face_switch_reason;
};

struct FacePrediction
{
  int idx = -1;
  double yaw = 0.0;       // face normal yaw in odom [rad]
  double radius = 0.0;    // radius used by this face [m]
  double z_offset = 0.0;  // height offset for this face [m]
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
};

struct TrackerDebugInfo
{
  bool matched = false;
  double mahalanobis = -1.0;
  double pd = -1.0;
  double yd = -1.0;
  bool oblique = false;
  double det_yaw = 0.0;
  double state_yaw = 0.0;
  bool yaw_replaced_by_bearing = false;
  double reproj_error = -1.0;
  std::string selected_class_id;

  bool jump_detected = false;
  double jump_abs = 0.0;
  double jump_dir = 0.0;
  bool one_face_jump = false;
  double dt_jump = -1.0;
  double vyaw_est_from_timing = 0.0;
  bool vyaw_timing_accepted = false;

  bool phase_confident = false;
  int matched_face_idx = -1;
  double association_mahalanobis = -1.0;
  bool association_yaw_valid = false;
  int association_yaw_hypothesis = 0;
  int last_matched_face_idx = -1;
  bool face_transition_observed = false;
};

// ── All config in one struct ──
struct TrackerConfig
{
  // Detection filters
  double light_ratio       = 0.85;
  double max_armor_dist    = 8.0;
  double max_armor_z       = 1.2;

  // Tracker state machine
  int    confirm_frames    = 3;     // frames to promote DETECTING → TRACKING
  double lost_timeout      = 0.4;   // seconds before TEMP_LOST → LOST
  int    track_grace_misses = 2;     // missed TRACKING frames before TEMP_LOST
  bool   fire_in_temp_lost = false;  // allow bounded coast-fire in TEMP_LOST
  int    temp_lost_fire_max = 3;     // max consecutive missed frames that may fire

  // EKF process noise (higher = trust measurements more, respond faster).
  // STATIC values — the old adaptive/NIS q_pos controller was removed: it
  // adapted only the horizontal-center noise, but a spinning target's center
  // is ~still, so it was reacting to yaw/PnP model error (not real maneuvers),
  // cranking q_pos up and making the lead overshoot. See INSTRUCTIONS.md.
  double q_pos             = 5.0;
  double q_yaw             = 10.0;
  double q_r               = 1e-6;

  // EKF measurement noise (higher = trust measurements less, smoother)
  // RADIAL (along line of sight / depth) position noise: PnP depth is the weak
  // axis, so this is large and grows with range.
  double r_pos_base        = 0.05;
  double r_pos_slope       = 0.04;
  // TANGENTIAL (perpendicular to the line of sight) position noise: bearing is
  // pixel-precise, so this is much smaller. Splitting radial vs tangential lets
  // the EKF track a translating target tightly in the lateral channel (good
  // FOLLOWING, less overshoot on stop) without depth noise destabilising it.
  // Set tang == base/slope to recover the old isotropic behaviour. WORKLOG §4b.
  double r_pos_tang_base   = 0.025;
  double r_pos_tang_slope  = 0.010;
  double r_yaw_base        = 0.05;
  double r_yaw_slope       = 0.005;
  double max_oblique_deg   = 65.0;
  // ── Yaw observability (U-shaped trust vs obliquity) ──
  // PnP yaw is poorly observable BOTH near face-on (small yaw barely changes the
  // projected shape — the planar flip regime) AND at extreme obliquity (few
  // pixels). The base model already distrusts the oblique side (1/|cos|^4); this
  // floor adds the near-face-on side: yaw noise is inflated by 1/max(sin^2(a),
  // floor), where a is the bearing-vs-face angle. Lower floor = distrust frontal
  // yaw MORE (stronger guard against phantom spin on a static frontal plate).
  // 0.05 ≈ up to 20x noise at dead-on; raise toward 1.0 to disable the U-shape.
  double yaw_facing_obs_floor = 0.05;

  // Velocity damping: v *= alpha each frame (at ref_freq Hz).
  // alpha_yaw should be 1.0 (NO spin damping): a 小陀螺 spins at a roughly
  // constant rate, so damping vyaw between updates only makes the spin estimate
  // decay and the 4-face phase prediction lag. Let q_yaw + the EKF set vyaw.
  double alpha_pos         = 0.99;
  double alpha_yaw         = 1.00;
  double alpha_coast       = 0.95;  // stronger damping when no measurement

  // Armor geometry
  double initial_radius    = 0.24;
  double radius_ema_alpha  = 0.05;
  // Known height step between armor pairs [m]. Faces 0,2 are at center z;
  // faces 1,3 are at center z + initial_dz. Set to the measured physical
  // offset (e.g. 0.05 for 5cm step). The EKF refines it after observing a
  // face jump, but seeding avoids shooting low on the raised pair before
  // a jump is seen. Set to 0.0 if all four faces are at the same height.
  double initial_dz        = 0.0;
  bool   adapt_dz_enable   = false;

  // ── BALLISTICS & PHYSICAL GEOMETRY ──
  double bullet_speed      = 25.0;  // [m/s] — MEASURE THIS on your robot!
  double gravity           = 9.8;
  double gimbal_height     = 0.325; // camera/gimbal pivot above ground [m]

  // Camera-to-barrel offset in the GIMBAL body frame [m].
  // Measured from the camera optical center to the barrel exit point.
  //
  //   barrel_offset_x: forward (positive = barrel is in front of camera)
  //   barrel_offset_y: left    (positive = barrel is left of camera)
  //   barrel_offset_z: up      (positive = barrel is above camera)
  //
  // WHY THIS MATTERS:
  //   The camera sees the target from one position but the bullet leaves
  //   from a different position. At close range (< 2m), a 10cm offset
  //   causes ~3° aiming error — bigger than the armor plate.
  //   At 5m it's ~1° (borderline). Without this correction, close-range
  //   shots systematically miss in one direction.
  //
  // HOW TO MEASURE:
  //   Use a ruler from the center of the ZED camera lens to the barrel
  //   center. Positive X = barrel is further forward, positive Z = barrel
  //   is higher than camera, positive Y = barrel is to the left.
  double barrel_offset_x   = 0.0;   // typically ~0.0 to 0.05
  double barrel_offset_y   = 0.0;   // typically ~0.0 (centered)
  double barrel_offset_z   = -0.10; // typically -0.05 to -0.15 (barrel below camera)

  // Fire gate
  double angular_window    = 0.30;
  double window_ref_dist   = 3.0;
  double min_fire_dist     = 0.3;
  double max_fire_dist     = 6.0;
  // Static-target facing cap [rad]. The strict facing window (margin >= 0) is a
  // SPINNER timing gate: only fire while a plate sweeps through facing-you. For
  // a NON-spinning target (|vyaw| < face_lookahead_min_vyaw) the plate sits at a
  // fixed angle and the gimbal is already locked on its center, so we fire on
  // lock and only reject a genuinely edge-on/garbage plate beyond this cap.
  // 0.6 rad ≈ 34°: a plate canted that far is still a solid center hit; past it
  // the plate is too oblique to trust. Bullet-economy safe: fire still needs
  // command_locked + range downstream. Set equal to angular_window to restore
  // the old strict-everywhere behavior. See WORKLOG §4.
  double static_facing_max = 0.6;

  // Face lookahead is aim planning, not fire permission. The fire window below
  // still decides whether a shot is safe; lookahead only lets the gimbal settle
  // on an incoming armor face before that face becomes fire-valid.
  bool   face_lookahead_enable   = true;
  double face_lookahead_min_vyaw = 0.35;  // [rad/s] minimum spin rate to plan incoming faces
  double face_lookahead_horizon  = 0.35;  // [s] max pre-aim time before entering fire window
  double face_switch_hysteresis  = 0.08;  // [rad] score advantage required to switch aim face
  // TODO: implement timestamp-aware face hold when computeAim() receives a
  // monotonic time source. Hysteresis is the current anti-flicker mechanism.
  double min_face_hold_time      = 0.10;  // [s]

  // Timing
  // ── PREDICTION HORIZON (measured-latency split) ──
  // When use_measured_latency is true, the horizon between the EKF state time
  // (capture stamp) and bullet exit is:
  //     measured pipeline latency (set per frame by the node, = now − capture)
  //   + actuation_latency (constant: serial TX + gimbal settle + muzzle exit).
  // time_bias is then IGNORED — it remains only as the fixed-bias fallback for
  // use_measured_latency = false. Rationale: a fixed bias is wrong whenever
  // detector load varies; at 150 RPM every 10 ms of error ≈ 4 cm at the plate.
  bool   use_measured_latency = true;
  double actuation_latency    = 0.020;
  double time_bias         = 0.10;
  double ref_freq          = 100.0;

  // Match gates
  double max_match_dist    = 0.5;
  // Legacy tuning surface retained for launch compatibility. Face changes are
  // now normal face-indexed associations; spin timing is observed from matched
  // face_idx transitions instead of raw yaw jumps.
  double yaw_jump_thresh   = 0.50;
  double maha_threshold    = 13.3;
  // IPPE alternate-solution hysteresis [chi-square units]. A planar plate has two
  // PnP yaw solutions; the tracker normally keeps whichever is closer to its
  // predicted yaw (lowest Mahalanobis). Near face-on the two are nearly tied, so
  // tiny prediction drift flips the choice frame-to-frame, injecting a yaw step
  // → phantom vyaw spike → fire flicker. The runner-up (alternate) solution must
  // now beat the primary (PnP best-reproj) by THIS much Mahalanobis to be chosen.
  // A genuine face rotation makes the primary clearly wrong and easily overcomes
  // it; only noise-level flips are suppressed. 0 = old behavior. See WORKLOG §4.
  double ippe_alt_penalty  = 1.0;

  // ── SPIN RATE ESTIMATION FROM FACE JUMP TIMING ──
  // vyaw estimated directly from dt between consecutive 90° face jumps.
  // This is far more accurate and converges faster than waiting for the
  // EKF to infer vyaw from noisy position measurements.
  // After 2 same-direction jumps, vyaw = (pi/2) / dt_between_jumps.
  bool   use_vyaw_from_timing  = true;
  double vyaw_timing_min_dt    = 0.020;  // ignore if jump faster than this [s]
  double vyaw_timing_max_dt    = 0.400;  // ignore if jump slower than this [s]
  double vyaw_conf_p_max       = 2.0;    // P(7,7) below this = vyaw trusted (gates 4-face)
  // ── ANTI-SPURIOUS-JUMP GATE (the reason IMM/PLL/fast-vyaw kept misfiring) ──
  // A "90° jump" in the MEASURED yaw is not always a real face switch: a PnP
  // flip or a wrong association produces the same signature. Trusting it
  // (old code: 80% blend + P(7,7)->1.0 after ONE jump) locked a WRONG vyaw with
  // high confidence -> 4-face prediction led the shot ~45° to the side.
  // We now only feed the timing estimator a jump that is geometrically credible:
  //   - PnP reprojection error below vyaw_timing_max_reproj, AND
  //   - yaw was NOT replaced by bearing (a faked yaw can't measure spin), AND
  //   - two consecutive same-direction estimates agree within
  //     vyaw_timing_consistency (relative). Only then do we blend/tighten P.
  double vyaw_timing_max_reproj   = 8.0;   // [px] max reproj err to trust a jump for timing
  double vyaw_timing_consistency  = 0.35;  // successive vyaw timing estimates must agree within ±35%

  // ── TARGET SWITCHING ──
  // POLICY: always prefer the closest enemy (highest hit probability).
  // Only stay on a farther target if:
  //   (a) it's confirmed TRACKING and cooldown hasn't elapsed, AND
  //   (b) the closer enemy isn't dramatically closer (within ratio)
  //
  // switch_range_ratio = 0.85 means: switch if new target is < 85% of
  // current range. This is aggressive — you almost always shoot the
  // closest robot. Set to 0.5 if you prefer to finish the current target.
  double switch_range_ratio = 0.85;
  // Hysteresis: don't switch back for N frames after switching.
  // Prevents jitter between two robots at similar range.
  int    switch_cooldown    = 10;  // ~0.33s at 30Hz
  // Spatial identity threshold [m]. If a new detection is within this distance
  // of the currently tracked target's predicted face position, it is treated
  // as the SAME physical robot (not a switch candidate). This is how the
  // tracker handles multiple enemies of the same color: class_id is color only,
  // identity comes from spatial proximity.
  // Tune up if robots can be back-to-back (false split into two tracks).
  // Tune down if two passing enemies get incorrectly merged into one track.
  double same_target_identity_dist = 1.0;
};

// =========================================================================
// Tracker — spinning-top EKF + aim solver + smart target switching.
//
// State: [xc, vxc, yc, vyc, za, vza, yaw, vyaw, r]  (9D)
// Observation: [xa, ya, za, yaw]  (4D)
//
// TARGET SWITCHING POLICY:
//   - Tracks the current target as long as it's TRACKING or TEMP_LOST
//   - Switches to a new target if:
//     (a) current target is LOST, OR
//     (b) a new detection is significantly closer (< switch_range_ratio × current range)
//     (c) and switch cooldown has elapsed
//   - This ensures you don't ignore a robot charging at you from 1m
//     while you're shooting at one at 5m
// =========================================================================
class Tracker
{
public:
  explicit Tracker(const TrackerConfig & cfg);

  /// Process one frame. dt is the real detection interval; defaults are tuned for fast auto-aim.
  /// All detections must be in odom frame (TF-transformed).
  /// now: current ROS time, used by the vyaw-from-timing estimator.
  void update(const std::vector<ArmorDetection> & detections, double dt,
              const rclcpp::Time & now);

  /// Tell the tracker where the robot is in odom frame. Used internally so
  /// targetRange() returns the distance from CAMERA to target (not from world
  /// origin), which is what shouldSwitch() and the range filter actually need.
  /// Call once per frame before update() and before computeAim().
  void setEgoPose(double robot_x, double robot_y)
  {
    ego_x_ = robot_x;
    ego_y_ = robot_y;
  }

  /// Translate the EKF target state by (dx, dy) in the world frame.
  /// Use when the node resets robot_x_/robot_y_ to zero due to drift cap.
  /// Without this shift the EKF would think the target jumped by exactly the
  /// reset amount and either reject all subsequent detections or go LOST.
  /// Pass dx = -robot_x_old, dy = -robot_y_old (the negative of the displacement
  /// that was reset away) — i.e. the target's coordinates also move by that
  /// amount in the new frame.
  void shiftWorldFrame(double dx, double dy)
  {
    x_(0) += dx;
    x_(2) += dy;
    // ego pose changes too — node will update it via setEgoPose next frame anyway,
    // but keep them consistent now to avoid one frame of bad targetRange().
    ego_x_ += dx;
    ego_y_ += dy;
    // Velocities are unaffected (purely translational shift).
  }

  /// Compute gimbal angles from current EKF state.
  /// robot_x/robot_y: current robot position in odom frame (from ego-motion integration).
  /// robot_vx/robot_vy: current robot velocity in odom/world frame [m/s].
  ///   Used to propagate the barrel position forward by (flight_time + time_bias),
  ///   so the aim accounts for our own motion during bullet flight. Pass 0,0 when
  ///   stationary or when ego_velocity_available is false.
  /// Required so the barrel-to-target vector is correct when the robot has moved
  /// and when the robot is still moving during bullet flight.
  AimResult computeAim(double current_yaw, double current_pitch,
                       double robot_x = 0.0, double robot_y = 0.0,
                       double robot_vx = 0.0, double robot_vy = 0.0) const;

  /// Per-frame measured pipeline latency [s]: (now − capture stamp) computed
  /// by the node at aim time. Clamped to [0, 0.25] so a missing/foreign-clock
  /// stamp can never blow up the prediction horizon. Call before computeAim().
  void setPipelineLatency(double s)
  {
    pipeline_latency_ = std::clamp(s, 0.0, 0.25);
  }
  double pipelineLatency() const { return pipeline_latency_; }

  enum State { LOST, DETECTING, TRACKING, TEMP_LOST };
  State state() const { return state_; }
  bool fireStatePermits() const;
  const Eigen::VectorXd & ekfState() const { return x_; }
  double radius() const { return radius_; }
  double otherRadius() const { return other_radius_; }
  double dz() const { return dz_; }
  std::string targetId() const { return target_id_; }
  /// Monotonically increasing counter, bumped on every initFromDetection.
  /// The node compares this across frames to detect target switches.
  int targetGeneration() const { return target_generation_; }
  /// Current tracked target range (for debug / visualization)
  double targetRange() const;
  const TrackerDebugInfo & debugInfo() const { return debug_info_; }
  double pVyaw() const { return (P_.rows() > 7 && P_.cols() > 7) ? P_(7, 7) : 0.0; }
  bool phaseConfident() const;
  bool vyawConfident() const { return phaseConfident(); }
  int consecutiveSameDirJumps() const { return consecutive_same_dir_jumps_; }
  /// Static horizontal process noise (kept as a method so the debug message
  /// publisher does not need to reach into cfg_). The adaptive scaler is gone.
  double qPos() const { return cfg_.q_pos; }

private:
  struct AssociationHypothesis
  {
    bool valid = false;
    const ArmorDetection * det = nullptr;
    int face_idx = -1;
    double yaw_meas = 0.0;
    bool yaw_valid = false;
    int yaw_hypothesis = 0;  // 0=primary, 1=IPPE alternate, -1=position-only
    double mahalanobis = std::numeric_limits<double>::infinity();
    double position_error = std::numeric_limits<double>::infinity();
    double yaw_error = std::numeric_limits<double>::infinity();
  };

  void ekfPredict(double dt);
  Eigen::VectorXd ekfUpdateFace(
    const ArmorDetection & det,
    int face_idx,
    double yaw_meas,
    bool yaw_valid);
  double ekfMahalanobisFace(
    const ArmorDetection & det,
    int face_idx,
    double yaw_meas,
    bool yaw_valid) const;
  Eigen::Matrix4d measurementNoise(
    const ArmorDetection & det,
    double face_yaw,
    bool yaw_valid) const;
  AssociationHypothesis associateDetections(
    const std::vector<ArmorDetection> & detections) const;
  FacePrediction predictFace(const Eigen::VectorXd & x, int face_idx, double t = 0.0) const;
  Eigen::Vector3d armorFromState(const Eigen::VectorXd & x) const;
  void initFromDetection(const ArmorDetection & det);
  void observeFaceAssociation(
    int matched_face_idx,
    const ArmorDetection & det,
    const rclcpp::Time & now,
    bool yaw_valid);
  void updateRadiusFromAssociation(
    const ArmorDetection & det,
    int face_idx,
    bool oblique);
  double unwrapYaw(double raw_yaw);

  struct BallisticResult { double pitch, flight_time; bool valid; };
  BallisticResult solveBallistic(double ground_dist, double dz) const;

  /// Non-flight part of the prediction horizon (EKF state time → bullet exit).
  double predictionBias() const
  {
    return cfg_.use_measured_latency ? (pipeline_latency_ + cfg_.actuation_latency)
                                     : cfg_.time_bias;
  }

  /// Check if we should switch to a closer target
  bool shouldSwitch(const ArmorDetection & candidate) const;

  TrackerConfig cfg_;
  Eigen::VectorXd x_;
  Eigen::MatrixXd P_, P0_;

  State state_ = LOST;
  int detect_count_ = 0;
  int lost_count_ = 0;
  int lost_thresh_ = 12;
  int switch_cooldown_counter_ = 0;
  int target_generation_ = 0;
  // Robot position in odom frame (set by node each frame via setEgoPose).
  // Used so targetRange() returns the camera-to-target distance, not the
  // world-origin-to-target distance — which would be wrong once ego-motion
  // has moved the robot.
  double ego_x_ = 0.0;
  double ego_y_ = 0.0;

  // Measured pipeline latency for the current frame (set by the node).
  double pipeline_latency_ = 0.0;

  double radius_ = 0.24;
  double other_radius_ = 0.28;
  double dz_ = 0.0;
  bool dz_initialized_ = false;
  double last_yaw_ = 0.0;
  std::string target_id_;
  TrackerDebugInfo debug_info_;
  // Previous face selected by computeAim(). This is only face scheduling
  // hysteresis inside one tracked robot; it never resets or switches the EKF.
  mutable int last_aim_face_idx_ = 0;

  int last_matched_face_idx_ = -1;
  rclcpp::Time last_face_assoc_time_;
  bool last_face_assoc_time_valid_ = false;
  bool phase_timing_confident_ = false;

  // ── Spin rate estimation from face jump timing ──
  // Tracks the ROS time of the last face jump and the direction.
  // After two consecutive same-direction jumps, vyaw = (pi/2) / dt.
  // This is more accurate and faster-converging than EKF-only estimation,
  // especially during spin-up and at 200-300 RPM.
  rclcpp::Time last_jump_time_;
  bool last_jump_time_valid_ = false;
  double last_jump_dir_ = 0.0;   // sign of last jump (+1 CCW, -1 CW)
  int consecutive_same_dir_jumps_ = 0;
  // Last vyaw estimate from face-jump timing — used to require two consecutive
  // CONSISTENT estimates before trusting the spin rate (anti-spurious-jump gate).
  double last_vyaw_timing_est_ = 0.0;
};

}  // namespace autoaim
#endif
