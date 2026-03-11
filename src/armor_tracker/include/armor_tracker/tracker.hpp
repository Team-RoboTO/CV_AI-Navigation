#ifndef ARMOR_PROCESSOR__TRACKER_HPP_
#define ARMOR_PROCESSOR__TRACKER_HPP_

// Eigen
#include <Eigen/Eigen>

// ROS
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

// STD
#include <memory>
#include <string>

#include "armor_tracker/extended_kalman_filter.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"

namespace rm_auto_aim
{

// ---------------------------------------------------------------------------
// Scalar Kalman Filter — 1D KF for quasi-static parameters like orbit radius.
//
// Model:  x_{k+1} = x_k + w,   z_k = x_k + v
//   where w ~ N(0, Q) is process noise and v ~ N(0, R) is measurement noise.
//   The state is assumed constant (random walk with tiny Q), making this
//   suitable for slowly-changing geometric parameters like armor orbit radius.
//
// Why not just an EMA?
//   A KF provides adaptive gain:
//   - When P is large (e.g. right after init or a pair swap), K → 1 and
//     new measurements are trusted heavily → fast initial convergence.
//   - As P shrinks (steady-state), K → Q/(Q+R) ≈ tiny → strong noise rejection.
//   An EMA has fixed gain α that can't adapt — it either converges slowly
//   or oscillates in steady state.
//
// Usage in the tracker:
//   r_active_kf_ tracks the radius of the currently-visible armor pair (0/2).
//   r_other_kf_  tracks the other pair's radius (1/3).
//   On 90° armor-jump, the two KFs are swapped — the newly-visible pair
//   inherits the covariance that grew while it was unobserved, giving it
//   a naturally higher gain for fast re-convergence.
// ---------------------------------------------------------------------------
struct ScalarKF
{
  double x = 0.26;        // State estimate (e.g., radius in metres)
  double P = 0.0064;      // Estimate variance (σ² ≈ 0.08² m²)
  double Q = 3.3e-8;      // Process noise: how much x can drift per step
  double R = 0.0004;      // Measurement noise: PnP radius noise (σ ≈ 2 cm)

  // Predict: grow variance by Q each frame (no state change — constant model)
  void predict() { P += Q; }

  // Update: fuse measurement z → compute Kalman gain K, correct x and shrink P
  //   K = P / (P + R)      — gain (0..1): ratio of prediction to total uncertainty
  //   x += K * (z - x)     — shift estimate towards measurement
  //   P = (1 - K) * P      — reduce uncertainty (we gained information from z)
  void update(double z)
  {
    double denom = P + R;
    if (denom < 1e-15) return;  // guard: P=R=0 would produce NaN
    double K = P / denom;
    x += K * (z - x);
    P = (1.0 - K) * P;
  }

  // Hard reset: used on tracker init and when radius is clamped out-of-bounds
  void reset(double x0, double P0) { x = x0; P = P0; }
};

// Number of armor plates on a given robot type.
// Determines face spacing (2π/n), jump thresholds, and whether r2/dz apply.
enum class ArmorsNum { NORMAL_4 = 4, BALANCE_2 = 2, OUTPOST_3 = 3 };

// ---------------------------------------------------------------------------
// Tracker — "Spinning Top" model for a RoboMaster robot.
//
// Geometric model:
//   The robot is modelled as a point (xc, yc) on the ground with n armor
//   plates arranged in a circle of radius r around it.  Each plate sits at
//   a fixed angular offset from the robot heading (yaw):
//
//       plate_position(i) = center − r · [cos(yaw + i·2π/n), sin(yaw + i·2π/n)]
//
//   For 4-armor robots, alternating plates have different radii (r1, r2) and
//   heights (za, za+dz), forming an elliptical arrangement.
//
// EKF state vector (9D):
//   [xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r]
//    │    │     │    │     │    │     │     │      └─ orbit radius (active pair)
//    │    │     │    │     │    │     │     └──────── yaw rate (rad/s)
//    │    │     │    │     │    │     └────────────── robot heading (rad)
//    │    │     │    │     │    └──────────────────── vertical velocity
//    │    │     │    │     └───────────────────────── armor plate Z (height)
//    │    │     │    └────────────────────────────── Y velocity of center
//    │    │     └─────────────────────────────────── Y position of center
//    │    └───────────────────────────────────────── X velocity of center
//    └────────────────────────────────────────────── X position of center
//
// State machine:
//   LOST → DETECTING → TRACKING ⇌ TEMP_LOST → LOST
//   See update() for the full transition logic and threshold descriptions.
//
// Key algorithms:
//   - Mahalanobis gating for measurement association (chi² 4-DOF)
//   - Armor-jump detection (yaw gate + obliquity bypass)
//   - Yaw-projected radius measurement (scalar KFs for r1, r2)
//   - Adaptive velocity damping (driven by innovation signal)
// ---------------------------------------------------------------------------
class Tracker
{
public:
  Tracker(double max_match_distance, double max_track_range);

  using Armors = auto_aim_interfaces::msg::Armors;
  using Armor = auto_aim_interfaces::msg::Armor;

  void init(const Armors::SharedPtr & armors_msg);

  void initFromArmor(const Armor & armor);

  // Predict-only step: runs EKF predict + scalar KF predict.
  // Must be called before computeAssignmentCost() and before update().
  // After calling this, update() will skip its internal predict.
  Eigen::VectorXd predictStep();

  // Compute Mahalanobis distance from current prediction to an armor detection.
  // Does NOT modify yaw unwrapping state (safe for cross-tracker comparison).
  // Must be called after predictStep().
  double computeAssignmentCost(const Armor & armor);

  void update(const Armors::SharedPtr & armors_msg);

  void setMaxYawObliqueAngle(double angle_rad) { max_yaw_oblique_angle_ = angle_rad; }

  // The 9D EKF instance — one per tracker, created by createTracker() factory.
  // Each tracker's EKF lambdas capture a raw pointer to this Tracker, so the
  // process model f() reads per-tracker damping_alpha values.
  ExtendedKalmanFilter ekf;

  // tracking_thres: consecutive matched frames required to promote DETECTING → TRACKING.
  //   WHY NOT 1: A single detection could be a false positive (YOLO sees a reflection,
  //   overlapping bbox, or glare).  Requiring N consecutive matches means the target
  //   must be consistently visible — this eliminates transient false positives at the
  //   cost of N×33ms (≈170ms at thres=5) reaction delay for new targets.
  int tracking_thres;

  // lost_thres: consecutive missed frames allowed in TEMP_LOST before transitioning to LOST.
  //   Computed dynamically as lost_time_thres_ / dt_ (time-based, not frame-count-based)
  //   so the timeout is consistent regardless of frame rate.
  int lost_thres;

  // ---------------------------------------------------------------------------
  // TRACKER STATE MACHINE
  //
  // Why 4 states instead of 2 (tracked / not-tracked)?
  //
  //   LOST → DETECTING → TRACKING ⇌ TEMP_LOST → LOST
  //
  //   LOST:       No target.  Waiting for init() or initFromArmor().
  //               The tracker does nothing; it's either new or pruned.
  //
  //   DETECTING:  Candidate target seen.  EKF is running but output is
  //               suppressed (tracking=false in Target msg).  Must accumulate
  //               tracking_thres consecutive matches to promote.  Any single
  //               miss drops back to LOST — the candidate was unreliable.
  //               PURPOSE: Prevents false-positive YOLO detections from
  //               triggering gimbal movement or fire commands.
  //
  //   TRACKING:   Confirmed target.  EKF output is published (tracking=true).
  //               The trajectory solver uses this for aiming and firing.
  //               A single miss transitions to TEMP_LOST (not LOST).
  //
  //   TEMP_LOST:  Target briefly occluded.  EKF coasts on prediction only
  //               (predict-without-update).  tracking=true still so the
  //               trajectory solver keeps aiming but fire is suppressed.
  //               If a match reappears, snap back to TRACKING immediately.
  //               After lost_thres frames without a match, transition to LOST.
  //               PURPOSE: Graceful degradation during brief YOLO dropouts
  //               (1-3 frames due to partial occlusion, motion blur, etc.)
  //               without the cost of re-confirming through DETECTING.
  // ---------------------------------------------------------------------------
  enum State {
    LOST,
    DETECTING,
    TRACKING,
    TEMP_LOST,
  } tracker_state;

  // Unique integer ID assigned by the tracker node (monotonically increasing).
  // Used to key per-tracker data (acceleration EMA, last measurement time).
  // Different from tracked_id (which is the YOLO class string like "3").
  int tracker_id = -1;

  // YOLO class ID of the target (e.g. "0"=blue, "3"=red).  Set at init time
  // and used to filter detections during association (only same-color armors
  // can be associated to this tracker).
  std::string tracked_id;

  // The most recent matched armor detection (full Armor msg with pose, type, etc.).
  // Used for:
  //   - Optimal bbox publishing (back-mapping to YOLO detection index)
  //   - RViz marker sizing (armor type → plate dimensions)
  //   - Debug logging of the detection that was actually fused
  Armor tracked_armor;

  // Robot type (determines face count and geometry):
  //   NORMAL_4: 4 faces, 90° spacing, alternating r1/r2 and dz
  //   BALANCE_2: 2 faces, 180° spacing, single radius
  //   OUTPOST_3: 3 faces, 120° spacing, single radius
  // Currently always NORMAL_4 (robot-type detection not yet implemented).
  ArmorsNum tracked_armors_num;

  // ---------------------------------------------------------------------------
  // DIAGNOSTIC FIELDS — computed each frame, consumed by the tracker node for:
  //   1. Adaptive damping (info_position_innov, info_yaw_innov_signed)
  //   2. TrackerInfo debug message (info_position_diff, info_yaw_diff, info_face_angle)
  //   3. Not used for EKF math — purely observational / downstream.
  //
  // info_position_diff:     Euclidean distance between predicted and measured armor [m]
  // info_yaw_diff:          Absolute yaw difference |predicted_yaw - measured_yaw| [rad]
  // info_yaw_innov_signed:  Signed yaw innovation (measured - predicted) [rad]
  //                         Sign matters for damping: positive = target ahead of prediction
  // info_position_innov:    3D vector (measured - predicted) armor position [m]
  //                         Used for overshoot detection: dot(innov, velocity) < 0 means
  //                         the EKF is overshooting the target (prediction went too far)
  // info_face_angle:        Obliqueness angle [rad] (0=face-on, π/2=edge-on)
  //                         Published in TrackerInfo for PnP quality monitoring
  // ---------------------------------------------------------------------------
  double info_position_diff;
  double info_yaw_diff;
  double info_yaw_innov_signed = 0.0;
  Eigen::Vector3d info_position_innov = Eigen::Vector3d::Zero();
  double info_face_angle = 0.0;

  // The 4D measurement vector [xa, ya, za, yaw] that was fed into ekf.update()
  // this frame.  Valid only when measurement_valid = true.
  // Used by the tracker node to fill TrackerInfo debug messages.
  Eigen::VectorXd measurement;

  // True when a measurement was successfully fused into the EKF this frame.
  // False when the tracker coasted on prediction (no match, or match rejected
  // by Mahalanobis gate).  Used by the node to:
  //   - Update tracker_last_meas_times_ (only when a real measurement was fused)
  //   - Decide whether to publish measurement data in TrackerInfo
  bool measurement_valid = false;

  // The current best estimate of the 9D state vector.
  //
  // LIFECYCLE:
  //   1. Initialized by initEKF() from the first detection
  //   2. Updated by predictStep() → target_state = EKF prediction
  //   3. Corrected by update() → target_state = EKF posterior (if matched)
  //      or left as prediction (if no match)
  //   4. Modified by handleArmorJump() (yaw snap, za snap, r swap)
  //   5. Safety-clamped (r bounds, v_yaw max, yaw normalization)
  //   6. Published via fillTargetMsg() in the tracker node
  //
  // IMPORTANT: target_state may differ from ekf.x_post after handleArmorJump()
  // or safety clamps.  ekf.setState() is called to sync them before the next
  // predict() cycle.
  Eigen::VectorXd target_state;

  // ---------------------------------------------------------------------------
  // SECONDARY PAIR GEOMETRY — parameters for the OTHER armor pair on a
  // 4-armor robot.  The active pair's radius is in state[8] (managed by
  // r_active_kf_); these track the pair that's currently NOT visible.
  //
  //   dz:        Height offset between the two armor pairs [m].
  //              Pair A at height za, Pair B at height za + dz.
  //              Estimated via EMA (not Kalman-filtered) because dz is
  //              only observable during pair switches or when both faces
  //              are simultaneously visible — too few updates per second
  //              for a KF to offer a meaningful advantage over EMA.
  //              Typical values: 0.05-0.15m (depends on robot chassis design).
  //
  //   another_r: Radius of the other pair [m] (from r_other_kf_.x).
  //              Updated when both faces are visible simultaneously, or
  //              when the radius KFs are swapped during a 90° jump.
  //              Published in Target.msg as radius_2 for the trajectory
  //              solver's alternating-geometry face prediction.
  //
  // WHY NOT IN THE EKF:
  //   Both are quasi-static (change only with robot design, not dynamics).
  //   Including them in the 9D state would add 2 more dimensions, complicate
  //   all Jacobians, and require careful handling of the discontinuity when
  //   the pair switches.  Tracking them externally (ScalarKF + EMA) is
  //   simpler and sufficient for the current accuracy requirements.
  //   See the project assessment (delightful-sprouting-eagle plan) for the
  //   full cost/benefit analysis of a unified EKF approach.
  // ---------------------------------------------------------------------------
  double dz = 0.0, another_r = 0.26;

  // Geometry adaptation — r1, r2, dz are quasi-fixed; estimated by scalar
  // Kalman filters that provide adaptive gain (fast initial convergence,
  // low steady-state noise).  Armor plates sit on an ellipse: r1 ≠ r2.
  double initial_r1_ = 0.22;          // Initial radius for pair 0/2 (shorter axis)
  double initial_r2_ = 0.30;          // Initial radius for pair 1/3 (longer axis)
  double r_kf_Q_ = 3.3e-8;           // Process noise for radius KFs
  double r_kf_R_ = 0.0004;           // Measurement noise for radius KFs (sigma≈2cm)
  double r_kf_P_init_ = 0.0064;      // Initial covariance for radius KFs (sigma≈8cm)
  double r_adapt_max_dist_ = 4.0;     // Only adapt r when target range < this [m]
  double dz_adapt_alpha_ = 0.05;      // EMA factor for dz (low = smooth, high = reactive)
  double r_yaw_uncertainty_scale_ = 50.0;  // Inflate radius KF R by (1 + scale * yaw_var)
  bool dz_initialized_ = false;       // First dz measurement gets full value, subsequent use EMA
  ScalarKF r_active_kf_;              // Scalar KF for the currently-tracked pair's radius
  ScalarKF r_other_kf_;               // Scalar KF for the other pair's radius

  // Configurable thresholds (exposed as ROS parameters)
  double v_yaw_max_ = 15.0;
  double maha_match_threshold_ = 13.3;
  double maha_jump_threshold_ = 20.0;

  // Secondary face fusion: fuse secondary armor face into 9D EKF
  bool use_secondary_face_fusion = true;
  double secondary_r_inflation = 2.0;
  double secondary_maha_threshold = 13.3;
  // R computation parameters (copied from node for inline R_2 calculation)
  double r_xyz_base_ = 0.04;
  double r_xyz_slope_ = 0.03;
  double r_yaw_base_ = 0.05;
  double r_yaw_slope_ = 0.002;
  double r_yaw_angle_power_ = 4.0;
  double max_yaw_oblique_deg_ = 65.0;

  // Per-tracker damping alphas (set by the node each frame before predict)
  double xyz_damping_alpha = 0.95;
  double yaw_damping_alpha = 0.98;

  // ---------------------------------------------------------------------------
  // Prior velocities — captured from the EKF prediction (x_pri) BEFORE the
  // measurement update, i.e. the EKF's best velocity guess using only the
  // motion model (no new detection fused yet).
  //
  // WHY STORED: To compute innovation-based acceleration in the tracker node:
  //   raw_accel = (v_posterior - v_prior) / dt
  //
  // This difference is exactly the velocity correction that the Kalman gain K
  // applied during the EKF update.  It represents how much the EKF "thinks"
  // the target accelerated — filtered by the Kalman gain, which adapts to
  // measurement quality (noisy detections → small K → small correction →
  // naturally conservative acceleration estimate; clean detections → larger K
  // → more responsive acceleration estimate).
  //
  // The old approach (finite-differencing v_posterior[k] - v_posterior[k-1])
  // was noisier because both posterior velocities carry full measurement noise,
  // and the difference amplifies it (noise adds in quadrature when subtracting
  // correlated signals).  The innovation-based approach only has one noise
  // contribution (through the Kalman gain), giving ~3× lower noise.
  //
  // Set in predictStep() and in update()'s internal predict path.
  // Read in tracker_node's fillTargetMsg() to compute published acceleration.
  // ---------------------------------------------------------------------------
  double prior_vx = 0.0, prior_vy = 0.0, prior_vz = 0.0;

private:
  // Bootstrap the EKF state from a single armor detection.
  // Back-calculates robot center from armor position + assumed radius.
  void initEKF(const Armor & a);

  // Determine robot type (NORMAL_4, BALANCE_2, OUTPOST_3) from the armor detection.
  // Currently always returns NORMAL_4 (robot-type classifier not yet implemented).
  void updateArmorsNum(const Armor & a);

  // Handle face-switch events: yaw snap, radius KF swap, dz update,
  // divergence detection + hard reset, covariance inflation.
  // Called when position is close but yaw differs by > max_match_yaw_diff_.
  void handleArmorJump(const Armor & a);

  // Convert quaternion orientation → continuous yaw angle.
  // Uses shortest_angular_distance to unwrap relative to last_yaw_,
  // producing a monotonic yaw signal that doesn't wrap at ±π.
  // IMPORTANT: Modifies last_yaw_ state — only call for the selected armor.
  double orientationToYaw(const geometry_msgs::msg::Quaternion & q);

  // Convert center-based EKF state → armor plate position.
  // Inverse of the observation model: armor = center − r·[cos(yaw), sin(yaw)]
  Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd & x);

  // ---------------------------------------------------------------------------
  // DATA ASSOCIATION THRESHOLDS
  //
  // max_match_distance_:  Euclidean distance gate [m] between predicted armor
  //                       position and detected armor position.  If the
  //                       detection is farther than this, it's classified as
  //                       either a MISS (wrong target) or a potential JUMP
  //                       (same target, different face).
  //                       Typical: 0.15m (tight; only accepts the right face).
  //
  // max_match_yaw_diff_:  Yaw angle gate [rad] for same-face classification.
  //                       If |predicted_yaw - measured_yaw| < this, the
  //                       detection is the same face (MATCH).  If larger and
  //                       position is close, it's a face switch (JUMP).
  //                       Set to π/3 (60°) — midpoint between PnP noise
  //                       (±15°) and 90° face spacing.
  //
  // max_track_range_:     Maximum 3D range [m] to maintain tracking.  Beyond
  //                       this, the target is dropped to LOST (PnP accuracy
  //                       and fire solution reliability degrade too much).
  //
  // max_yaw_oblique_angle_: Face obliqueness [rad] beyond which the yaw gate
  //                         is bypassed.  When the armor is nearly edge-on
  //                         (>65°), PnP yaw is unreliable noise — we still
  //                         fuse XYZ position but ignore yaw entirely.
  // ---------------------------------------------------------------------------
  double max_match_distance_;
  double max_match_yaw_diff_;
  double max_track_range_;
  double max_yaw_oblique_angle_;

  // State machine counters:
  //   detect_count_: consecutive matched frames in DETECTING state.
  //                  Resets to 0 on any miss.  Promotes to TRACKING when
  //                  it exceeds tracking_thres.
  //   lost_count_:   consecutive missed frames in TEMP_LOST state.
  //                  Resets to 0 when a match reappears.  Transitions to
  //                  LOST when it exceeds lost_thres.
  int detect_count_ = 0;
  int lost_count_ = 0;

  // Yaw unwrapping state: the previous frame's continuous yaw value.
  // orientationToYaw() computes new_yaw = last_yaw_ + shortest_angular_distance(...)
  // to produce a smooth, unwrapped yaw signal.  Reset to 0 in initEKF().
  double last_yaw_ = 0.0;

  // DUAL-PATH PREDICT ARCHITECTURE:
  //   When the tracker node runs multi-tracker data association, it calls
  //   predictStep() on ALL trackers FIRST, then assigns detections, then
  //   calls update().  The predicted_ flag prevents update() from running
  //   a second predict() (which would double-count the time evolution).
  //
  //   Flow A (multi-tracker):  predictStep() → predicted_=true → update() skips predict
  //   Flow B (legacy single):  update() → predicted_=false → update() does its own predict
  bool predicted_ = false;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__TRACKER_HPP_
