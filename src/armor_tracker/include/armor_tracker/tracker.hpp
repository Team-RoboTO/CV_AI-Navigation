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

  void update(const Armors::SharedPtr & armors_msg);

  void setMaxYawObliqueAngle(double angle_rad) { max_yaw_oblique_angle_ = angle_rad; }

  ExtendedKalmanFilter ekf;

  int tracking_thres;
  int lost_thres;

  enum State {
    LOST,
    DETECTING,
    TRACKING,
    TEMP_LOST,
  } tracker_state;

  std::string tracked_id;
  Armor tracked_armor;
  ArmorsNum tracked_armors_num;

  double info_position_diff;
  double info_yaw_diff;
  double info_yaw_innov_signed = 0.0;
  Eigen::Vector3d info_position_innov = Eigen::Vector3d::Zero();
  double info_face_angle = 0.0;  // obliqueness [rad] (0=face-on, π/2=edge-on)

  Eigen::VectorXd measurement;
  bool measurement_valid = false;  // true only when measurement was fused this frame

  Eigen::VectorXd target_state;

  // To store another pair of armors message
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
  double dz_adapt_alpha_ = 0.2;       // EMA factor for dz
  bool dz_initialized_ = false;       // First dz measurement gets full value, subsequent use EMA
  ScalarKF r_active_kf_;              // Scalar KF for the currently-tracked pair's radius
  ScalarKF r_other_kf_;               // Scalar KF for the other pair's radius

private:
  void initEKF(const Armor & a);

  void updateArmorsNum(const Armor & a);

  void handleArmorJump(const Armor & a);

  double orientationToYaw(const geometry_msgs::msg::Quaternion & q);

  Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd & x);

  double max_match_distance_;
  double max_match_yaw_diff_;
  double max_track_range_;
  double max_yaw_oblique_angle_;  // face angle beyond which yaw gate is bypassed [rad]

  int detect_count_ = 0;
  int lost_count_ = 0;

  double last_yaw_ = 0.0;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__TRACKER_HPP_
