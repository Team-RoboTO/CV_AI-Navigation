#ifndef AUTOAIM_V2__TRACKER_HPP_
#define AUTOAIM_V2__TRACKER_HPP_

#include <Eigen/Dense>
#include <vector>

#include "autoaim_v2/types.hpp"

namespace aim
{

struct TrackerParams
{
  // ── Process noise (continuous white-noise acceleration PSD) ──
  double q_accel_xy = 60.0;   // [m^2/s^4] translation: high = chase movers
  double q_accel_z = 4.0;     // [m^2/s^4] plates barely move vertically
  double q_alpha = 300.0;     // [rad^2/s^4] spin acceleration
  double q_r = 2e-5;          // random-walk rate of r / l / h
  double q_l = 4e-6;
  double q_h = 4e-6;

  // ── Measurement noise ──
  double r_bearing = 0.004;       // [rad] yaw & pitch of plate center
  double r_dist_base = 0.03;      // [m]
  double r_dist_slope = 0.05;     // [m per m of range]
  double r_theta_min = 0.04;      // [rad] floor for the plate-angle sigma

  // ── Gates / state machine ──
  double maha_gate = 16.0;        // chi2_4 ~ 13.3 @ 99%; a bit loose
  double reinit_pos_gate = 1.2;   // [m] farther than this from every face
                                  // counts as divergence
  int reinit_after = 3;           // consecutive divergent frames -> reinit
  int confirm_frames = 2;         // DETECTING -> TRACKING
  double lost_timeout = 0.6;      // [s] TEMP_LOST -> LOST
  double coast_vel_damping = 0.85;  // per-second velocity keep-factor in coast
                                    // (omega is NOT damped: spinners keep
                                    // spinning behind walls)

  // ── Geometry priors / clamps ──
  double initial_r = 0.24;
  double r_min = 0.10, r_max = 0.35;
  double l_abs_max = 0.15;
  double h_abs_max = 0.12;
  double omega_abs_max = 52.4;    // 500 RPM: physical bound

  double gimbal_height = 0.42;    // used in h(x) z reference
};

// Whole-car EKF, 11 states:
//   x = [xc vx yc vy zc vz theta omega r l h]
// Plate i (i = 0..3): phi_i = theta + i*pi/2, r_i = r (+l if i odd),
//   z_i = zc (+h if i odd),
//   plate_i = [xc - r_i cos(phi_i), yc - r_i sin(phi_i), z_i].
// Observation per plate: z = [yaw, pitch, dist, theta_i] with yaw/pitch/dist
// spherical coordinates of the plate center relative to the CAMERA origin.
// Rationale for spherical: PnP depth noise is 16-63x bearing noise; observing
// Cartesian xyz with a shared R poisons the mm-grade angles (DESIGN.md §3).
class Tracker
{
public:
  enum State { LOST, DETECTING, TRACKING, TEMP_LOST };

  explicit Tracker(const TrackerParams & p);

  // armors: all enemy-color plates in this frame (already solved to world).
  // cam0: camera origin in world at the capture time (for h(x)).
  void update(const std::vector<ArmorWorld> & armors, TimePoint t,
              const Eigen::Vector3d & cam0);

  State state() const { return state_; }
  bool tracking() const { return state_ == TRACKING || state_ == TEMP_LOST; }
  const Eigen::VectorXd & x() const { return x_; }
  const Eigen::MatrixXd & P() const { return P_; }
  TimePoint stamp() const { return t_; }
  int generation() const { return generation_; }
  int last_matched_face() const { return last_face_; }
  int matched_count() const { return matched_count_; }

  // Propagate a copy of the state dt seconds forward (constant velocity /
  // constant spin, no covariance) — used by the aimer for the impact-time
  // prediction.
  Eigen::VectorXd predict_state(double dt) const;

  static Eigen::Vector3d armor_pos(const Eigen::VectorXd & x, int i);
  static double armor_theta(const Eigen::VectorXd & x, int i);

private:
  void ekf_predict(double dt);
  bool ekf_update(const ArmorWorld & a, const Eigen::Vector3d & cam0);
  double mahalanobis(const ArmorWorld & a, int face,
                     const Eigen::Vector3d & cam0) const;
  int associate(const ArmorWorld & a, const Eigen::Vector3d & cam0) const;
  void init_from(const ArmorWorld & a, TimePoint t);
  void clamp_state();

  Eigen::Vector4d h_of(const Eigen::VectorXd & x, int face,
                       const Eigen::Vector3d & cam0) const;
  Eigen::Matrix<double, 4, 11> H_of(const Eigen::VectorXd & x, int face,
                                    const Eigen::Vector3d & cam0) const;
  Eigen::Matrix4d R_of(const ArmorWorld & a) const;

  TrackerParams p_;
  Eigen::VectorXd x_;
  Eigen::MatrixXd P_, P0_;
  TimePoint t_{};
  State state_ = LOST;
  int confirm_count_ = 0;
  int divergent_count_ = 0;
  int generation_ = 0;
  int last_face_ = 0;
  int matched_count_ = 0;
  double temp_lost_since_ = 0;  // seconds accumulated in TEMP_LOST
};

}  // namespace aim

#endif  // AUTOAIM_V2__TRACKER_HPP_
