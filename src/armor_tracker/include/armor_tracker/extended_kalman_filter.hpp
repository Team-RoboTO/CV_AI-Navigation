#ifndef ARMOR_PROCESSOR__KALMAN_FILTER_HPP_
#define ARMOR_PROCESSOR__KALMAN_FILTER_HPP_

#include <Eigen/Dense>
#include <functional>

namespace rm_auto_aim
{

// ---------------------------------------------------------------------------
// Extended Kalman Filter (EKF) for nonlinear state estimation.
//
// Background:
//   A standard Kalman filter assumes linear dynamics.  Real robotics systems
//   (like the spinning-top tracker) are nonlinear — the armor plate position
//   is a trigonometric function of the robot center + yaw + radius.  The EKF
//   linearizes these nonlinear functions around the current estimate each step,
//   then applies the standard Kalman equations on the linearized system.
//
// Two-step cycle (called once per camera frame, ~30 Hz):
//
//   PREDICT — propagate state forward in time using the process model f(x):
//     x_pri  = f(x_post)                       // predicted state
//     P_pri  = F·P_post·Fᵀ + Q                 // predicted covariance
//     where F = ∂f/∂x (Jacobian), Q = process noise
//
//   UPDATE  — fuse a new measurement z into the predicted state:
//     y = z − h(x_pri)                          // innovation (measurement residual)
//     S = H·P_pri·Hᵀ + R                        // innovation covariance
//     K = P_pri·Hᵀ·S⁻¹                          // Kalman gain
//     x_post = x_pri + K·y                       // corrected state
//     P_post = (I − K·H)·P_pri·(I−K·H)ᵀ + K·R·Kᵀ   // Joseph form (numerically stable)
//     where H = ∂h/∂x (Jacobian), R = measurement noise
//
// In this project:
//   State  x = [xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r]  (9D)
//   Meas.  z = [xa, ya, za, yaw]                                 (4D)
//   where (xc,yc) = robot center, (xa,ya) = armor plate position,
//   r = orbit radius, yaw = robot heading, v_* = velocities.
//
// The process model f() is a damped constant-velocity model (see tracker_node.cpp).
// The observation model h() converts center-based state to armor-plate coordinates:
//   xa = xc − r·cos(yaw),  ya = yc − r·sin(yaw)
// ---------------------------------------------------------------------------
class ExtendedKalmanFilter
{
public:
  ExtendedKalmanFilter() = default;

  // Function type aliases for the pluggable nonlinear functions:
  //   VecVecFunc:  Rⁿ → Rᵐ   (state transition f, observation h)
  //   VecMatFunc:  Rⁿ → Rᵐˣⁿ (Jacobians J_f, J_h; measurement noise R)
  //   VoidMatFunc: void → Rⁿˣⁿ (process noise Q — depends only on dt, not state)
  using VecVecFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd &)>;
  using VecMatFunc = std::function<Eigen::MatrixXd(const Eigen::VectorXd &)>;
  using VoidMatFunc = std::function<Eigen::MatrixXd()>;

  // Construct with all required functions and initial covariance P0.
  // f:   process model x_{k+1} = f(x_k)
  // h:   observation model z = h(x)
  // j_f: Jacobian ∂f/∂x (linearized process model)
  // j_h: Jacobian ∂h/∂x (linearized observation model)
  // u_q: returns Q matrix (process noise, recomputed each step for dt-dependence)
  // u_r: returns R matrix (measurement noise, depends on measurement z for range-scaling)
  // P0:  initial state covariance (diagonal: uncertainty in each state at startup)
  explicit ExtendedKalmanFilter(
    const VecVecFunc & f, const VecVecFunc & h, const VecMatFunc & j_f, const VecMatFunc & j_h,
    const VoidMatFunc & u_q, const VecMatFunc & u_r, const Eigen::MatrixXd & P0);

  // Set the initial state
  void setState(const Eigen::VectorXd & x0);

  // Reset P_post to initial covariance (call after hard state resets)
  void resetCovariance();

  // Inflate covariance of one state element (and its cross-correlations) by factor.
  // Use after yaw snaps to reflect that the new value bypassed the EKF update.
  void inflateCovariance(int idx, double factor);

  // Decouple a state element from the rest of the filter by zeroing all
  // cross-covariance terms and setting the diagonal to a specified variance.
  // Use when a state element is managed externally (e.g., radius via ScalarKF).
  void decoupleState(int idx, double variance);

  // Copy x_post/P_post into x_pri/P_pri so the next update() sees post-jump state.
  void syncPrior();

  // Compute a predicted state
  Eigen::MatrixXd predict();

  // Update the estimated state based on measurement
  Eigen::MatrixXd update(const Eigen::VectorXd & z);

  // Update with an externally-provided observation model.
  // Use for secondary-face fusion where h() differs from the primary.
  // Operates on x_post/P_post (i.e., after the primary update()).
  Eigen::MatrixXd updateWithModel(
    const Eigen::VectorXd & z,
    const Eigen::VectorXd & z_pred,
    const Eigen::MatrixXd & H_ext,
    const Eigen::MatrixXd & R_ext);

  // Compute squared Mahalanobis distance of measurement z from predicted state.
  // Returns (z - h(x_pri))^T S^{-1} (z - h(x_pri)) where S = H P_pri H^T + R.
  double mahalanobis(const Eigen::VectorXd & z);

  // Mahalanobis distance with H linearized at a custom point.
  // For armor jumps where x_pri's yaw is ~90° off, linearizing at
  // the measured yaw produces a more accurate innovation covariance.
  double mahalanobisAt(const Eigen::VectorXd & z, const Eigen::VectorXd & x_linearize);

  // Get posterior variance of a single state element
  double getVariance(int idx) const { return P_post(idx, idx); }

  // Read-only access to the posterior covariance (for external gating)
  const Eigen::MatrixXd & P_post_view() const { return P_post; }

  // Per-state covariance upper bounds (set externally; empty = no clamping)
  Eigen::VectorXd max_covariance;

  // Spectral health check: if condition number of P exceeds this, blend with P0.
  // 0 = disabled.  Typical value: 1e6.
  double max_condition_number = 0.0;

private:
  // --- Pluggable nonlinear functions (set once at construction) ---
  VecVecFunc f;           // Process model:     x_{k+1} = f(x_k)
  VecVecFunc h;           // Observation model:  z = h(x)
  VecMatFunc jacobian_f;  // ∂f/∂x — linearizes f() at the current state each step
  Eigen::MatrixXd F;      // Cached Jacobian matrix from the most recent predict()
  VecMatFunc jacobian_h;  // ∂h/∂x — linearizes h() for the Kalman gain computation
  Eigen::MatrixXd H;      // Cached Jacobian matrix from the most recent update()

  // --- Noise covariance generators (called each step for dynamic tuning) ---
  VoidMatFunc update_Q;   // Returns Q (9×9 process noise); depends on dt_ via closure
  Eigen::MatrixXd Q;      // Cached Q from the most recent predict()
  VecMatFunc update_R;    // Returns R (4×4 measurement noise); depends on measurement z
                          // (range-dependent + angle-aware PnP degradation)
  Eigen::MatrixXd R;      // Cached R from the most recent update()

  // --- Covariance matrices ---
  // P_pri:  predicted (prior) covariance — how uncertain we are BEFORE seeing z
  // P_post: corrected (posterior) covariance — how uncertain we are AFTER fusing z
  // P0_:    initial covariance used by resetCovariance() after hard state resets
  Eigen::MatrixXd P_pri;
  Eigen::MatrixXd P_post;
  Eigen::MatrixXd P0_;

  // K: Kalman gain (9×4) — blends prediction and measurement.
  //    K ≈ 0 → trust prediction (low P, high R).  K ≈ 1 → trust measurement.
  Eigen::MatrixXd K;

  int n;                  // State dimension (9 in this project)
  Eigen::MatrixXd I;      // n×n identity matrix (precomputed to avoid re-allocation)

  // x_pri:  predicted state (after predict(), before update())
  // x_post: corrected state (after update(); also used as "current best" between frames)
  Eigen::VectorXd x_pri;
  Eigen::VectorXd x_post;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__KALMAN_FILTER_HPP_
