#include "armor_tracker/extended_kalman_filter.hpp"

#include <cmath>

namespace rm_auto_aim
{
ExtendedKalmanFilter::ExtendedKalmanFilter(
  const VecVecFunc & f, const VecVecFunc & h, const VecMatFunc & j_f, const VecMatFunc & j_h,
  const VoidMatFunc & u_q, const VecMatFunc & u_r, const Eigen::MatrixXd & P0)
: f(f),
  h(h),
  jacobian_f(j_f),
  jacobian_h(j_h),
  update_Q(u_q),
  update_R(u_r),
  P_pri(P0),
  P_post(P0),
  P0_(P0),
  n(P0.rows()),
  I(Eigen::MatrixXd::Identity(n, n)),
  x_pri(n),
  x_post(n)
{
}

void ExtendedKalmanFilter::setState(const Eigen::VectorXd & x0)
{
  x_post = x0;
  x_pri = x0;
}

void ExtendedKalmanFilter::resetCovariance()
{
  P_post = P0_;
  P_pri = P0_;
}

void ExtendedKalmanFilter::syncPrior()
{
  x_pri = x_post;
  P_pri = P_post;
}

void ExtendedKalmanFilter::inflateCovariance(int idx, double factor)
{
  // Scale the row and column by sqrt(factor) to inflate the diagonal by factor
  // while preserving the correlation structure of off-diagonal elements.
  double s = std::sqrt(factor);
  P_post.row(idx) *= s;
  P_post.col(idx) *= s;
  // Re-enforce symmetry after the two separate scalings
  P_post = (P_post + P_post.transpose()) * 0.5;
}

void ExtendedKalmanFilter::decoupleState(int idx, double variance)
{
  // Zero all cross-covariance terms: the externally-managed state has no
  // statistical coupling with other EKF states.  Set the diagonal to the
  // external filter's variance so Mahalanobis gating and Kalman gains
  // reflect the true uncertainty.
  // Apply to both P_pri and P_post so that mahalanobis() and update(),
  // which read P_pri, also see the decoupled covariance.
  P_post.row(idx).setZero();
  P_post.col(idx).setZero();
  P_post(idx, idx) = variance;
  P_pri.row(idx).setZero();
  P_pri.col(idx).setZero();
  P_pri(idx, idx) = variance;
}

// ---------------------------------------------------------------------------
// PREDICT step — propagate the state and covariance one timestep forward.
//
// Mathematical operations:
//   1. Linearize: F = ∂f/∂x|_{x_post}       (Jacobian at current estimate)
//   2. Propagate state:      x_pri = f(x_post)
//   3. Propagate covariance: P_pri = F · P_post · Fᵀ + Q
//      The F·P·Fᵀ term rotates/scales the uncertainty ellipsoid through the
//      nonlinear dynamics; Q adds process noise (model uncertainty per step).
//
// After predict(), x_post/P_post are set to x_pri/P_pri so that if no
// measurement arrives before the next predict(), the filter gracefully coasts
// (predict-only mode during TEMP_LOST).
// ---------------------------------------------------------------------------
Eigen::MatrixXd ExtendedKalmanFilter::predict()
{
  // Evaluate Jacobian F and process noise Q at the current posterior state.
  // Both depend on dt_ (captured via closure), which changes each frame.
  F = jacobian_f(x_post), Q = update_Q();

  // State prediction: apply the nonlinear process model (damped constant-velocity)
  x_pri = f(x_post);
  // Covariance prediction: F·P·Fᵀ propagates old uncertainty through dynamics,
  // Q adds fresh uncertainty from unmodeled accelerations / model mismatch.
  P_pri = F * P_post * F.transpose() + Q;

  // Safety: clamp diagonal covariance elements to configurable upper bounds.
  // During TEMP_LOST, P grows every frame with no measurement correction.
  // Without clamping, P can explode → huge Kalman gain when detection resumes
  // → the filter snaps to a noisy measurement and overshoots.
  // The clamp scales the entire row+column (not just diagonal) to preserve
  // the correlation structure while limiting the absolute uncertainty.
  // Applied to P_pri so that mahalanobis() and update() (which read P_pri)
  // also see the clamped covariance.
  if (max_covariance.size() == n) {
    for (int i = 0; i < n; i++) {
      double cov_max = std::max(max_covariance(i), 1e-6);  // guard against zero/negative
      if (P_pri(i, i) > cov_max) {
        // Scale factor: sqrt(max / current) applied to row+col gives diagonal = max
        double scale = std::sqrt(cov_max / std::max(P_pri(i, i), 1e-10));
        P_pri.row(i) *= scale;
        P_pri.col(i) *= scale;
      }
    }
  }

  // Spectral health check: detect ill-conditioned P from large off-diagonal terms
  // that per-element clamping cannot catch.  When triggered, blend P towards P0
  // to restore numerical stability without a full hard reset.
  if (max_condition_number > 0.0) {
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(P_pri, Eigen::EigenvaluesOnly);
    if (es.info() == Eigen::Success) {
      double lmin = es.eigenvalues().minCoeff();
      double lmax = es.eigenvalues().maxCoeff();
      if (lmin < 1e-10 || lmax / std::max(lmin, 1e-15) > max_condition_number) {
        P_pri = 0.7 * P_pri + 0.3 * P0_;
        P_pri = (P_pri + P_pri.transpose()) * 0.5;
      }
    }
  }

  // Copy prior → posterior: if update() is never called (no detection this frame),
  // the next predict() will use these as the "best current estimate".
  x_post = x_pri;
  P_post = P_pri;

  return x_pri;
}

// ---------------------------------------------------------------------------
// Mahalanobis distance — statistical gating metric for outlier rejection.
//
// The Mahalanobis distance measures "how many standard deviations away" a
// measurement z is from the predicted measurement h(x_pri), accounting for
// the shape of the uncertainty ellipsoid (innovation covariance S).
//
//   d² = yᵀ · S⁻¹ · y     where y = z − h(x_pri),  S = H·P_pri·Hᵀ + R
//
// For a 4D measurement, d² follows a χ²(4) distribution.  Thresholds:
//   χ²(4, 0.95) ≈  9.49   (95% confidence — mild gate)
//   χ²(4, 0.99) ≈ 13.28   (99% confidence — used for normal match)
//   χ²(4, 0.999)≈ 18.47   (99.9% — used with relaxation for jumps)
//
// A measurement beyond the gate is likely from a different target or a
// sensor glitch — rejecting it prevents the EKF from being corrupted.
// ---------------------------------------------------------------------------
double ExtendedKalmanFilter::mahalanobis(const Eigen::VectorXd & z)
{
  if (!z.allFinite()) return 1e9;  // NaN/Inf measurement → treat as extreme outlier
  Eigen::MatrixXd H_tmp = jacobian_h(x_pri);
  Eigen::MatrixXd R_tmp = update_R(z);
  // S = innovation covariance: how uncertain is our predicted measurement?
  // Combines state uncertainty (H·P·Hᵀ) with sensor noise (R).
  Eigen::MatrixXd S = H_tmp * P_pri * H_tmp.transpose() + R_tmp;
  // Innovation vector: difference between what we measured and what we expected
  Eigen::VectorXd y = z - h(x_pri);
  // LDLT decomposition for stable inversion (avoids explicit S⁻¹)
  Eigen::LDLT<Eigen::MatrixXd> S_ldlt(S);
  if (S_ldlt.info() != Eigen::Success || !S_ldlt.isPositive()) {
    return 1e9;  // degenerate S → treat as extreme outlier
  }
  // d² = yᵀ · S⁻¹ · y — squared Mahalanobis distance
  return y.transpose() * S_ldlt.solve(y);
}

// ---------------------------------------------------------------------------
// UPDATE step — fuse a measurement z into the predicted state.
//
// Mathematical operations:
//   1. Innovation:        y = z − h(x_pri)          (what we measured vs predicted)
//   2. Innovation cov:    S = H·P_pri·Hᵀ + R        (total uncertainty of innovation)
//   3. Kalman gain:       K = P_pri·Hᵀ·S⁻¹          (optimal blend of pred. & meas.)
//   4. State correction:  x_post = x_pri + K·y       (shift state towards measurement)
//   5. Covariance update: P_post via Joseph form      (reduce uncertainty)
//
// The Kalman gain K determines how much the measurement "pulls" the state:
//   - If P_pri is large (uncertain prediction) and R is small (good sensor),
//     K → H⁻¹ and the filter trusts the measurement almost entirely.
//   - If P_pri is small (confident prediction) and R is large (noisy sensor),
//     K → 0 and the filter mostly ignores the measurement.
//
// Joseph form (step 5) is algebraically equivalent to the simpler
//   P_post = (I − K·H)·P_pri
// but is numerically more stable — it guarantees P_post stays positive
// semi-definite even with floating-point rounding errors.
// ---------------------------------------------------------------------------
Eigen::MatrixXd ExtendedKalmanFilter::update(const Eigen::VectorXd & z)
{
  // Linearize observation model and compute measurement noise at this z
  H = jacobian_h(x_pri), R = update_R(z);

  // Innovation covariance: S combines prediction uncertainty and sensor noise
  Eigen::MatrixXd S = H * P_pri * H.transpose() + R;

  // LDLT decomposition: numerically stable factorisation with O(n²) solve.
  // Also provides a positive-definiteness check — if S is degenerate
  // (e.g. from a nearly singular P_pri), we skip the update entirely.
  Eigen::LDLT<Eigen::MatrixXd> S_ldlt(S);
  if (S_ldlt.info() != Eigen::Success || !S_ldlt.isPositive()) {
    // Degenerate innovation covariance — cannot compute a meaningful gain.
    // Fall back to prediction-only (keep x_post = x_pri).
    x_post = x_pri;
    P_post = P_pri;
    return x_post;
  }

  // Kalman gain via LDLT solve: K = P_pri · Hᵀ · S⁻¹
  // We solve S · Kᵀ = (H · P_priᵀ) instead of inverting S explicitly,
  // which is both faster and more numerically stable.
  K = S_ldlt.solve(H * P_pri.transpose()).transpose();

  // State correction: shift the predicted state towards the measurement
  // by an amount proportional to K (the Kalman gain) × innovation (z − h(x_pri)).
  x_post = x_pri + K * (z - h(x_pri));

  // Joseph form covariance update: P_post = (I−KH)·P_pri·(I−KH)ᵀ + K·R·Kᵀ
  // This is equivalent to P_post = (I−KH)·P_pri but guarantees symmetry
  // and positive-definiteness even with floating-point rounding errors.
  Eigen::MatrixXd IKH = I - K * H;
  P_post = IKH * P_pri * IKH.transpose() + K * R * K.transpose();

  // Force exact symmetry: tiny asymmetries accumulate over thousands of frames
  // and can eventually cause Eigen decompositions to fail.
  P_post = (P_post + P_post.transpose()) * 0.5;

  return x_post;
}

}  // namespace rm_auto_aim
