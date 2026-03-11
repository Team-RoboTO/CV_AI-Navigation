#include "armor_tracker/extended_kalman_filter.hpp"

#include <cmath>

namespace rm_auto_aim
{

// ---------------------------------------------------------------------------
// HOW AN EKF WORKS — general math reference for this file
//
// The EKF maintains a Gaussian belief over the hidden state x:
//   mean  x_post  (best estimate after the last correction)
//   covariance P_post  (uncertainty — large diagonal = high uncertainty)
//
// --- PREDICT STEP (time update) ---
//
//   x_pri  = f(x_post, u)
//     f is the nonlinear motion model — integrates velocities, spins yaw, etc.
//     x_pri is the predicted mean BEFORE seeing the new measurement.
//
//   F  = df/dx |_{x_post}
//     F is the Jacobian (matrix of first-order partial derivatives) of f.
//     It linearises f locally so we can propagate the covariance.
//
//   Q  = process noise covariance
//     Models unmodelled dynamics / random disturbances (accel, wind, vibration).
//     Larger Q → trust the motion model less, converge faster to measurements.
//
//   P_pri  = F * P_post * F^T + Q
//     Propagates uncertainty through the linear approximation and adds noise.
//
// --- CORRECT STEP (measurement update) ---
//
//   y  = z - h(x_pri)
//     z is the raw measurement (e.g. observed armor position + yaw from PnP).
//     h is the nonlinear observation model (state → expected measurement).
//     y is the innovation — how much the prediction was wrong.
//
//   H  = dh/dx |_{x_pri}
//     Jacobian of h — linearises the observation model at the predicted state.
//
//   R  = measurement noise covariance
//     Models sensor accuracy.  Larger R → trust measurements less.
//
//   S  = H * P_pri * H^T + R
//     Innovation covariance — total uncertainty of the prediction in measurement
//     space.  Used to normalise the innovation and compute Mahalanobis distance.
//
//   K  = P_pri * H^T * S^{-1}
//     Kalman gain — balances how much to trust prediction vs measurement.
//     High P_pri (uncertain prediction) → large K → lean on measurement.
//     High R (noisy sensor) → small K → lean on prediction.
//
//   x_post  = x_pri + K * y
//     Corrected mean: nudge the prediction toward the measurement by K.
//
//   P_post  = (I - K * H) * P_pri
//     Corrected covariance: reduces uncertainty along the observed directions.
//
// The EKF approximation holds as long as f and h are nearly linear over the
// uncertainty region.  Highly nonlinear models (e.g., fast spinning yaw) may
// cause divergence — handled here by innovation gating and safety-net clamps.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Constructor — inject all nonlinear functions and set initial covariance.
//
// DESIGN PATTERN: Dependency injection via std::function.
//   The EKF class is generic — it doesn't know about armor plates, spinning
//   robots, or 9D states.  All domain-specific math (process model, observation
//   model, noise tuning) is injected as lambdas at construction time.  This
//   lets the same EKF implementation be reused for different state models
//   without inheritance or templates.
//
// WHY std::function INSTEAD OF TEMPLATES:
//   Templates would give better inlining/performance but would make the EKF
//   a header-only class and complicate the build.  std::function adds ~10ns
//   per call (virtual dispatch), which is negligible at 30Hz with 9D state.
//   The flexibility of runtime function binding also allows per-tracker lambda
//   captures (each tracker's f() captures its own damping_alpha pointer).
//
// INITIALIZATION:
//   P_pri = P_post = P0: Both prior and posterior start at the same initial
//   uncertainty.  P0_ is stored permanently for resetCovariance().
//   x_pri and x_post are allocated but uninitialised — setState() must be
//   called before the first predict() to set the actual state values.
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// setState — overwrite the state estimate (both prior and posterior).
//
// WHEN CALLED:
//   1. After initEKF() — set the initial state from the first PnP measurement.
//   2. After handleArmorJump() — yaw/za/r were snapped to new values outside
//      the EKF, so x_post/x_pri must be synchronised with target_state.
//   3. After safety-net clamps (r, v_yaw) — the clamped values must be written
//      back into the EKF or the next predict() would use the unclamped state.
//   4. After yaw normalisation — to keep x_post in sync with target_state(6).
//
// WHY BOTH x_post AND x_pri:
//   If only x_post were set, a subsequent call to mahalanobis() (which reads
//   x_pri) would use the OLD prior, producing wrong Mahalanobis distances.
//   If only x_pri were set, update() (which corrects x_pri → x_post) would
//   overwrite our intended state.  Setting both avoids these inconsistencies.
//
// IMPORTANT: This does NOT reset covariance — call resetCovariance() or
// decoupleState() separately if the state change invalidates P.
// ---------------------------------------------------------------------------
void ExtendedKalmanFilter::setState(const Eigen::VectorXd & x0)
{
  x_post = x0;
  x_pri = x0;
}

// ---------------------------------------------------------------------------
// resetCovariance — restore P to initial uncertainty (P0_).
//
// WHEN CALLED:
//   - After a hard state reset in handleArmorJump() (EKF diverged, position
//     was back-calculated from a single measurement → high uncertainty).
//   - After v_yaw is clamped to max (filter is clearly wrong → P is too
//     optimistic given that we just forcibly changed the state).
//
// WHY NOT JUST INFLATE?
//   inflateCovariance() scales the EXISTING P.  After divergence, P may be
//   tiny (the filter was confident but WRONG), and inflating tiny × 4 is
//   still tiny.  resetCovariance() replaces P with the original high-
//   uncertainty P0_, giving the filter a fresh start with large Kalman
//   gains so it re-converges quickly from the new (possibly inaccurate)
//   state estimate.
//
// SETS BOTH P_post AND P_pri: same rationale as setState() — ensures
// consistency whether the next operation is predict(), update(), or
// mahalanobis().
// ---------------------------------------------------------------------------
void ExtendedKalmanFilter::resetCovariance()
{
  P_post = P0_;
  P_pri = P0_;
}

// ---------------------------------------------------------------------------
// syncPrior — copy posterior (x_post, P_post) into prior (x_pri, P_pri).
//
// WHEN CALLED: After handleArmorJump() modifies x_post/P_post directly
// (yaw snap, covariance inflation, decoupleState).  The EKF's update()
// reads from x_pri/P_pri, so if they still hold the OLD prediction (from
// predict() earlier this frame), the Kalman gain and innovation would be
// computed against the stale pre-jump state, producing wrong corrections.
//
// By calling syncPrior(), the subsequent ekf.update() treats the post-jump
// state as its starting point and computes a meaningful correction from there.
//
// WHY NOT JUST CALL predict() AGAIN?
//   predict() would advance the state ANOTHER timestep forward, double-
//   counting the time evolution.  We want to keep the current time position
//   but update the linearisation point — syncPrior() does exactly this.
// ---------------------------------------------------------------------------
void ExtendedKalmanFilter::syncPrior()
{
  x_pri = x_post;
  P_pri = P_post;
}

// ---------------------------------------------------------------------------
// inflateCovariance — increase uncertainty of one state element.
//
// WHEN CALLED: After handleArmorJump() snaps yaw, za, or v_yaw to new
// values.  These direct state writes bypass the EKF's principled covariance
// propagation, so P no longer reflects true uncertainty.  Inflation
// corrects this by saying "we changed state[idx] outside the filter, so
// we're now more uncertain about it."
//
// MATH:
//   We want P(idx, idx) *= factor  (inflate the variance).
//   But we also need to update the off-diagonal terms P(idx, j) for j≠idx
//   to maintain the correlation structure.  The standard approach:
//
//     Correlation:  ρ(i,j) = P(i,j) / √(P(i,i) · P(j,j))
//
//   If we scale ONLY P(idx,idx), the correlations change (denominator grows
//   while numerator stays the same).  Instead, we scale the entire row AND
//   column by √factor:
//
//     P(idx, j) *= √factor    for all j   (row scaling)
//     P(j, idx) *= √factor    for all j   (column scaling)
//
//   This gives: P(idx, idx) *= factor  (diagonal inflated correctly)
//               P(idx, j)   *= √factor (off-diagonals scaled proportionally)
//               ρ(idx, j) stays the same (both numerator and denominator
//               scale by √factor).
//
// WHY SYMMETRY ENFORCEMENT: The row and column scaling are two separate
// operations.  Due to floating-point, P(i,j) and P(j,i) can drift apart
// by ~1e-15 each frame.  Over thousands of frames, this asymmetry compounds
// and can cause Eigen's LDLT decomposition to fail (it requires symmetry).
// Averaging P and Pᵀ forces exact symmetry.
// ---------------------------------------------------------------------------
void ExtendedKalmanFilter::inflateCovariance(int idx, double factor)
{
  double s = std::sqrt(factor);
  P_post.row(idx) *= s;
  P_post.col(idx) *= s;
  P_post = (P_post + P_post.transpose()) * 0.5;
}

// ---------------------------------------------------------------------------
// decoupleState — sever all statistical coupling between state[idx] and
// the rest of the EKF state, then set its variance from an external source.
//
// PROBLEM:
//   The orbit radius (state[8]) is managed by its own 1D scalar Kalman filter
//   (ScalarKF) rather than the 9D EKF.  But the EKF needs radius in its state
//   vector because the observation model h(x) computes:
//     xa = xc − r·cos(yaw),   ya = yc − r·sin(yaw)
//   So r must be present for correct Jacobian computation (j_h depends on r).
//
//   After each EKF predict() or update(), the Jacobian-based covariance
//   propagation (P = F·P·Fᵀ + Q or P = (I-KH)·P·(I-KH)ᵀ + K·R·Kᵀ)
//   regenerates cross-covariance terms P(8,j) and P(j,8) because H has
//   nonzero entries in the radius column (∂xa/∂r = -cos(yaw)).
//   These cross-terms would cause the EKF's Kalman gain to adjust radius
//   based on position/yaw innovations — conflicting with the scalar KF,
//   which does its own independent update.
//
// SOLUTION:
//   After every EKF operation, zero the entire row and column for the
//   externally-managed state, and write the scalar KF's variance on the
//   diagonal.  This makes the 9D EKF treat radius as a known parameter
//   (with its own uncertainty) that doesn't interact with other states.
//
// EFFECT ON THE COVARIANCE MATRIX (9×9):
//   Before:  P = [ P_pos/vel    P_cross ]    ← cross-terms exist
//                [ P_crossᵀ    P_r     ]
//   After:   P = [ P_pos/vel    0       ]    ← radius isolated
//                [ 0           r_kf.P  ]
//
// CONSEQUENCE:
//   The Kalman gain K = P·Hᵀ·S⁻¹ has K(8,:) ≈ 0 for all measurement
//   components, so the EKF update never changes radius.  This is correct
//   because the scalar KF already handled the radius update separately.
//   The price: radius cannot benefit from position/yaw innovations.
//   In practice, this lost coupling is negligible because Q_r ≈ 1e-6
//   (radius barely drifts), so cross-covariance would be tiny anyway.
//
// WHY BOTH P_pri AND P_post:
//   mahalanobis() reads P_pri; update() reads P_pri for Kalman gain.
//   If we only zeroed P_post, the next update() call would see stale
//   cross-terms in P_pri and compute wrong gains.  Zeroing both ensures
//   consistency no matter which function is called next.
//
// CALL FREQUENCY: Called 8 times per frame across different code paths
// (predict, update, secondary fusion, jump, clamps).  Each one is
// necessary because any EKF operation regenerates cross-terms.
// ---------------------------------------------------------------------------
void ExtendedKalmanFilter::decoupleState(int idx, double variance)
{
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
// mahalanobisAt — Mahalanobis distance with H linearized at a custom state.
//
// Same as mahalanobis() but evaluates the Jacobian H = ∂h/∂x at
// x_linearize instead of x_pri.  Innovation y = z − h(x_pri) still
// uses x_pri (nonlinear prediction, correct).
//
// USE CASE: Armor jumps.  During a jump, x_pri's yaw is ~90° away from
// the measured yaw.  H contains terms like r·sin(yaw) and -r·cos(yaw),
// which are maximally wrong at 90° offset.  Linearizing at the measured
// yaw (via x_linearize) gives a realistic S, producing a more accurate
// Mahalanobis distance for jump gating decisions.
// ---------------------------------------------------------------------------
double ExtendedKalmanFilter::mahalanobisAt(
  const Eigen::VectorXd & z, const Eigen::VectorXd & x_linearize)
{
  if (!z.allFinite()) return 1e9;
  Eigen::MatrixXd H_tmp = jacobian_h(x_linearize);
  Eigen::MatrixXd R_tmp = update_R(z);
  Eigen::MatrixXd S = H_tmp * P_pri * H_tmp.transpose() + R_tmp;
  Eigen::VectorXd y = z - h(x_pri);
  Eigen::LDLT<Eigen::MatrixXd> S_ldlt(S);
  if (S_ldlt.info() != Eigen::Success || !S_ldlt.isPositive()) {
    return 1e9;
  }
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

// ---------------------------------------------------------------------------
// updateWithModel — sequential EKF update with an externally-provided
// observation model (H, R, z_pred).
//
// PROBLEM:
//   The primary h(x) models the ACTIVE face:  xa = xc − r·cos(yaw).
//   When a second armor face is visible simultaneously (~90° away), its
//   observation model differs:
//     xa₂ = xc − r₂·cos(yaw + π/2)      ← different radius AND yaw offset
//     za₂ = za + dz                       ← different height
//   The primary update() uses the built-in h, j_h, u_r lambdas, which
//   are configured for the active face.  We can't change them mid-frame.
//
// SOLUTION:
//   Provide an external H (Jacobian), R (noise), z (measurement), and
//   z_pred (expected measurement) computed in tracker.cpp using the
//   secondary face's geometry.  This function applies a standard Kalman
//   update using those external matrices, treating the primary update's
//   posterior (x_post, P_post) as the prior.
//
// WHY SEQUENTIAL (not stacked measurement):
//   Fusing both measurements in a single update would require building a
//   combined H (8×9) and R (8×8), which complicates the code and makes
//   R non-diagonal (harder to tune).  Sequential updates are mathematically
//   equivalent to a joint update when the two measurements are independent
//   (which they are — separate PnP solves from separate bounding boxes).
//   The only cost is a ~10% larger P_post due to linearisation error in
//   the second update (negligible at 90° face separation).
//
// Operates on x_post / P_post (the posterior from the primary update),
// treating them as the prior for this secondary update.
// ---------------------------------------------------------------------------
Eigen::MatrixXd ExtendedKalmanFilter::updateWithModel(
  const Eigen::VectorXd & z,
  const Eigen::VectorXd & z_pred,
  const Eigen::MatrixXd & H_ext,
  const Eigen::MatrixXd & R_ext)
{
  // Innovation covariance: S = H · P_post · Hᵀ + R
  Eigen::MatrixXd S = H_ext * P_post * H_ext.transpose() + R_ext;

  // LDLT decomposition for stable inversion
  Eigen::LDLT<Eigen::MatrixXd> S_ldlt(S);
  if (S_ldlt.info() != Eigen::Success || !S_ldlt.isPositive()) {
    // Degenerate S — skip this secondary update
    return x_post;
  }

  // Kalman gain: K = P_post · Hᵀ · S⁻¹
  Eigen::MatrixXd K_ext = S_ldlt.solve(H_ext * P_post.transpose()).transpose();

  // State correction
  x_post = x_post + K_ext * (z - z_pred);

  // Joseph form covariance update
  Eigen::MatrixXd IKH = I - K_ext * H_ext;
  P_post = IKH * P_post * IKH.transpose() + K_ext * R_ext * K_ext.transpose();

  // Force symmetry
  P_post = (P_post + P_post.transpose()) * 0.5;

  return x_post;
}

}  // namespace rm_auto_aim
