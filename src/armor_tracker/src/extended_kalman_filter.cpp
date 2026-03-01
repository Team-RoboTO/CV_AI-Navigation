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
  P_post(P0),
  P0_(P0),
  n(P0.rows()),
  I(Eigen::MatrixXd::Identity(n, n)),
  x_pri(n),
  x_post(n)
{
}

void ExtendedKalmanFilter::setState(const Eigen::VectorXd & x0) { x_post = x0; }

void ExtendedKalmanFilter::resetCovariance() { P_post = P0_; }

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

Eigen::MatrixXd ExtendedKalmanFilter::predict()
{
  F = jacobian_f(x_post), Q = update_Q();

  x_pri = f(x_post);
  P_pri = F * P_post * F.transpose() + Q;

  // handle the case when there will be no measurement before the next predict
  x_post = x_pri;
  P_post = P_pri;

  // Enforce covariance upper bounds to prevent P explosion during TEMP_LOST
  if (max_covariance.size() == n) {
    for (int i = 0; i < n; i++) {
      if (P_post(i, i) > max_covariance(i)) {
        double scale = std::sqrt(max_covariance(i) / std::max(P_post(i, i), 1e-10));
        P_post.row(i) *= scale;
        P_post.col(i) *= scale;
      }
    }
  }

  return x_pri;
}

Eigen::MatrixXd ExtendedKalmanFilter::update(const Eigen::VectorXd & z)
{
  H = jacobian_h(x_pri), R = update_R(z);

  Eigen::MatrixXd S = H * P_pri * H.transpose() + R;

  // LDLT decomposition: O(n^2) solve, built-in positive-definiteness check
  Eigen::LDLT<Eigen::MatrixXd> S_ldlt(S);
  if (S_ldlt.info() != Eigen::Success || !S_ldlt.isPositive()) {
    x_post = x_pri;
    P_post = P_pri;
    return x_post;
  }

  // Solve S * K^T = H * P_pri for K (avoids explicit inverse)
  K = S_ldlt.solve(H * P_pri.transpose()).transpose();

  x_post = x_pri + K * (z - h(x_pri));

  // Joseph form for numerical stability
  Eigen::MatrixXd IKH = I - K * H;
  P_post = IKH * P_pri * IKH.transpose() + K * R * K.transpose();

  // Enforce symmetry to prevent drift from floating-point accumulation
  P_post = (P_post + P_post.transpose()) * 0.5;

  return x_post;
}

}  // namespace rm_auto_aim
