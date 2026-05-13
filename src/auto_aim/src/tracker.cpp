

#include "auto_aim/tracker.hpp"
#include <angles/angles.h>
#include <algorithm>
#include <cmath>

namespace auto_aim
{

Tracker::Tracker(const TrackerConfig & cfg) : cfg_(cfg)
{
  x_ = Eigen::VectorXd::Zero(9);
  Eigen::VectorXd p0(9);
  //       xc   vxc  yc   vyc  za   vza  yaw  vyaw  r
  p0 << 0.1, 1.0, 0.1, 1.0, 0.1, 0.2, 0.1, 3.0, 0.01;
  P0_ = p0.asDiagonal();
  P_ = P0_;
}

// EKF predict with a damped constant-velocity model.
// each state pair evolves as:
//   pos_new = pos + vel * damping * dt
//   vel_new = vel * damping
//
// damping is time-normalized, so changing the frame rate does not change the
// per-second decay too much.
void Tracker::ekfPredict(double dt)
{
  double b, a;
  if (state_ == TEMP_LOST) {
    // no measurement is coming, so decay velocity faster.
    b = a = std::pow(cfg_.alpha_coast, dt * cfg_.ref_freq);
  } else {
    b = std::pow(cfg_.alpha_pos, dt * cfg_.ref_freq);
    a = std::pow(cfg_.alpha_yaw, dt * cfg_.ref_freq);
  }

  // state transition f(x).
  Eigen::VectorXd xn = x_;
  xn(0) += x_(1)*b*dt;  xn(1) = x_(1)*b;  // xc, vxc
  xn(2) += x_(3)*b*dt;  xn(3) = x_(3)*b;  // yc, vyc
  xn(4) += x_(5)*b*dt;  xn(5) = x_(5)*b;  // za, vza
  xn(6) += x_(7)*a*dt;  xn(7) = x_(7)*a;  // yaw, vyaw

  // jacobian F.
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(9, 9);
  F(0,1) = b*dt; F(1,1) = b;
  F(2,3) = b*dt; F(3,3) = b;
  F(4,5) = b*dt; F(5,5) = b;
  F(6,7) = a*dt; F(7,7) = a;

  // P8 (cleaned up): adaptive Q scheduling.
  //
  //   * Default leaves q_pos / q_yaw unchanged.
  //   * When enable_adaptive_q is true, three independent soft saturations
  //     vote for "stationary": position innovation norm (meters), velocity
  //     speed, and yaw rate. Earlier code mixed meters+radians into a single
  //     innovation norm, which made the threshold dimensionally wrong. Now
  //     we use last_innovation_pos_norm_ exclusively for the stationary vote.
  //   * The aggregate stationary confidence is EMA-smoothed (q_stationary_alpha)
  //     so brief bumps do not flap Q.
  //   * Yaw rate also boosts q_yaw to make the EKF responsive against spin.
  //   * Both effective values are clamped to [q_*_eff_min, q_*_eff_max] so
  //     the schedule cannot collapse Q to zero (which would freeze the EKF)
  //     or blow it up to numerical instability.
  double q_pos_eff = cfg_.q_pos;
  double q_yaw_eff = cfg_.q_yaw;
  if (cfg_.enable_adaptive_q) {
    const double speed = std::sqrt(x_(1)*x_(1) + x_(3)*x_(3) + x_(5)*x_(5));
    const double yawrate = std::abs(x_(7));
    const double inn_pos = last_innovation_pos_norm_;
    const double s_speed   = std::max(0.0, 1.0 - speed   / std::max(cfg_.stationary_speed_thresh,   1e-6));
    const double s_yawrate = std::max(0.0, 1.0 - yawrate / std::max(cfg_.stationary_yawrate_thresh, 1e-6));
    const double s_inn     = std::max(0.0, 1.0 - inn_pos / std::max(cfg_.stationary_innov_thresh,   1e-6));
    const double stationary_raw = std::min({s_speed, s_yawrate, s_inn});

    const double a = std::clamp(cfg_.q_stationary_alpha, 0.0, 1.0);
    q_stationary_ema_ = a * stationary_raw + (1.0 - a) * q_stationary_ema_;
    const double reduction = cfg_.q_reduction_max * q_stationary_ema_;
    q_pos_eff *= (1.0 - reduction);
    q_yaw_eff *= (1.0 - reduction);

    const double boost = std::min(cfg_.yawrate_q_boost_max,
                                  cfg_.yawrate_q_boost_per_rad * yawrate);
    q_yaw_eff *= (1.0 + boost);
  }
  q_pos_eff = std::clamp(q_pos_eff, cfg_.q_pos_eff_min, cfg_.q_pos_eff_max);
  q_yaw_eff = std::clamp(q_yaw_eff, cfg_.q_yaw_eff_min, cfg_.q_yaw_eff_max);
  last_q_pos_eff_ = q_pos_eff;
  last_q_yaw_eff_ = q_yaw_eff;

  // process noise Q, using a white-noise acceleration model.
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(9, 9);
  double t = dt;
  auto block = [&](int i, double s2) {
    Q(i,i) = std::pow(t,4)/4*s2;  Q(i,i+1) = std::pow(t,3)/2*s2;
    Q(i+1,i) = Q(i,i+1);          Q(i+1,i+1) = t*t*s2;
  };
  block(0, q_pos_eff); block(2, q_pos_eff); block(4, q_pos_eff);
  block(6, q_yaw_eff);
  Q(8,8) = std::pow(t,4)/4 * cfg_.q_r;

  x_ = xn;
  P_ = F * P_ * F.transpose() + Q;
  P_ = (P_ + P_.transpose()) * 0.5;

  // keep covariance bounded while coasting.
  Eigen::VectorXd mc(9);
  mc << 1.0, 10.0, 1.0, 10.0, 1.0, 2.0, 1.0, 30.0, 0.01;
  for (int i = 0; i < 9; i++) {
    if (P_(i,i) > mc(i)) {
      double s = std::sqrt(mc(i) / std::max(P_(i,i), 1e-10));
      P_.row(i) *= s;  P_.col(i) *= s;
    }
  }
}

// EKF update for z = [xa, ya, za, yaw].
// observation model h(x):
//   xa = xc - r * cos(yaw)     (armor is offset from center by radius)
//   ya = yc - r * sin(yaw)
//   za = za                    (pass-through)
//   yaw = yaw                  (pass-through)
//
// Measurement noise R has three components:
//   1. Range slope (legacy): position and yaw sigmas grow linearly with range.
//   2. Obliquity (legacy): 1/cos^2 on position, 1/cos^4 on yaw. yaw sigma is
//      blown up to 1e6 when the plate is more oblique than max_oblique_deg.
//   3. Adaptive R (P5, behind enable_adaptive_r): scales the position and yaw
//      sigmas by a quality factor derived from PnP reprojection error and
//      detector confidence. The factor is EMA-smoothed across frames.
Eigen::VectorXd Tracker::ekfUpdate(const Eigen::Vector4d & z, const ArmorDetection & meta)
{
  double yaw = x_(6), r = x_(8);

  // H is the jacobian of h(x).
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(4, 9);
  H(0,0) = 1;  H(0,6) =  r*sin(yaw);  H(0,8) = -cos(yaw);
  H(1,2) = 1;  H(1,6) = -r*cos(yaw);  H(1,8) = -sin(yaw);
  H(2,4) = 1;
  H(3,6) = 1;

  // R depends on range and obliquity.
  double dist = std::sqrt(z(0)*z(0) + z(1)*z(1) + z(2)*z(2));
  double ps = cfg_.r_pos_base + cfg_.r_pos_slope * dist;
  double ys = cfg_.r_yaw_base + cfg_.r_yaw_slope * dist;
  double xyz_f = 1.0, yaw_f = 1.0;
  if (z(0)*z(0) + z(1)*z(1) > 0.01) {
    double bearing = std::atan2(z(1), z(0));
    double face_a = std::abs(angles::shortest_angular_distance(z(3), bearing));
    double cf = std::cos(face_a);
    xyz_f = 1.0 / std::max(cf*cf, 0.04);
    yaw_f = 1.0 / std::max(std::pow(std::abs(cf), 4.0), 1e-4);
    if (face_a > cfg_.max_oblique_deg * M_PI / 180.0)
      yaw_f = 1e6;  // ignore yaw when the armor is edge-on.
  }

  // P5 adaptive-R quality scaler. Inflates *both* position and yaw sigmas
  // (so the same scaler enters R via square below) when the bbox-PnP looked
  // suspicious or the detector confidence was low. Scaler = 1 means "fully
  // trust", grows linearly between r_reproj_norm_soft and r_reproj_norm_hard,
  // and is also raised when confidence is below r_confidence_floor.
  double quality_scaler_raw = 1.0;
  if (cfg_.enable_adaptive_r) {
    double rep = meta.pnp_reproj_err_norm;
    double rep_lo = std::max(cfg_.r_reproj_norm_soft, 1e-6);
    double rep_hi = std::max(cfg_.r_reproj_norm_hard, rep_lo + 1e-6);
    double rep_t = std::clamp((rep - rep_lo) / (rep_hi - rep_lo), 0.0, 1.0);
    double rep_scaler = 1.0 + rep_t * (cfg_.r_reproj_scaler_max - 1.0);

    double conf_floor = std::max(cfg_.r_confidence_floor, 0.0);
    double conf_t = std::clamp((conf_floor - meta.confidence) / std::max(conf_floor, 1e-6),
                               0.0, 1.0);
    double conf_scaler = 1.0 + conf_t * (cfg_.r_reproj_scaler_max - 1.0);

    quality_scaler_raw = std::max(rep_scaler, conf_scaler);
  }
  double a_r = std::clamp(cfg_.r_quality_alpha, 0.0, 1.0);
  r_quality_ema_ = a_r * quality_scaler_raw + (1.0 - a_r) * r_quality_ema_;

  // Compose the effective sigmas. quality_scaler enters as a multiplicative
  // sigma factor (so R sees it squared). xyz_f and yaw_f keep their 1/cos^k
  // shape — adaptive-R is on top, not in place of, the obliquity model.
  double quality_scaler = cfg_.enable_adaptive_r ? r_quality_ema_ : 1.0;
  double ps_eff = ps * std::sqrt(xyz_f) * quality_scaler;
  double ys_eff = ys * std::sqrt(yaw_f) * quality_scaler;
  last_r_pos_eff_ = ps_eff;
  last_r_yaw_eff_ = ys_eff;

  Eigen::Matrix4d R = Eigen::Matrix4d::Zero();
  R(0,0) = R(1,1) = R(2,2) = ps_eff * ps_eff;
  R(3,3) = ys_eff * ys_eff;

  // innovation.
  Eigen::Vector4d z_pred;
  z_pred << x_(0) - r*cos(yaw), x_(2) - r*sin(yaw), x_(4), yaw;
  Eigen::Vector4d y = z - z_pred;
  last_innovation_norm_     = y.norm();
  last_innovation_pos_norm_ = std::sqrt(y(0)*y(0) + y(1)*y(1) + y(2)*y(2));
  last_innovation_yaw_abs_  = std::abs(y(3));

  // Kalman gain via LDLT.
  Eigen::Matrix4d S = H * P_ * H.transpose() + R;
  Eigen::LDLT<Eigen::Matrix4d> S_ldlt(S);
  if (S_ldlt.info() != Eigen::Success || !S_ldlt.isPositive()) return x_;

  Eigen::MatrixXd K = S_ldlt.solve(H * P_.transpose()).transpose();
  x_ = x_ + K * y;

  // joseph form keeps P better behaved numerically.
  Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(9,9) - K * H;
  P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
  P_ = (P_ + P_.transpose()) * 0.5;
  return x_;
}

double Tracker::ekfMahalanobis(const Eigen::Vector4d & z) const
{
  if (!z.allFinite()) return 1e9;
  double yaw = x_(6), r = x_(8);
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(4,9);
  H(0,0)=1; H(0,6)=r*sin(yaw); H(0,8)=-cos(yaw);
  H(1,2)=1; H(1,6)=-r*cos(yaw); H(1,8)=-sin(yaw);
  H(2,4)=1; H(3,6)=1;
  double d = std::sqrt(z(0)*z(0)+z(1)*z(1)+z(2)*z(2));
  double ps = cfg_.r_pos_base + cfg_.r_pos_slope*d;
  double ys = cfg_.r_yaw_base + cfg_.r_yaw_slope*d;
  double xyz_f = 1.0, yaw_f = 1.0;
  if (z(0)*z(0) + z(1)*z(1) > 0.01) {
    const double bearing = std::atan2(z(1), z(0));
    const double face_a = std::abs(angles::shortest_angular_distance(z(3), bearing));
    const double cf = std::cos(face_a);
    xyz_f = 1.0 / std::max(cf*cf, 0.04);
    yaw_f = 1.0 / std::max(std::pow(std::abs(cf), 4.0), 1e-4);
    if (face_a > cfg_.max_oblique_deg * M_PI / 180.0) {
      yaw_f = 1e6;
    }
  }
  Eigen::Matrix4d R = Eigen::Matrix4d::Zero();
  R(0,0)=R(1,1)=R(2,2)=ps*ps*xyz_f; R(3,3)=ys*ys*yaw_f;
  Eigen::Vector4d zp; zp << x_(0)-r*cos(yaw), x_(2)-r*sin(yaw), x_(4), yaw;
  Eigen::Vector4d y = z - zp;
  Eigen::Matrix4d S = H*P_*H.transpose()+R;
  Eigen::LDLT<Eigen::Matrix4d> Sl(S);
  if (Sl.info()!=Eigen::Success) return 1e9;
  return (y.transpose()*Sl.solve(y)).value();
}

Eigen::Vector3d Tracker::armorFromState(const Eigen::VectorXd & x) const
{
  return {x(0)-x(8)*cos(x(6)), x(2)-x(8)*sin(x(6)), x(4)};
}

double Tracker::previewUnwrapYaw(double raw_yaw, double reference_yaw) const
{
  return reference_yaw + angles::shortest_angular_distance(reference_yaw, raw_yaw);
}

void Tracker::commitYaw(double yaw)
{
  last_yaw_ = yaw;
}

double Tracker::targetRange() const
{
  return std::sqrt(x_(0)*x_(0) + x_(2)*x_(2) + x_(4)*x_(4));
}

void Tracker::initFromDetection(const ArmorDetection & det)
{
  double yaw = previewUnwrapYaw(det.yaw, 0.0);
  commitYaw(yaw);
  double r = cfg_.initial_radius;
  radius_ = r;  other_radius_ = r;
  dz_ = 0;  dz_initialized_ = false;
  x_ = Eigen::VectorXd::Zero(9);
  x_ << det.x+r*cos(yaw), 0, det.y+r*sin(yaw), 0, det.z, 0, yaw, 0, r;
  P_ = P0_;
  target_id_ = det.class_id;
  detect_count_ = 0;  lost_count_ = 0;
  switch_cooldown_counter_ = cfg_.switch_cooldown;
  // Reset track-quality counters and adaptive Q/R memories for the new lock.
  match_count_ = 0;
  miss_count_  = 0;
  last_match_time_s_ = -1.0;
  last_meas_quality_ = MQ_NONE;
  q_stationary_ema_ = 0.0;
  r_quality_ema_ = 1.0;
}

// target switching is intentionally conservative.
// stay locked while tracking, and only switch from TEMP_LOST if a much closer
// target appears after the cooldown.
bool Tracker::shouldSwitch(const ArmorDetection & candidate) const
{
  if (state_ == LOST) return true;
  if (state_ == DETECTING) return false;
  if (switch_cooldown_counter_ > 0) return false;
  double cur_range = targetRange();
  double new_range = candidate.range();
  if (state_ == TRACKING) {
    return cfg_.enable_tracking_switch &&
      new_range < cur_range * cfg_.tracking_switch_range_ratio;
  }
  // during TEMP_LOST, only switch to something clearly closer.
  return new_range < cur_range * cfg_.switch_range_ratio;
}

// armor jump: the robot rotated and a different face became visible.
// snap yaw first, then let the EKF update from a reasonable linearization point.
void Tracker::handleArmorJump(const ArmorDetection & det)
{
  double yaw = previewUnwrapYaw(det.yaw, x_(6));
  commitYaw(yaw);
  double jump = angles::shortest_angular_distance(x_(6), yaw);

  // spin reversal: jump direction opposes estimated spin.
  if (std::abs(x_(7)) > 0.2 && jump * x_(7) < 0) x_(7) = 0;
  x_(6) = yaw;

  // around 90 deg means the other face pair is visible.
  double ja = std::abs(jump);
  if (ja > M_PI/4 && ja < 3*M_PI/4) {
    if (dz_initialized_) dz_ = -dz_;
    double new_dz = std::clamp(x_(4) - det.z, -0.25, 0.25);
    dz_ = dz_initialized_ ? 0.05*new_dz + 0.95*dz_ : new_dz;
    dz_initialized_ = true;
    x_(4) = det.z;  x_(5) = 0;
    std::swap(radius_, other_radius_);
    x_(8) = radius_;
  }

  // hard reset if the inferred armor is too far from the detection.
  auto inferred = armorFromState(x_);
  Eigen::Vector3d detected(det.x, det.y, det.z);
  if ((inferred - detected).norm() > cfg_.max_match_dist) {
    double r = x_(8);
    x_ << det.x+r*cos(yaw), 0, det.y+r*sin(yaw), 0, det.z, 0, yaw, 0, r;
    P_ = P0_;
    radius_ = cfg_.initial_radius;  other_radius_ = cfg_.initial_radius;
    dz_ = 0;  dz_initialized_ = false;
  } else {
    // inflate yaw covariance because we bypassed the normal EKF path.
    double s = 2.0;
    P_.row(6)*=s; P_.col(6)*=s;  P_.row(7)*=s; P_.col(7)*=s;
    P_ = (P_+P_.transpose())*0.5;
  }

  // now run the normal EKF update with the jump measurement.
  Eigen::Vector4d z(det.x, det.y, det.z, yaw);
  x_ = ekfUpdate(z, det);

  // update radius with a small EMA step.
  double bearing = std::atan2(det.y, det.x);
  double fa = std::abs(angles::shortest_angular_distance(x_(6), bearing));
  if (fa < cfg_.max_oblique_deg * M_PI/180.0) {
    double rm = (x_(0)-det.x)*cos(x_(6)) + (x_(2)-det.y)*sin(x_(6));
    if (rm > 0.10 && rm < 0.45)
      radius_ = (1-cfg_.radius_ema_alpha)*radius_ + cfg_.radius_ema_alpha*rm;
  }
  x_(8) = radius_;
}

// main per-frame tracker update.
void Tracker::update(const std::vector<ArmorDetection> & detections, double dt,
                     double now_s)
{
  // Default to MQ_NONE; ekfUpdate paths overwrite this when they run.
  last_meas_quality_ = MQ_NONE;
  last_best_match_mahalanobis_ = 0.0;
  last_best_match_position_diff_ = 0.0;
  last_best_match_yaw_diff_ = 0.0;
  last_match_reject_reason_.clear();
  last_tracker_miss_reason_.clear();
  last_assigned_count_ = 0;
  last_association_reject_count_ = 0;

  if (switch_cooldown_counter_ > 0) switch_cooldown_counter_--;

  // check target switching before doing the normal association.
  if (!detections.empty() && state_ != LOST) {
    const ArmorDetection * closest = nullptr;
    double min_range = 1e9;
    for (const auto & d : detections) {
      if (d.range() < min_range) { min_range = d.range(); closest = &d; }
    }
    if (closest && shouldSwitch(*closest)) {
      initFromDetection(*closest);
      if (cfg_.confirm_frames <= 1) {
        state_ = TRACKING;
        switch_cooldown_counter_ = cfg_.switch_cooldown;
      } else {
        state_ = DETECTING;
      }
      last_assigned_count_ = 1;
      last_match_reject_reason_ = "accepted_switch";
      last_meas_quality_ = MQ_ACCEPTED;
      match_count_ = 1;
      miss_count_ = 0;
      if (now_s > 0.0) last_match_time_s_ = now_s;
      return;
    }
  }

  // lost state: start from the closest detection.
  if (state_ == LOST) {
    if (detections.empty()) {
      last_tracker_miss_reason_ = "no_detections";
      return;
    }
    auto best = std::min_element(detections.begin(), detections.end(),
      [](const auto& a, const auto& b) { return a.range() < b.range(); });
    initFromDetection(*best);
    if (cfg_.confirm_frames <= 1) {
      state_ = TRACKING;
      switch_cooldown_counter_ = cfg_.switch_cooldown;
    } else {
      state_ = DETECTING;
    }
    last_assigned_count_ = 1;
    last_meas_quality_ = MQ_ACCEPTED;
    match_count_ = 1;
    miss_count_ = 0;
    if (now_s > 0.0) last_match_time_s_ = now_s;
    return;
  }

  lost_thresh_ = std::max(1, (int)(cfg_.lost_timeout / std::max(dt, 0.01)));

  // predict.
  ekfPredict(dt);

  // associate with Mahalanobis gating.
  auto pred_armor = armorFromState(x_);
  bool matched = false;
  bool face_jump_forced = false;
  ArmorDetection best_det{};
  double best_maha = cfg_.maha_threshold;
  double best_seen_maha = 1e9;
  bool saw_same_class = false;
  bool has_face_jump_candidate = false;
  ArmorDetection face_jump_det{};
  double best_face_jump_pd = 1e9;
  double best_face_jump_yd = 0.0;

  for (const auto & det : detections) {
    if (det.class_id != target_id_) continue;
    saw_same_class = true;
    // Preview unwrap has no side effect. Rejected candidates must not mutate
    // last_yaw_, otherwise a bad detection corrupts yaw continuity for the next
    // frame. We commit yaw only after accepting a measurement.
    double uw = previewUnwrapYaw(det.yaw, x_(6));
    Eigen::Vector4d z(det.x, det.y, det.z, uw);
    double m = ekfMahalanobis(z);
    Eigen::Vector3d dp(det.x, det.y, det.z);
    double pd = (pred_armor - dp).norm();
    double yd = std::abs(angles::shortest_angular_distance(x_(6), uw));
    if (m < best_seen_maha) {
      best_seen_maha = m;
      last_best_match_mahalanobis_ = m;
    }
    if (m < best_maha) { best_maha = m; best_det = det; matched = true; }
    const double face_jump_max_dist =
      cfg_.max_match_dist * std::max(1.0, cfg_.face_jump_max_match_dist_ratio);
    if (pd < face_jump_max_dist &&
        yd >= cfg_.face_jump_min_yaw &&
        yd <= cfg_.face_jump_max_yaw &&
        std::isfinite(det.x) && std::isfinite(det.y) &&
        std::isfinite(det.z) && std::isfinite(det.yaw)) {
      if (!has_face_jump_candidate || pd < best_face_jump_pd) {
        has_face_jump_candidate = true;
        face_jump_det = det;
        best_face_jump_pd = pd;
        best_face_jump_yd = yd;
      }
    }
  }
  if (!matched && has_face_jump_candidate) {
    matched = true;
    face_jump_forced = true;
    best_det = face_jump_det;
    last_best_match_position_diff_ = best_face_jump_pd;
    last_best_match_yaw_diff_ = best_face_jump_yd;
    last_match_reject_reason_ = "rejected_maha_but_face_jump_candidate";
    last_association_reject_count_ = 1;
  }
  last_mahalanobis_ = (matched && !face_jump_forced) ? best_maha :
    (std::isfinite(best_seen_maha) && best_seen_maha < 1e8 ? best_seen_maha : 0.0);
  if (!matched) {
    if (detections.empty()) {
      last_match_reject_reason_ = "no_detections";
    } else if (!saw_same_class) {
      last_match_reject_reason_ = "class_mismatch";
    } else {
      last_match_reject_reason_ = "mahalanobis_gate";
      last_association_reject_count_ = 1;
    }
  }

  // update or coast.
  if (matched) {
    double meas_yaw = previewUnwrapYaw(best_det.yaw, x_(6));
    double yd = std::abs(angles::shortest_angular_distance(x_(6), meas_yaw));
    Eigen::Vector3d dp(best_det.x, best_det.y, best_det.z);
    double pd = (pred_armor - dp).norm();
    last_best_match_position_diff_ = pd;
    last_best_match_yaw_diff_ = yd;

    // obliquity check.
    double bearing = std::atan2(best_det.y, best_det.x);
    double fa = std::abs(angles::shortest_angular_distance(meas_yaw, bearing));
    bool oblique = fa > cfg_.max_oblique_deg * M_PI/180.0;

    if (face_jump_forced) {
      handleArmorJump(best_det);
      last_meas_quality_ = MQ_ACCEPTED;
      last_assigned_count_ = 1;
      last_match_reject_reason_ = "accepted_face_jump_after_maha";
    } else if (pd < cfg_.max_match_dist && (yd < cfg_.yaw_jump_thresh || oblique)) {
      // normal EKF update.
      Eigen::Vector4d z(best_det.x, best_det.y, best_det.z, meas_yaw);
      x_ = ekfUpdate(z, best_det);
      commitYaw(meas_yaw);
      last_assigned_count_ = 1;
      last_match_reject_reason_ = "accepted";
      // Quality classification: oblique view means the yaw measurement was
      // effectively ignored (R(3,3) blown up upstream), so flag DEGRADED.
      // Adaptive R inflation does not by itself trigger DEGRADED — we rely on
      // the smooth scaler instead of a hard label.
      last_meas_quality_ = oblique ? MQ_DEGRADED : MQ_ACCEPTED;
      // radius EMA, skipped when the view is too oblique.
      if (!oblique) {
        double rm = (x_(0)-best_det.x)*cos(x_(6)) + (x_(2)-best_det.y)*sin(x_(6));
        if (rm > 0.10 && rm < 0.45)
          radius_ = (1-cfg_.radius_ema_alpha)*radius_ + cfg_.radius_ema_alpha*rm;
        x_(8) = radius_;
      }
    } else if (yd > cfg_.yaw_jump_thresh && !oblique) {
      // face switch.
      handleArmorJump(best_det);
      last_meas_quality_ = MQ_ACCEPTED;
      last_assigned_count_ = 1;
      last_match_reject_reason_ = "accepted_face_jump";
    } else {
      // measurement available but failed the gate (e.g. too far, or yaw jump
      // with oblique view that we don't trust as a face switch).
      last_meas_quality_ = MQ_REJECTED;
      last_association_reject_count_ = 1;
      if (pd >= cfg_.max_match_dist) {
        last_match_reject_reason_ = "position_gate";
      } else {
        last_match_reject_reason_ = "yaw_jump_gate";
      }
      matched = false;
    }
  }

  // safety clamps.
  x_(8) = std::clamp(x_(8), 0.12, 0.40);
  radius_ = x_(8);
  if (std::abs(x_(7)) > 25.0) { x_(7) = std::copysign(25.0, x_(7)); P_ = P0_; }
  double wy = angles::normalize_angle(x_(6));
  if (wy != x_(6)) { x_(6) = wy; }

  // state machine.
  if (state_ == DETECTING) {
    if (matched) {
      detect_count_++;
      lost_count_ = 0;  // reset miss counter on match.
      if (detect_count_ >= cfg_.confirm_frames) {
        state_ = TRACKING; detect_count_ = 0; lost_count_ = 0;
        switch_cooldown_counter_ = cfg_.switch_cooldown;
      }
    } else {
      // tolerate a few missed frames while confirming a new target.
      detect_count_ = 0;  // reset consecutive match counter.
      if (++lost_count_ > 3) {
        state_ = LOST;
        lost_count_ = 0;
      }
    }
  } else if (state_ == TRACKING) {
    if (!matched) { state_ = TEMP_LOST; lost_count_ = 1; }
  } else if (state_ == TEMP_LOST) {
    if (matched) { state_ = TRACKING; lost_count_ = 0; }
    else if (++lost_count_ > lost_thresh_) { state_ = LOST; }
  }

  if (!matched && last_tracker_miss_reason_.empty()) {
    last_tracker_miss_reason_ = last_match_reject_reason_.empty()
      ? "unmatched"
      : last_match_reject_reason_;
  }

  // Track-quality counters and last-match timestamp. now_s == 0.0 means the
  // caller did not pass a wall-clock; we then leave last_match_time_s_ alone
  // (so age stays zero). Otherwise we keep the most recent match anchor.
  if (matched) {
    match_count_++;
    miss_count_ = 0;
    if (now_s > 0.0) last_match_time_s_ = now_s;
  } else {
    miss_count_++;
    match_count_ = 0;
  }
}

// simple iterative pitch solve for gravity.
Tracker::BallisticResult Tracker::solveBallistic(double gd, double dz) const
{
  BallisticResult res{0, 0, false};
  if (gd < 0.01) return res;
  double v = cfg_.bullet_speed;
  double pitch = std::atan2(dz, gd);
  for (int i = 0; i < 5; i++) {
    double vx = v * std::cos(pitch);
    if (vx < 1.0) return res;
    double t = gd / vx;
    pitch = std::atan2(dz + 0.5*cfg_.gravity*t*t, gd);
  }
  res.flight_time = gd / std::max(v*std::cos(pitch), 1.0);
  res.pitch = pitch;
  res.valid = std::abs(pitch) < 1.2;
  return res;
}

// predict the face the bullet should hit and compute the gimbal command.
// the aim vector starts at the barrel, not the camera, so close-range parallax
// from camera/barrel offset is handled here.
AimResult Tracker::computeAim(double cur_yaw, double cur_pitch,
                              double override_pred_lead_s) const
{
  AimResult aim;
  if (state_ != TRACKING && state_ != TEMP_LOST) return aim;

  // barrel position in odom. the offset is fixed in the gimbal body frame.
  double cos_y = std::cos(cur_yaw), sin_y = std::sin(cur_yaw);
  double barrel_x = cfg_.barrel_offset_x * cos_y - cfg_.barrel_offset_y * sin_y;
  double barrel_y = cfg_.barrel_offset_x * sin_y + cfg_.barrel_offset_y * cos_y;
  double barrel_z = cfg_.gimbal_height + cfg_.barrel_offset_z;

  // pred_lead_extra is the non-flight component of the prediction lead. When
  // override_pred_lead_s is provided, it replaces cfg_.time_bias entirely;
  // otherwise we fall back to the legacy fixed time_bias. The override is
  // honoured across BOTH iterations of the face search below, fixing an
  // earlier bug where iter-1 silently reset to time_bias and dropped any
  // measured-latency contribution.
  const double pred_lead_extra = (override_pred_lead_s >= 0.0)
    ? override_pred_lead_s
    : cfg_.time_bias;
  double pred_t = pred_lead_extra + 1.5 / std::max(cfg_.bullet_speed, 1.0);
  constexpr int FACES = 4;

  struct Face {
    double x = 0, y = 0, z = 0, range = 0, bearing = 0;
    double abs_pitch = 0, flight_time = 0, margin = -1e9, vis = 0;
    bool valid = false;
    int idx = -1;
    double face_yaw = 0;       // future face yaw at impact, odom frame
    double residual = 0;       // P9 anti-gyro residual [s]
  };
  Face best;
  Face best_residual;
  bool found = false;
  bool found_residual = false;
  const bool anti_gyro_mode =
    cfg_.enable_anti_gyro && std::abs(x_(7)) > cfg_.anti_gyro_min_yawrate;

  // two passes: rough time first, then refine with the actual flight time.
  for (int iter = 0; iter < 2; iter++) {
    for (int fi = 0; fi < FACES; fi++) {
      double fy = x_(6) + x_(7)*pred_t + fi*(2*M_PI/FACES);
      bool alt = (fi % 2 == 1);
      double r = alt ? other_radius_ : radius_;
      double dzo = alt ? dz_ : 0.0;

      // predicted center at impact.
      double cx = x_(0) + x_(1)*pred_t;
      double cy = x_(2) + x_(3)*pred_t;
      double cz = x_(4) + x_(5)*pred_t;

      // face position in odom.
      double fx = cx - r*std::cos(fy);
      double fpy = cy - r*std::sin(fy);
      double fz = cz + dzo;

      // aim vector from barrel to target face.
      double dx = fx - barrel_x;
      double dy = fpy - barrel_y;
      double dz_aim = fz - barrel_z;

      double gd = std::hypot(dx, dy);           // ground distance from barrel
      double range = std::sqrt(dx*dx + dy*dy + dz_aim*dz_aim);
      double bearing = std::atan2(dy, dx);       // yaw from barrel to target

      // visibility uses the camera side, since this is about what we can see.
      double fn = fy + M_PI;
      double cd = std::atan2(-fpy, -fx);  // camera is near origin
      double obl = std::abs(angles::shortest_angular_distance(fn, cd));
      double vis = std::clamp(std::cos(obl), 0.0, 1.0);

      // ballistic solution from barrel to target.
      auto bal = solveBallistic(gd, dz_aim);
      if (!bal.valid) continue;

      // positive margin means the face is aligned enough to fire.
      double ds = std::min(cfg_.window_ref_dist / std::max(range, 0.5), 2.0);
      double win = cfg_.angular_window * ds;
      double err = std::abs(angles::shortest_angular_distance(bearing, fy));
      double margin = win - err;

      Face c;
      c.x = fx; c.y = fpy; c.z = fz;
      c.range = range; c.bearing = bearing;
      c.abs_pitch = bal.pitch; c.flight_time = bal.flight_time;
      c.margin = margin; c.vis = vis; c.valid = true;
      c.idx = fi;
      c.face_yaw = fy;

      // P9 anti-gyro residual: time offset between bullet impact and the
      // moment the face yaw matches the bearing (face normal-on the barrel).
      // Sign carries direction; the gate checks magnitude.
      if (std::abs(x_(7)) > 0.1) {
        double e = angles::shortest_angular_distance(c.face_yaw, c.bearing);
        c.residual = e / x_(7);
      } else {
        c.residual = 0.0;
      }

      if (!found || margin > best.margin ||
          (std::abs(margin - best.margin) < 0.01 && vis > best.vis)) {
        best = c; found = true;
      }

      // P9: track the face with the smallest |residual| as a candidate
      // anti-gyro pick. Restrict to faces with non-negative margin so the
      // gate is geometric AND timing-based, not just timing.
      if (anti_gyro_mode && c.margin >= 0.0 &&
          (!found_residual || std::abs(c.residual) < std::abs(best_residual.residual))) {
        best_residual = c; found_residual = true;
      }
    }
    if (found) pred_t = best.flight_time + pred_lead_extra;
  }

  // Switch to the anti-gyro pick when active and a viable face exists.
  if (anti_gyro_mode && found_residual) {
    best = best_residual;
  }

  if (!found) return aim;
  aim.tracking = true;

  // absolute target angles in the IMU/odom startup frame.
  aim.abs_yaw   = angles::normalize_angle(best.bearing);
  aim.abs_pitch = best.abs_pitch;

  // relative angular error is only for fire/debug.
  aim.rel_yaw   = angles::shortest_angular_distance(cur_yaw, aim.abs_yaw);
  aim.rel_pitch = aim.abs_pitch - cur_pitch;
  aim.distance  = best.range;
  aim.target_valid = true;
  aim.target_x = best.x;
  aim.target_y = best.y;
  aim.target_z = best.z;

  // clamp only the relative error used by fire/debug.
  double max_rad = 30.0 * M_PI / 180.0;
  aim.rel_yaw   = std::clamp(aim.rel_yaw, -max_rad, max_rad);
  aim.rel_pitch = std::clamp(aim.rel_pitch, -max_rad, max_rad);

  aim.face_index = best.idx;
  aim.flight_time = best.flight_time;
  aim.pred_t = pred_t;
  aim.fire_margin = best.margin;
  aim.anti_gyro_residual = best.residual;
  aim.anti_gyro_active = anti_gyro_mode;

  aim.fire = best.valid &&
    state_ == TRACKING &&
    best.range >= cfg_.min_fire_dist &&
    best.range <= cfg_.max_fire_dist &&
    best.margin >= 0.0;

  return aim;
}

}  // namespace auto_aim
