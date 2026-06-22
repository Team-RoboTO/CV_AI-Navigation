// =========================================================================
// tracker.cpp — Single-file EKF tracker + aim solver for RoboMaster.
//
// This replaces 4 packages / 15 files / ~4000 lines with ~450 lines.
// Same core math (9D spinning-top EKF), stripped of all rarely-useful
// features that created debugging complexity during competition.
//
// WHAT'S HERE:
//   - 9D EKF:  [xc, vxc, yc, vyc, za, vza, yaw, vyaw, r]
//   - Armor jump detection (face switches at 90°/180°)
//   - Obliquity-aware measurement noise
//   - Radius adaptation (simple EMA instead of scalar KF)
//   - 4-face prediction at bullet arrival time
//   - Iterative ballistic solver (gravity compensation)
//   - Smart target switching (closer enemy steals focus)
//
// WHAT WAS REMOVED (and why):
//   - Multi-tracker infrastructure → you shoot one robot at a time
//   - Scalar Kalman filters for radius → EMA is simpler, sufficient
//   - Secondary face fusion → marginal gain, large complexity
//   - Adaptive damping → fixed damping with coast mode is fine
//   - Innovation-based acceleration → process noise handles it
//   - Indirect fire mode → direct mode with face prediction covers spinning
//   - Separate trajectory solver node → eliminates inter-node bugs
// =========================================================================

#include "autoaim/tracker.hpp"
#include <angles/angles.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace autoaim
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

// =========================================================================
// EKF PREDICT — damped constant-velocity model.
//
// Each state pair (pos, vel) evolves as:
//   pos_new = pos + vel * damping * dt
//   vel_new = vel * damping
//
// Damping is time-normalized: damping = alpha^(dt * ref_freq)
//   At ref_freq (30Hz): damping = alpha (configured value)
//   At 60Hz:            damping = sqrt(alpha) (less per frame, same per second)
//   At 15Hz:            damping = alpha^2 (more per frame, same per second)
// =========================================================================
void Tracker::ekfPredict(double dt)
{
  double b, a;
  if (state_ == TEMP_LOST) {
    // Aggressive damping: no measurement to correct, so decay velocity fast
    b = a = std::pow(cfg_.alpha_coast, dt * cfg_.ref_freq);
  } else {
    b = std::pow(cfg_.alpha_pos, dt * cfg_.ref_freq);
    a = std::pow(cfg_.alpha_yaw, dt * cfg_.ref_freq);
  }

  // State transition f(x)
  Eigen::VectorXd xn = x_;
  xn(0) += x_(1)*b*dt;  xn(1) = x_(1)*b;  // xc, vxc
  xn(2) += x_(3)*b*dt;  xn(3) = x_(3)*b;  // yc, vyc
  xn(4) += x_(5)*b*dt;  xn(5) = x_(5)*b;  // za, vza
  xn(6) += x_(7)*a*dt;  xn(7) = x_(7)*a;  // yaw, vyaw

  // Jacobian F
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(9, 9);
  F(0,1) = b*dt; F(1,1) = b;
  F(2,3) = b*dt; F(3,3) = b;
  F(4,5) = b*dt; F(5,5) = b;
  F(6,7) = a*dt; F(7,7) = a;

  // Process noise Q: discrete white-noise acceleration model. STATIC q_pos for
  // all center axes (xc, yc, za) and STATIC q_yaw for the spin. The old online
  // q_pos adaptation was removed (it amplified yaw/PnP model error into the
  // center estimate and caused lead overshoot).
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(9, 9);
  double t = dt;
  auto block = [&](int i, double s2) {
    Q(i,i) = std::pow(t,4)/4*s2;  Q(i,i+1) = std::pow(t,3)/2*s2;
    Q(i+1,i) = Q(i,i+1);          Q(i+1,i+1) = t*t*s2;
  };
  block(0, cfg_.q_pos); block(2, cfg_.q_pos); block(4, cfg_.q_pos);
  block(6, cfg_.q_yaw);
  Q(8,8) = std::pow(t,4)/4 * cfg_.q_r;

  x_ = xn;
  P_ = F * P_ * F.transpose() + Q;
  P_ = (P_ + P_.transpose()) * 0.5;

  // Clamp covariance (prevents explosion during TEMP_LOST)
  Eigen::VectorXd mc(9);
  mc << 1.0, 10.0, 1.0, 10.0, 1.0, 2.0, 1.0, 30.0, 0.01;
  for (int i = 0; i < 9; i++) {
    if (P_(i,i) > mc(i)) {
      double s = std::sqrt(mc(i) / std::max(P_(i,i), 1e-10));
      P_.row(i) *= s;  P_.col(i) *= s;
    }
  }
}

FacePrediction Tracker::predictFace(const Eigen::VectorXd & x, int face_idx, double t) const
{
  FacePrediction face;
  const int idx = ((face_idx % 4) + 4) % 4;
  const double offset = idx * M_PI / 2.0;
  const bool odd_face = (idx % 2) == 1;
  const double radius = odd_face ? other_radius_ : x(8);
  const double z_offset = odd_face ? dz_ : 0.0;
  const double theta = x(6) + x(7) * t + offset;

  face.idx = idx;
  face.yaw = theta;
  face.radius = radius;
  face.z_offset = z_offset;
  face.position = Eigen::Vector3d(
    x(0) + x(1) * t - radius * std::cos(theta),
    x(2) + x(3) * t - radius * std::sin(theta),
    x(4) + x(5) * t + z_offset);
  return face;
}

Eigen::Matrix4d Tracker::measurementNoise(
  const ArmorDetection & det,
  double face_yaw,
  bool yaw_valid) const
{
  const double rel_range = det.range() > 1e-6 ?
    det.range() :
    std::sqrt(
      std::pow(det.x - ego_x_, 2) +
      std::pow(det.y - ego_y_, 2) +
      std::pow(det.z - cfg_.gimbal_height, 2));
  // ── ANISOTROPIC position noise (monocular-PnP aware) ──
  // PnP localises a small planar plate precisely in BEARING (pixel-accurate) but
  // poorly in DEPTH (range). Isotropic noise forced the EKF to over-smooth the
  // precise lateral channel just to tolerate depth noise, so it LAGGED a
  // translating target and OVERSHOT on stop. Split it: radial (along the line of
  // sight) = large and range-growing (weak depth); tangential (perpendicular) =
  // small (strong bearing). Vertical z uses the tangential scale. WORKLOG §4b.
  const double s_rad = cfg_.r_pos_base + cfg_.r_pos_slope * rel_range;
  const double s_tan = cfg_.r_pos_tang_base + cfg_.r_pos_tang_slope * rel_range;
  const double ys = cfg_.r_yaw_base + cfg_.r_yaw_slope * rel_range;

  double xyz_f = 1.0;
  double yaw_f = 1.0;
  double bearing = 0.0;
  const double dx = det.x - ego_x_;
  const double dy = det.y - ego_y_;
  if (dx * dx + dy * dy > 0.01) {
    bearing = std::atan2(dy, dx);
    const double face_a =
      std::abs(angles::shortest_angular_distance(face_yaw, bearing));
    const double cf = std::cos(face_a);
    xyz_f = 1.0 / std::max(cf * cf, 0.04);
    yaw_f = 1.0 / std::max(std::pow(std::abs(cf), 4.0), 1e-4);
    // U-shaped yaw trust: ALSO distrust near face-on, where PnP yaw is poorly
    // observable (small yaw barely changes the image — the planar flip regime).
    // Without this, a static near-frontal plate feeds yaw noise straight into
    // vyaw as phantom spin and the facing/fire margin wanders. See WORKLOG §4.
    const double sf = std::abs(std::sin(face_a));
    yaw_f /= std::max(sf * sf, cfg_.yaw_facing_obs_floor);
    if (face_a > cfg_.max_oblique_deg * M_PI / 180.0) {
      yaw_f = 1e6;
    }
  }

  // Rotate diag(radial, tangential) by the bearing into odom x,y. Obliquity
  // (xyz_f) inflates both — an edge-on plate degrades the whole position fix.
  // Eigenvalues stay {vr, vt} > 0, so R is positive-definite.
  const double vr = s_rad * s_rad * xyz_f;
  const double vt = s_tan * s_tan * xyz_f;
  const double cb = std::cos(bearing);
  const double sb = std::sin(bearing);
  Eigen::Matrix4d R = Eigen::Matrix4d::Zero();
  R(0, 0) = vr * cb * cb + vt * sb * sb;
  R(1, 1) = vr * sb * sb + vt * cb * cb;
  R(0, 1) = R(1, 0) = (vr - vt) * cb * sb;
  R(2, 2) = vt;  // vertical ~ tangential (bearing) precision
  R(3, 3) = yaw_valid ? ys * ys * yaw_f : 1e6;
  return R;
}

Eigen::VectorXd Tracker::ekfUpdateFace(
  const ArmorDetection & det,
  int face_idx,
  double yaw_meas,
  bool yaw_valid)
{
  if (!std::isfinite(det.x) || !std::isfinite(det.y) || !std::isfinite(det.z)) {
    return x_;
  }

  const int idx = ((face_idx % 4) + 4) % 4;
  const bool odd_face = (idx % 2) == 1;
  const FacePrediction face = predictFace(x_, idx, 0.0);
  const double theta = face.yaw;
  const double r = face.radius;

  Eigen::Matrix<double, 4, 9> H = Eigen::Matrix<double, 4, 9>::Zero();
  H(0, 0) = 1.0;
  H(0, 6) = r * std::sin(theta);
  H(1, 2) = 1.0;
  H(1, 6) = -r * std::cos(theta);
  H(2, 4) = 1.0;
  if (!odd_face) {
    H(0, 8) = -std::cos(theta);
    H(1, 8) = -std::sin(theta);
  }
  if (yaw_valid) {
    H(3, 6) = 1.0;
  }

  const Eigen::Matrix4d R =
    measurementNoise(det, yaw_valid ? yaw_meas : face.yaw, yaw_valid);
  Eigen::Vector4d innovation;
  innovation << det.x - face.position.x(),
    det.y - face.position.y(),
    det.z - face.position.z(),
    yaw_valid ? angles::shortest_angular_distance(face.yaw, yaw_meas) : 0.0;

  const Eigen::Matrix4d S = H * P_ * H.transpose() + R;
  Eigen::LDLT<Eigen::Matrix4d> S_ldlt(S);
  if (S_ldlt.info() != Eigen::Success || !S_ldlt.isPositive()) {
    return x_;
  }

  const Eigen::Matrix<double, 9, 4> K =
    S_ldlt.solve(H * P_.transpose()).transpose();
  x_ = x_ + K * innovation;

  const Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(9, 9) - K * H;
  P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
  P_ = (P_ + P_.transpose()) * 0.5;
  return x_;
}

double Tracker::ekfMahalanobisFace(
  const ArmorDetection & det,
  int face_idx,
  double yaw_meas,
  bool yaw_valid) const
{
  if (!std::isfinite(det.x) || !std::isfinite(det.y) || !std::isfinite(det.z)) {
    return 1e9;
  }

  const int idx = ((face_idx % 4) + 4) % 4;
  const bool odd_face = (idx % 2) == 1;
  const FacePrediction face = predictFace(x_, idx, 0.0);
  const double theta = face.yaw;
  const double r = face.radius;

  Eigen::Matrix<double, 4, 9> H = Eigen::Matrix<double, 4, 9>::Zero();
  H(0, 0) = 1.0;
  H(0, 6) = r * std::sin(theta);
  H(1, 2) = 1.0;
  H(1, 6) = -r * std::cos(theta);
  H(2, 4) = 1.0;
  if (!odd_face) {
    H(0, 8) = -std::cos(theta);
    H(1, 8) = -std::sin(theta);
  }
  if (yaw_valid) {
    H(3, 6) = 1.0;
  }

  const Eigen::Matrix4d R =
    measurementNoise(det, yaw_valid ? yaw_meas : face.yaw, yaw_valid);
  Eigen::Vector4d innovation;
  innovation << det.x - face.position.x(),
    det.y - face.position.y(),
    det.z - face.position.z(),
    yaw_valid ? angles::shortest_angular_distance(face.yaw, yaw_meas) : 0.0;

  const Eigen::Matrix4d S = H * P_ * H.transpose() + R;
  Eigen::LDLT<Eigen::Matrix4d> S_ldlt(S);
  if (S_ldlt.info() != Eigen::Success) {
    return 1e9;
  }
  return (innovation.transpose() * S_ldlt.solve(innovation)).value();
}

Eigen::Vector3d Tracker::armorFromState(const Eigen::VectorXd & x) const
{
  return predictFace(x, 0, 0.0).position;
}

double Tracker::unwrapYaw(double raw_yaw)
{
  double yaw = last_yaw_ + angles::shortest_angular_distance(last_yaw_, raw_yaw);
  last_yaw_ = yaw;
  return yaw;
}

double Tracker::targetRange() const
{
  // Camera-relative distance: x_(0)/x_(2) are world coords of the target center,
  // ego_x_/ego_y_ are world coords of the robot. The difference is what matters
  // for "is the target close enough to switch to". Falling back to origin-relative
  // (the old behavior) made shouldSwitch comparisons inconsistent with detection's
  // rel_range once the robot moved.
  double dx = x_(0) - ego_x_;
  double dy = x_(2) - ego_y_;
  double dz = x_(4) - cfg_.gimbal_height;  // approx camera height
  return std::sqrt(dx*dx + dy*dy + dz*dz);
}

bool Tracker::fireStatePermits() const
{
  if (state_ == TRACKING) {
    return true;
  }

  return
    state_ == TEMP_LOST &&
    cfg_.fire_in_temp_lost &&
    lost_count_ <= cfg_.temp_lost_fire_max &&
    phaseConfident();
}

bool Tracker::phaseConfident() const
{
  if (std::abs(x_(7)) < cfg_.face_lookahead_min_vyaw) {
    return true;
  }
  return phase_timing_confident_;
}

Tracker::AssociationHypothesis Tracker::associateDetections(
  const std::vector<ArmorDetection> & detections) const
{
  AssociationHypothesis best;
  // Comparison key for `best`: the raw Mahalanobis plus the IPPE alternate
  // penalty. Stored separately so best.mahalanobis stays RAW (the match gate
  // must not see the penalty), while selection between the two yaw solutions of
  // the same face gets the hysteresis bias. See WORKLOG §4 (cause 3).
  double best_cmp = std::numeric_limits<double>::infinity();

  struct YawHypothesis
  {
    double yaw = 0.0;
    bool valid = false;
    int id = -1;
  };

  for (const auto & det : detections) {
    if (det.class_id != target_id_) {
      continue;
    }

    std::array<YawHypothesis, 3> yaw_hypotheses{};
    int yaw_count = 0;
    if (det.yaw_replaced_by_bearing) {
      yaw_hypotheses[yaw_count++] = YawHypothesis{0.0, false, -1};
    } else {
      yaw_hypotheses[yaw_count++] = YawHypothesis{det.yaw, true, 0};
      if (det.has_yaw_alt && !det.yaw_replaced_by_bearing_alt) {
        yaw_hypotheses[yaw_count++] = YawHypothesis{det.yaw_alt, true, 1};
      }
    }

    for (int face_idx = 0; face_idx < 4; ++face_idx) {
      const FacePrediction face = predictFace(x_, face_idx, 0.0);
      const Eigen::Vector3d det_pos(det.x, det.y, det.z);
      const double position_error = (det_pos - face.position).norm();

      for (int yi = 0; yi < yaw_count; ++yi) {
        const auto & yh = yaw_hypotheses[yi];
        const double yaw_meas = yh.valid ?
          face.yaw + angles::shortest_angular_distance(face.yaw, yh.yaw) :
          face.yaw;
        const double yaw_error = yh.valid ?
          std::abs(angles::shortest_angular_distance(face.yaw, yaw_meas)) :
          std::numeric_limits<double>::infinity();
        const double m =
          ekfMahalanobisFace(det, face_idx, yaw_meas, yh.valid);
        // IPPE hysteresis: the alternate (runner-up) yaw solution must beat the
        // primary by cfg_.ippe_alt_penalty to win, so near-face-on noise cannot
        // flip the chosen solution frame-to-frame (WORKLOG §4, cause 3). Only the
        // SELECTION key carries the penalty; best.mahalanobis stays raw.
        const double m_cmp = m + (yh.id == 1 ? cfg_.ippe_alt_penalty : 0.0);

        if (!best.valid ||
            m_cmp < best_cmp ||
            (std::abs(m_cmp - best_cmp) < 1e-6 &&
             position_error < best.position_error))
        {
          best.valid = true;
          best.det = &det;
          best.face_idx = face_idx;
          best.yaw_meas = yaw_meas;
          best.yaw_valid = yh.valid;
          best.yaw_hypothesis = yh.id;
          best.mahalanobis = m;
          best.position_error = position_error;
          best.yaw_error = yaw_error;
          best_cmp = m_cmp;
        }
      }
    }
  }

  return best;
}

void Tracker::initFromDetection(const ArmorDetection & det)
{
  last_yaw_ = 0;
  last_aim_face_idx_ = 0;
  double yaw = unwrapYaw(det.yaw);
  double r = cfg_.initial_radius;
  radius_ = r;  other_radius_ = r;
  dz_ = cfg_.initial_dz;
  dz_initialized_ = std::abs(dz_) > 1e-6;
  x_ = Eigen::VectorXd::Zero(9);
  x_ << det.x+r*cos(yaw), 0, det.y+r*sin(yaw), 0, det.z, 0, yaw, 0, r;
  P_ = P0_;
  target_id_ = det.class_id;
  target_generation_++;
  detect_count_ = 0;  lost_count_ = 0;
  switch_cooldown_counter_ = cfg_.switch_cooldown;
  // Reset timing estimator — new target has unknown spin phase/rate
  last_jump_time_valid_ = false;
  last_jump_dir_ = 0.0;
  consecutive_same_dir_jumps_ = 0;
  last_vyaw_timing_est_ = 0.0;
  last_matched_face_idx_ = 0;
  last_face_assoc_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_face_assoc_time_valid_ = false;
  phase_timing_confident_ = false;
}

// =========================================================================
// TARGET SWITCHING — should we drop the current target for this closer one?
//
// Policy:
//   YES if: current state is LOST (no target at all)
//   YES if: state is DETECTING and candidate is a DIFFERENT class_id and closer
//   YES if: TRACKING/TEMP_LOST and candidate class differs and is dramatically closer
//   NO  otherwise (stay on current target for stable aim)
//
// CRITICAL: during DETECTING we must only switch to a different target.
// If we returned YES for the same target every frame, initFromDetection()
// would reset detect_count_=0 forever and the tracker would never reach
// TRACKING. That was a real bug — fixed by requiring class_id != target_id_.
//
// The cooldown prevents rapid flipping between two equidistant robots.
// =========================================================================
// =========================================================================
// TARGET SWITCHING — should we drop the current target for this closer one?
//
// PHYSICAL IDENTITY VIA SPATIAL PROXIMITY
// class_id from the YOLO detector identifies COLOR (0=blue, 2=red), not the
// individual robot. In a match there are multiple enemy robots of the same
// color. We cannot use class_id to decide "is this the same physical target".
//
// Instead we compare the candidate detection's position to the predicted
// position of the currently tracked target (its visible face). If close →
// it's the same physical robot, do not switch. If far → it's a different
// robot, allow the switch decision based on range.
//
// Policy:
//   YES if: state is LOST (no current target)
//   NO  if: candidate is spatially close to current target's prediction
//   YES if: state is DETECTING, candidate is far AND closer to camera
//   YES if: state is TRACKING/TEMP_LOST, candidate is far AND much closer to camera
//           (with cooldown to prevent flapping)
//
// The "spatially close" test resolves the same-color-multiple-robots problem
// AND prevents the DETECTING-stuck bug, because a detection of the SAME robot
// will be close to where we just initialized, so shouldSwitch returns false
// and detect_count_ can increment normally.
// =========================================================================
bool Tracker::shouldSwitch(const ArmorDetection & candidate) const
{
  if (state_ == LOST) return true;

  // Class mismatch: definitely a different physical target.
  // (e.g. friendly blue robot detected while tracking a red enemy)
  if (candidate.class_id != target_id_) {
    // Different color → if closer, switch (no cooldown — could be the new
    // primary threat).
    double cur_range = targetRange();
    double new_range = candidate.range();
    return new_range < cur_range * cfg_.switch_range_ratio;
  }

  // ── Same class. Decide identity by nearest predicted face. ──
  // A different armor face on the same robot is still the same physical target.
  double spatial_dist = std::numeric_limits<double>::infinity();
  const Eigen::Vector3d candidate_pos(candidate.x, candidate.y, candidate.z);
  for (int face_idx = 0; face_idx < 4; ++face_idx) {
    const Eigen::Vector3d pred = predictFace(x_, face_idx, 0.0).position;
    spatial_dist = std::min(spatial_dist, (candidate_pos - pred).norm());
  }

  // If within identity threshold → same physical robot, no switch.
  // The threshold is a hyperparameter; same_target_identity_dist in cfg.
  // Default of 1.0 m means "if the new detection is within 1 m of where we
  // predicted the current armor, it's the same robot". Tune up if robots
  // can be very close together (back-to-back), tune down if false-merges
  // happen between two enemies passing close to each other.
  if (spatial_dist < cfg_.same_target_identity_dist) {
    return false;
  }

  // Spatially distinct → different physical robot of the same color.
  if (state_ == DETECTING) {
    // During DETECTING, switch if the new (different) robot is closer.
    double cur_range = targetRange();
    double new_range = candidate.range();
    return new_range < cur_range * cfg_.switch_range_ratio;
  }

  // TRACKING / TEMP_LOST: cooldown must elapse, and new target must be much closer.
  if (state_ == TRACKING && switch_cooldown_counter_ > 0) return false;
  double cur_range = targetRange();
  double new_range = candidate.range();
  return new_range < cur_range * cfg_.switch_range_ratio;
}

// =========================================================================
// FACE TRANSITION OBSERVER — spin timing only, never EKF ownership.
// =========================================================================
void Tracker::observeFaceAssociation(
  int matched_face_idx,
  const ArmorDetection & det,
  const rclcpp::Time & now,
  bool yaw_valid)
{
  debug_info_.last_matched_face_idx = last_matched_face_idx_;

  if (matched_face_idx < 0) {
    return;
  }

  if (last_matched_face_idx_ < 0) {
    last_matched_face_idx_ = matched_face_idx;
    last_face_assoc_time_ = now;
    last_face_assoc_time_valid_ = true;
    return;
  }

  int raw_step = (matched_face_idx - last_matched_face_idx_ + 4) % 4;
  int face_step = raw_step;
  if (face_step == 3) {
    face_step = -1;
  }
  const bool transition = face_step != 0;
  debug_info_.face_transition_observed = transition;

  if (transition) {
    const bool one_face_jump = std::abs(face_step) == 1;
    // If visible face index increments, the robot yaw moved the opposite way.
    const double yaw_dir = one_face_jump ? -static_cast<double>(face_step) : 0.0;
    debug_info_.jump_detected = true;
    debug_info_.jump_abs = std::abs(face_step) * M_PI / 2.0;
    debug_info_.jump_dir = yaw_dir;
    debug_info_.one_face_jump = one_face_jump;

    const bool reproj_ok =
      det.reproj_error >= 0.0 && det.reproj_error <= cfg_.vyaw_timing_max_reproj;
    const bool transition_credible =
      cfg_.use_vyaw_from_timing && one_face_jump && yaw_valid && reproj_ok;

    if (transition_credible && last_jump_time_valid_) {
      const double dt_jump = (now - last_jump_time_).seconds();
      const bool same_dir = yaw_dir * last_jump_dir_ > 0.0;
      debug_info_.dt_jump = dt_jump;

      if (same_dir && dt_jump >= cfg_.vyaw_timing_min_dt &&
          dt_jump <= cfg_.vyaw_timing_max_dt)
      {
        const double vyaw_est = (M_PI / 2.0) / dt_jump * yaw_dir;
        debug_info_.vyaw_est_from_timing = vyaw_est;
        const bool consistent =
          consecutive_same_dir_jumps_ >= 1 &&
          std::abs(vyaw_est - last_vyaw_timing_est_) <=
            cfg_.vyaw_timing_consistency * std::max(std::abs(vyaw_est), 1e-3);

        consecutive_same_dir_jumps_++;
        last_vyaw_timing_est_ = vyaw_est;

        if (consecutive_same_dir_jumps_ >= 2 && consistent) {
          const double blend = (consecutive_same_dir_jumps_ >= 3) ? 0.70 : 0.50;
          x_(7) = blend * vyaw_est + (1.0 - blend) * x_(7);
          P_(7, 7) = std::min(P_(7, 7), 1.5);
          phase_timing_confident_ = true;
          debug_info_.vyaw_timing_accepted = true;
        }
      } else if (!same_dir) {
        consecutive_same_dir_jumps_ = 0;
        last_vyaw_timing_est_ = 0.0;
        phase_timing_confident_ = false;
      }
    } else if (!transition_credible) {
      consecutive_same_dir_jumps_ = 0;
      last_vyaw_timing_est_ = 0.0;
      phase_timing_confident_ = false;
      last_jump_time_valid_ = false;
    }

    if (transition_credible) {
      last_jump_time_ = now;
      last_jump_time_valid_ = true;
      last_jump_dir_ = yaw_dir;
    }
  }

  last_matched_face_idx_ = matched_face_idx;
  last_face_assoc_time_ = now;
  last_face_assoc_time_valid_ = true;
}

void Tracker::updateRadiusFromAssociation(
  const ArmorDetection & det,
  int face_idx,
  bool oblique)
{
  if (oblique || face_idx < 0) {
    return;
  }

  const FacePrediction face = predictFace(x_, face_idx, 0.0);
  const double rm =
    (x_(0) - det.x) * std::cos(face.yaw) +
    (x_(2) - det.y) * std::sin(face.yaw);
  if (rm <= 0.10 || rm >= 0.45) {
    return;
  }

  if (face.idx % 2 == 0) {
    radius_ = (1.0 - cfg_.radius_ema_alpha) * radius_ + cfg_.radius_ema_alpha * rm;
    x_(8) = radius_;
  } else {
    other_radius_ =
      (1.0 - cfg_.radius_ema_alpha) * other_radius_ + cfg_.radius_ema_alpha * rm;
  }

  if (cfg_.adapt_dz_enable && face.idx % 2 == 1) {
    const double measured_dz = det.z - x_(4);
    if (std::abs(measured_dz) < 0.20) {
      if (!dz_initialized_) {
        dz_ = measured_dz;
        dz_initialized_ = true;
      } else {
        dz_ = 0.95 * dz_ + 0.05 * measured_dz;
      }
    }
  }
}

// =========================================================================
// UPDATE — main per-frame function.
// =========================================================================
void Tracker::update(const std::vector<ArmorDetection> & detections, double dt,
                     const rclcpp::Time & now)
{
  debug_info_ = TrackerDebugInfo{};
  debug_info_.state_yaw = x_(6);
  debug_info_.phase_confident = phaseConfident();
  debug_info_.last_matched_face_idx = last_matched_face_idx_;

  if (switch_cooldown_counter_ > 0) switch_cooldown_counter_--;

  // ── TARGET SWITCHING CHECK ──
  // Before doing anything else, check if a significantly closer target exists.
  if (!detections.empty() && state_ != LOST) {
    const ArmorDetection * closest = nullptr;
    double min_range = 1e9;
    for (const auto & d : detections) {
      if (d.range() < min_range) { min_range = d.range(); closest = &d; }
    }
    if (closest && shouldSwitch(*closest)) {
      debug_info_.det_yaw = closest->yaw;
      debug_info_.yaw_replaced_by_bearing = closest->yaw_replaced_by_bearing;
      debug_info_.reproj_error = closest->reproj_error;
      debug_info_.selected_class_id = closest->class_id;
      initFromDetection(*closest);
      state_ = DETECTING;
      return;
    }
  }

  // ── LOST STATE: pick closest detection to start tracking ──
  if (state_ == LOST) {
    if (detections.empty()) return;
    auto best = std::min_element(detections.begin(), detections.end(),
      [](const auto& a, const auto& b) { return a.range() < b.range(); });
    debug_info_.det_yaw = best->yaw;
    debug_info_.yaw_replaced_by_bearing = best->yaw_replaced_by_bearing;
    debug_info_.reproj_error = best->reproj_error;
    debug_info_.selected_class_id = best->class_id;
    initFromDetection(*best);
    state_ = DETECTING;
    return;
  }

  lost_thresh_ = std::max(1, (int)(cfg_.lost_timeout / std::max(dt, 0.01)));

  // ── PREDICT ──
  ekfPredict(dt);

  // ── ASSOCIATE: detection × face × yaw-hypothesis Mahalanobis gating ──
  bool matched = false;
  const AssociationHypothesis assoc = associateDetections(detections);
  if (assoc.valid && assoc.det != nullptr) {
    debug_info_.mahalanobis = assoc.mahalanobis;
    debug_info_.association_mahalanobis = assoc.mahalanobis;
    debug_info_.matched_face_idx = assoc.face_idx;
    debug_info_.association_yaw_valid = assoc.yaw_valid;
    debug_info_.association_yaw_hypothesis = assoc.yaw_hypothesis;
    debug_info_.pd = assoc.position_error;
    debug_info_.yd = assoc.yaw_error;
    debug_info_.det_yaw = assoc.yaw_meas;
    debug_info_.yaw_replaced_by_bearing = assoc.det->yaw_replaced_by_bearing;
    debug_info_.reproj_error = assoc.det->reproj_error;
    debug_info_.selected_class_id = assoc.det->class_id;
    matched =
      assoc.mahalanobis < cfg_.maha_threshold &&
      assoc.position_error < cfg_.max_match_dist;
  }

  // ── UPDATE or COAST ──
  if (matched && assoc.det != nullptr) {
    const double state_yaw_before_update = x_(6);
    const ArmorDetection & best_det = *assoc.det;
    const FacePrediction face_before_update =
      predictFace(x_, assoc.face_idx, 0.0);
    const double bearing = std::atan2(best_det.y - ego_y_, best_det.x - ego_x_);
    const double obliquity_yaw = assoc.yaw_valid ? assoc.yaw_meas : face_before_update.yaw;
    const double fa =
      std::abs(angles::shortest_angular_distance(obliquity_yaw, bearing));
    bool oblique = fa > cfg_.max_oblique_deg * M_PI/180.0;

    debug_info_.matched = true;
    debug_info_.mahalanobis = assoc.mahalanobis;
    debug_info_.oblique = oblique;
    debug_info_.det_yaw = assoc.yaw_meas;
    debug_info_.state_yaw = state_yaw_before_update;
    debug_info_.yaw_replaced_by_bearing = best_det.yaw_replaced_by_bearing;
    debug_info_.reproj_error = best_det.reproj_error;
    debug_info_.selected_class_id = best_det.class_id;

    x_ = ekfUpdateFace(best_det, assoc.face_idx, assoc.yaw_meas, assoc.yaw_valid);
    updateRadiusFromAssociation(best_det, assoc.face_idx, oblique);
    observeFaceAssociation(assoc.face_idx, best_det, now, assoc.yaw_valid);
  } else if (assoc.valid) {
    debug_info_.matched = false;
  }

  // ── SAFETY CLAMPS ──
  x_(8) = std::clamp(x_(8), 0.12, 0.40);
  radius_ = x_(8);
  other_radius_ = std::clamp(other_radius_, 0.12, 0.40);

  // ── DYNAMIC SPIN-RATE BOUND ──
  // Old approach: hard clamp at 25 rad/s + full P_ reset.
  // Problem: at 300 RPM the real spin is 31.4 rad/s → clamp fires every frame,
  //          P_ resets every frame, EKF never converges, face prediction is wrong.
  //
  // New approach: soft physical cap at the mechanical limit of any RoboMaster chassis
  //   (~500 RPM = 52.4 rad/s — no robot can physically exceed this).
  //   When the estimate hits the cap we clamp the value but DO NOT reset P_.
  //   Instead, we only inflate the vyaw diagonal so the EKF is free to correct
  //   downward if subsequent measurements disagree with the high spin rate.
  //   At 200 RPM (20.9 rad/s) or 300 RPM (31.4 rad/s) the cap is never reached
  //   and the EKF runs completely unconstrained — it will learn the real spin rate
  //   from the face-jump timing, regardless of whether the enemy is spinning fast
  //   or slow or changing speed mid-match.
  static constexpr double VYAW_PHYSICAL_MAX = 52.4;  // 500 RPM in rad/s
  if (std::abs(x_(7)) > VYAW_PHYSICAL_MAX) {
    x_(7) = std::copysign(VYAW_PHYSICAL_MAX, x_(7));
    // Inflate only the vyaw variance row/col, not full P_ reset.
    // This allows the EKF to pull the estimate back down via measurement updates
    // if it was a transient spike rather than a real high-RPM spin.
    P_(7, 7) = std::max(P_(7, 7), 15.0);
    P_.row(7) *= 1.2;  P_.col(7) *= 1.2;
    P_ = (P_ + P_.transpose()) * 0.5;
  }

  double wy = angles::normalize_angle(x_(6));
  if (wy != x_(6)) { x_(6) = wy; last_yaw_ = wy; }
  debug_info_.phase_confident = phaseConfident();

  // ── STATE MACHINE ──
  if (state_ == DETECTING) {
    if (matched) {
      detect_count_++;
      lost_count_ = 0;  // reset miss counter on match
      if (detect_count_ >= cfg_.confirm_frames) {
        state_ = TRACKING; detect_count_ = 0; lost_count_ = 0;
        switch_cooldown_counter_ = cfg_.switch_cooldown;
      }
    } else {
      // Tolerate up to 3 missed frames during DETECTING.
      // YOLO can miss 1-2 frames due to motion blur, partial occlusion,
      // or confidence fluctuating around the threshold.
      // Without tolerance, the tracker flickers: LOST→DETECTING→LOST...
      detect_count_ = 0;  // reset consecutive match counter
      if (++lost_count_ > 3) {
        state_ = LOST;
        lost_count_ = 0;
        last_aim_face_idx_ = 0;
        last_matched_face_idx_ = -1;
        last_face_assoc_time_valid_ = false;
        last_jump_time_valid_ = false;
        phase_timing_confident_ = false;
      }
    }
  } else if (state_ == TRACKING) {
    if (matched) {
      lost_count_ = 0;
    } else if (++lost_count_ > cfg_.track_grace_misses) {
      state_ = TEMP_LOST;
      lost_count_ = 1;  // restart the TEMP_LOST timeout/coast-fire counter
    }
  } else if (state_ == TEMP_LOST) {
    if (matched) { state_ = TRACKING; lost_count_ = 0; }
    else if (++lost_count_ > lost_thresh_) {
      state_ = LOST;
      last_aim_face_idx_ = 0;
      last_matched_face_idx_ = -1;
      last_face_assoc_time_valid_ = false;
      last_jump_time_valid_ = false;
      phase_timing_confident_ = false;
    }
  }
}

// =========================================================================
// BALLISTIC SOLVER — iterative pitch for gravity.
// =========================================================================
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

// =========================================================================
// COMPUTE AIM — predict which face the bullet will hit.
//
// At 200-300 RPM with 4 plates, each plate is visible for ~50-75ms.
// Bullet at 3m takes ~120ms. So we MUST predict the future face position.
//
// BARREL OFFSET COMPENSATION:
//   The camera sees targets from one position but the bullet exits from
//   the barrel, which is typically 5-15cm below/beside the camera.
//   At 1.5m range with 10cm offset, parallax is ~3.8° — bigger than the
//   armor plate angular width. We compute the aim vector from the BARREL
//   position, not the camera position.
//
//   barrel_odom = R(gimbal_yaw) × barrel_offset_body + [0, 0, gimbal_height]
//   aim_vector  = target_face_position − barrel_odom
//
// Algorithm:
//   1. Compute barrel position in odom frame (rotated with gimbal)
//   2. For each of 4 faces, predict position at time T_impact
//   3. Compute bearing + elevation from BARREL to face
//   4. Pick a face for aim planning:
//      - fire-valid faces first,
//      - otherwise incoming faces that will enter the window soon,
//      - otherwise the best current-margin fallback
//   5. Solve ballistics (gravity) for that face
//   6. Compute absolute target angles and relative error
// =========================================================================
AimResult Tracker::computeAim(double cur_yaw, double cur_pitch,
                              double robot_x, double robot_y,
                              double robot_vx, double robot_vy) const
{
  AimResult aim;
  if (state_ != TRACKING && state_ != TEMP_LOST) return aim;
  aim.tracking = true;

  // ── Barrel position in odom frame (CURRENT) ──
  // The barrel offset is fixed in the gimbal body frame.
  // Rotate by current gimbal yaw to get odom coordinates.
  // Add robot_x/robot_y so the barrel is at the robot's CURRENT position,
  // not at the world origin. Without this, the aim vector (target − barrel)
  // has the wrong range and bearing once the robot has moved.
  double cos_y = std::cos(cur_yaw), sin_y = std::sin(cur_yaw);
  double barrel_x_now = robot_x + cfg_.barrel_offset_x * cos_y - cfg_.barrel_offset_y * sin_y;
  double barrel_y_now = robot_y + cfg_.barrel_offset_x * sin_y + cfg_.barrel_offset_y * cos_y;
  double barrel_z = cfg_.gimbal_height + cfg_.barrel_offset_z;

  // Seed the prediction horizon from the actual current target distance from
  // the camera, not a fixed 1.5 m constant. Also: must be relative to the robot,
  // not the world origin, because once ego-motion has moved us those are different.
  // At 2.5 m / 25 m/s the real flight is ~0.10 s; 1.5/25 = 0.06 s was wrong for
  // any range other than 1.5 m.
  double tx_dx = x_(0) - robot_x;
  double tx_dy = x_(2) - robot_y;
  // FIX: vertical term must be the height difference armor-vs-gimbal, not the
  // armor height above ground. Only affects the first iteration (loop refines).
  double tx_dz = x_(4) - cfg_.gimbal_height;
  double dist_now = std::sqrt(tx_dx*tx_dx + tx_dy*tx_dy + tx_dz*tx_dz);
  // predictionBias() = measured pipeline latency + actuation_latency
  // (or the fixed time_bias fallback). See TrackerConfig.
  double pred_t = predictionBias() + dist_now / std::max(cfg_.bullet_speed, 1.0);
  constexpr int FACES = 4;

  // Face lookahead is aim planning, not fire permission. The fire window still
  // decides whether a shot is safe (margin >= 0). Lookahead merely points the
  // gimbal at an incoming face early enough to settle before that face becomes
  // fire-valid, avoiding the old failure mode of tracking an outgoing face and
  // arriving late on the next plate. Keep the spin-confidence gate so untrusted
  // phase estimates do not re-enable wrong four-face prediction.
  const bool spin_phase_trusted = phaseConfident();
  // Apply the vyaw (rotational) lead to the AIM only for a CONFIRMED spinning
  // top. A translating target leaks its position innovation into vyaw through the
  // EKF position↔yaw coupling (phantom spin, e.g. vyaw≈-0.1 on a straight-moving
  // enemy); leading the aim by that phantom rotation drifts the shot. Translation
  // lead (vxc,vyc) ALWAYS applies — only the rotation is gated. WORKLOG §4b.
  const bool apply_spin_lead =
    phase_timing_confident_ && std::abs(x_(7)) >= cfg_.face_lookahead_min_vyaw;
  Eigen::VectorXd x_aim = x_;
  if (!apply_spin_lead) {
    x_aim(7) = 0.0;
  }
  const bool face_lookahead_active =
    cfg_.face_lookahead_enable &&
    spin_phase_trusted &&
    std::abs(x_(7)) >= cfg_.face_lookahead_min_vyaw;
  std::array<int, FACES> face_indices{};
  int faces_to_check = 0;
  if (spin_phase_trusted) {
    for (int i = 0; i < FACES; ++i) {
      face_indices[faces_to_check++] = i;
    }
  } else {
    const int held_face =
      (last_matched_face_idx_ >= 0 && last_matched_face_idx_ < FACES) ?
      last_matched_face_idx_ : 0;
    face_indices[faces_to_check++] = held_face;
  }
  aim.faces_checked = faces_to_check;
  aim.face_lookahead_active = face_lookahead_active;

  enum class FaceChoice {
    None,
    FireValid,
    Incoming,
    Fallback
  };

  struct Face {
    double x = 0, y = 0, z = 0, range = 0, bearing = 0;
    double abs_pitch = 0, flight_time = 0, phase_error = 0, fire_window = 0;
    double margin = -1e9, vis = 0;
    double phase_error_future = 0.0, margin_future = -1e9;
    double dwell_time = 0.0;
    double time_to_window = std::numeric_limits<double>::infinity();
    double score = std::numeric_limits<double>::infinity();
    int idx = -1;
    bool valid = false;
    bool fire_valid = false;
    bool incoming = false;
    bool approaching = false;
  };
  std::array<Face, FACES> candidates{};
  Face best;
  bool found = false;
  FaceChoice best_choice = FaceChoice::None;
  bool held_by_hysteresis = false;
  std::string fallback_reason = "fallback_margin";
  const int previous_face_idx =
    (last_aim_face_idx_ >= 0 && last_aim_face_idx_ < FACES) ? last_aim_face_idx_ : 0;

  auto better_margin = [&candidates](int idx, int best_idx) {
    if (best_idx < 0) return true;
    const auto & a = candidates[idx];
    const auto & b = candidates[best_idx];
    if (a.margin > b.margin + 0.01) return true;
    if (std::abs(a.margin - b.margin) < 0.01 && a.vis > b.vis) return true;
    return false;
  };

  auto better_incoming = [&candidates](int idx, int best_idx) {
    if (best_idx < 0) return true;
    const auto & a = candidates[idx];
    const auto & b = candidates[best_idx];
    if (a.time_to_window < b.time_to_window - 1e-3) return true;
    if (std::abs(a.time_to_window - b.time_to_window) < 1e-3) {
      if (a.margin > b.margin + 0.01) return true;
      if (std::abs(a.margin - b.margin) < 0.01 && a.vis > b.vis) return true;
    }
    return false;
  };

  // Two iterations: first rough estimate, then refine with actual flight time.
  // Three iterations converge tightly for ranges 0.5–6 m with any bullet speed.
  for (int iter = 0; iter < 3; iter++) {
    // ── Barrel position at IMPACT TIME (account for our own motion) ──
    // During (flight_time + latency bias), our robot is also moving. The bullet
    // exits from where the barrel will be in the FUTURE, not where it is now.
    // At 1 m/s over 0.1 s that's 10 cm — significant vs armor half-width.
    // When stationary or ego_velocity_available=false, robot_vx/vy are zero
    // and this reduces to the current barrel position (no harm done).
    double barrel_x_future = barrel_x_now + robot_vx * pred_t;
    double barrel_y_future = barrel_y_now + robot_vy * pred_t;

    candidates = {};
    int best_fire_idx = -1;
    int best_incoming_idx = -1;
    int best_fallback_idx = -1;
    bool nonfire_seen = false;
    bool approaching_seen = false;
    bool blocked_by_horizon = false;
    bool blocked_by_approach = false;

    for (int face_slot = 0; face_slot < faces_to_check; face_slot++) {
      const int fi = face_indices[face_slot];
      const FacePrediction face = predictFace(x_aim, fi, pred_t);
      const double fy = face.yaw;
      const double fx = face.position.x();
      const double fpy = face.position.y();
      const double fz = face.position.z();

      // ── AIM VECTOR: from FUTURE barrel to FUTURE target face ──
      double dx = fx - barrel_x_future;
      double dy = fpy - barrel_y_future;
      double dz_aim = fz - barrel_z;

      double gd = std::hypot(dx, dy);           // ground distance from barrel
      double range = std::sqrt(dx*dx + dy*dy + dz_aim*dz_aim);
      double bearing = std::atan2(dy, dx);       // yaw from barrel to target

      // Visibility: how face-on is the plate to the camera at impact time?
      // (Use predicted camera/robot position — robot may be moving during flight)
      double robot_x_future = robot_x + robot_vx * pred_t;
      double robot_y_future = robot_y + robot_vy * pred_t;
      double fn = fy + M_PI;
      double cd = std::atan2(robot_y_future - fpy, robot_x_future - fx);
      double obl = std::abs(angles::shortest_angular_distance(fn, cd));
      double vis = std::clamp(std::cos(obl), 0.0, 1.0);

      // Ballistic solution from barrel to target
      auto bal = solveBallistic(gd, dz_aim);
      if (!bal.valid) continue;

      // Fire window margin: positive = face is aligned with bearing.
      // phase_error is a FACING angle (plate normal vs line of sight); its
      // tolerance must NOT grow at close range. The old cap of 2.0 let the
      // window DOUBLE up close (angular_window 0.40 → 0.80 rad ≈ 46°), so the
      // shot could be taken on a plate 45° off-facing — exactly the "hits the
      // wheels / 45° to the side" symptom. Cap at 1.0: the window only SHRINKS
      // with distance (where the same facing error is a larger linear miss),
      // never expands beyond the configured angular_window.
      double ds = std::min(cfg_.window_ref_dist / std::max(range, 0.5), 1.0);
      double win = cfg_.angular_window * ds;
      double phase_error = angles::shortest_angular_distance(bearing, fy);
      double err = std::abs(phase_error);
      double margin = win - err;

      auto eval_phase = [&](double t_eval) {
        const FacePrediction eval_face = predictFace(x_aim, fi, t_eval);
        const double bx = barrel_x_now + robot_vx * t_eval;
        const double by = barrel_y_now + robot_vy * t_eval;
        const double dx_eval = eval_face.position.x() - bx;
        const double dy_eval = eval_face.position.y() - by;
        const double dz_eval = eval_face.position.z() - barrel_z;
        const double range_eval =
          std::sqrt(dx_eval * dx_eval + dy_eval * dy_eval + dz_eval * dz_eval);
        const double bearing_eval = std::atan2(dy_eval, dx_eval);
        const double win_eval =
          cfg_.angular_window *
          std::min(cfg_.window_ref_dist / std::max(range_eval, 0.5), 1.0);
        const double phase_eval =
          angles::shortest_angular_distance(bearing_eval, eval_face.yaw);
        return std::array<double, 3>{
          phase_eval,
          win_eval - std::abs(phase_eval),
          std::abs(phase_eval)
        };
      };

      const double future_dt =
        cfg_.face_lookahead_horizon > 0.0 ?
        std::min(std::max(cfg_.face_lookahead_horizon, 0.02), 0.12) :
        0.02;
      const auto future_phase = eval_phase(pred_t + future_dt);
      const double phase_error_future = future_phase[0];
      const double margin_future = future_phase[1];
      const bool approaching =
        margin >= 0.0 || future_phase[2] < err - 0.01;

      double excess = err - win;
      double time_to_window = 0.0;
      if (excess <= 0.0) {
        time_to_window = 0.0;
      } else if (approaching && std::abs(x_(7)) > 1e-3) {
        time_to_window = excess / std::abs(x_(7));
      } else {
        time_to_window = std::numeric_limits<double>::infinity();
      }

      Face c;
      c.idx = fi;
      c.x = fx; c.y = fpy; c.z = fz;
      c.range = range; c.bearing = bearing;
      c.abs_pitch = bal.pitch; c.flight_time = bal.flight_time;
      c.phase_error = phase_error;
      c.fire_window = win;
      c.margin = margin; c.vis = vis; c.valid = true;
      c.phase_error_future = phase_error_future;
      c.margin_future = margin_future;
      c.fire_valid = margin >= 0.0;
      c.time_to_window = time_to_window;
      c.approaching = approaching;
      c.incoming =
        face_lookahead_active &&
        !c.fire_valid &&
        c.approaching &&
        c.time_to_window <= cfg_.face_lookahead_horizon;
      // Lower is better. This keeps hysteresis in radians for both fire-valid
      // and incoming/fallback faces: it is just the negative fire margin.
      c.score = -c.margin;

      candidates[fi] = c;
      if (c.fire_valid && better_margin(fi, best_fire_idx)) {
        best_fire_idx = fi;
      }
      if (c.incoming && better_incoming(fi, best_incoming_idx)) {
        best_incoming_idx = fi;
      }
      if (better_margin(fi, best_fallback_idx)) {
        best_fallback_idx = fi;
      }
      if (face_lookahead_active && !c.fire_valid) {
        nonfire_seen = true;
        if (c.approaching) {
          approaching_seen = true;
          if (c.time_to_window > cfg_.face_lookahead_horizon) {
            blocked_by_horizon = true;
          }
        } else {
          blocked_by_approach = true;
        }
      }
    }

    int chosen_idx = -1;
    FaceChoice choice = FaceChoice::None;
    if (best_fire_idx >= 0) {
      chosen_idx = best_fire_idx;
      choice = FaceChoice::FireValid;
    } else if (best_incoming_idx >= 0) {
      chosen_idx = best_incoming_idx;
      choice = FaceChoice::Incoming;
    } else if (best_fallback_idx >= 0) {
      chosen_idx = best_fallback_idx;
      choice = FaceChoice::Fallback;
    }

    if (chosen_idx < 0) {
      continue;
    }

    bool held = false;
    const int prev_idx =
      (previous_face_idx >= 0 && previous_face_idx < FACES) ? previous_face_idx : 0;
    if (prev_idx != chosen_idx && candidates[prev_idx].valid &&
        cfg_.face_switch_hysteresis > 0.0)
    {
      bool previous_same_stage = false;
      if (choice == FaceChoice::FireValid) {
        previous_same_stage = candidates[prev_idx].fire_valid;
      } else if (choice == FaceChoice::Incoming) {
        previous_same_stage = candidates[prev_idx].incoming;
      } else if (choice == FaceChoice::Fallback) {
        previous_same_stage = true;
      }

      if (previous_same_stage) {
        const double score_advantage =
          candidates[prev_idx].score - candidates[chosen_idx].score;
        if (score_advantage < cfg_.face_switch_hysteresis) {
          chosen_idx = prev_idx;
          held = true;
        }
      }
    }

    best = candidates[chosen_idx];
    found = true;
    best_choice = choice;
    held_by_hysteresis = held;
    if (choice == FaceChoice::Fallback) {
      if (!cfg_.face_lookahead_enable) {
        fallback_reason = "fallback_lookahead_disabled";
      } else if (!spin_phase_trusted) {
        fallback_reason = "fallback_phase_not_confident";
      } else if (std::abs(x_(7)) < cfg_.face_lookahead_min_vyaw) {
        fallback_reason = "fallback_spin_below_min";
      } else if (blocked_by_horizon) {
        fallback_reason = "fallback_incoming_beyond_horizon";
      } else if (blocked_by_approach || (nonfire_seen && !approaching_seen)) {
        fallback_reason = "fallback_no_approaching_face";
      } else {
        fallback_reason = "fallback_margin";
      }
    }
    pred_t = best.flight_time + predictionBias();
  }

  if (!found) return aim;
  last_aim_face_idx_ = best.idx;

  // Absolute target angles in the IMU/odom startup frame.
  // These are destinations: the microcontroller should compare them with
  // its measured yaw/pitch, not add them to the previous target.
  aim.abs_yaw   = angles::normalize_angle(best.bearing);
  aim.abs_pitch = best.abs_pitch;

  // Relative angular error from current measured head orientation to target.
  // Used for fire gating/debug; not sent as the main command.
  aim.rel_yaw   = angles::shortest_angular_distance(cur_yaw, aim.abs_yaw);
  aim.rel_pitch = aim.abs_pitch - cur_pitch;
  aim.distance  = best.range;
  aim.target_valid = true;
  aim.target_x = best.x;
  aim.target_y = best.y;
  aim.target_z = best.z;
  aim.best_face_idx = best.idx;
  aim.selected_face_idx = best.idx;
  aim.phase_error = best.phase_error;
  aim.fire_window = best.fire_window;
  aim.flight_time = best.flight_time;
  aim.prediction_time = pred_t;
  aim.fire_margin = best.margin;
  aim.face_time_to_window =
    std::isfinite(best.time_to_window) ? best.time_to_window : -1.0;
  aim.face_margin = best.margin;
  aim.face_phase_error = best.phase_error;
  if (held_by_hysteresis) {
    switch (best_choice) {
      case FaceChoice::FireValid:
        aim.face_switch_reason = "hysteresis_hold_fire_valid";
        break;
      case FaceChoice::Incoming:
        aim.face_switch_reason = "hysteresis_hold_incoming";
        break;
      case FaceChoice::Fallback:
        aim.face_switch_reason = "hysteresis_hold_fallback";
        break;
      case FaceChoice::None:
        aim.face_switch_reason = "hysteresis_hold";
        break;
    }
  } else {
    switch (best_choice) {
      case FaceChoice::FireValid:
        aim.face_switch_reason = "fire_valid";
        break;
      case FaceChoice::Incoming:
        aim.face_switch_reason = "incoming_lookahead";
        break;
      case FaceChoice::Fallback:
        aim.face_switch_reason = fallback_reason;
        break;
      case FaceChoice::None:
        aim.face_switch_reason = "none";
        break;
    }
  }

  // Clamp only the relative error used by fire gating/debug. Do not clamp the
  // absolute destination; the microcontroller/gimbal limits should handle that.
  double max_rad = 30.0 * M_PI / 180.0;
  aim.rel_yaw   = std::clamp(aim.rel_yaw, -max_rad, max_rad);
  aim.rel_pitch = std::clamp(aim.rel_pitch, -max_rad, max_rad);

  // Facing gate. The strict margin (plate within the facing window at impact) is
  // a SPINNER timing gate. For a non-spinning target the plate sits at a fixed
  // angle and the gimbal is locked on its center (Stage B downstream), so fire
  // on lock and only reject a genuinely edge-on plate beyond static_facing_max.
  // This removes the static fire/no-fire flicker that occurred when a slightly
  // canted plate sat right on the window edge while perfectly aimed. WORKLOG §4.
  const bool spinning = std::abs(x_(7)) >= cfg_.face_lookahead_min_vyaw;
  const bool facing_ok = spinning ?
    (best.margin >= 0.0) :
    (std::abs(best.phase_error) <= cfg_.static_facing_max);
  aim.fire = best.valid &&
    fireStatePermits() &&
    best.range >= cfg_.min_fire_dist &&
    best.range <= cfg_.max_fire_dist &&
    facing_ok;

  return aim;
}

}  // namespace autoaim
