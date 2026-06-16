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

#include "auto_aim_3/tracker.hpp"
#include <angles/angles.h>
#include <algorithm>
#include <cmath>

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

  // Process noise Q (continuous white-noise acceleration model)
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

// =========================================================================
// EKF UPDATE — fuse measurement z = [xa, ya, za, yaw].
//
// Observation model h(x):
//   xa = xc - r * cos(yaw)     (armor is offset from center by radius)
//   ya = yc - r * sin(yaw)
//   za = za                    (pass-through)
//   yaw = yaw                  (pass-through)
//
// Measurement noise R is DYNAMIC:
//   - Position noise grows with range (PnP less accurate far away)
//   - Yaw noise explodes at oblique angles (edge-on armor = bad PnP)
// =========================================================================
Eigen::VectorXd Tracker::ekfUpdate(const Eigen::Vector4d & z)
{
  double yaw = x_(6), r = x_(8);

  // H: Jacobian of h(x)
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(4, 9);
  H(0,0) = 1;  H(0,6) =  r*sin(yaw);  H(0,8) = -cos(yaw);
  H(1,2) = 1;  H(1,6) = -r*cos(yaw);  H(1,8) = -sin(yaw);
  H(2,4) = 1;
  H(3,6) = 1;

  // R: range-dependent + obliquity-dependent noise.
  // Range and bearing must be relative to the camera/robot, not the world origin,
  // otherwise once ego-motion is active the noise scaling and obliquity decision
  // are based on the wrong geometry.
  double zdx = z(0) - ego_x_;
  double zdy = z(1) - ego_y_;
  double zdz = z(2) - cfg_.gimbal_height;
  double dist = std::sqrt(zdx*zdx + zdy*zdy + zdz*zdz);
  double ps = cfg_.r_pos_base + cfg_.r_pos_slope * dist;
  double ys = cfg_.r_yaw_base + cfg_.r_yaw_slope * dist;
  double xyz_f = 1.0, yaw_f = 1.0;
  if (zdx*zdx + zdy*zdy > 0.01) {
    double bearing = std::atan2(zdy, zdx);
    double face_a = std::abs(angles::shortest_angular_distance(z(3), bearing));
    double cf = std::cos(face_a);
    xyz_f = 1.0 / std::max(cf*cf, 0.04);
    yaw_f = 1.0 / std::max(std::pow(std::abs(cf), 4.0), 1e-4);
    if (face_a > cfg_.max_oblique_deg * M_PI / 180.0)
      yaw_f = 1e6;  // ignore yaw when armor is edge-on
  }
  Eigen::Matrix4d R = Eigen::Matrix4d::Zero();
  R(0,0) = R(1,1) = R(2,2) = ps*ps*xyz_f;
  R(3,3) = ys*ys*yaw_f;

  // Innovation
  Eigen::Vector4d z_pred;
  z_pred << x_(0) - r*cos(yaw), x_(2) - r*sin(yaw), x_(4), yaw;
  Eigen::Vector4d y = z - z_pred;

  // Kalman gain via LDLT (numerically stable)
  Eigen::Matrix4d S = H * P_ * H.transpose() + R;
  Eigen::LDLT<Eigen::Matrix4d> S_ldlt(S);
  if (S_ldlt.info() != Eigen::Success || !S_ldlt.isPositive()) return x_;

  Eigen::MatrixXd K = S_ldlt.solve(H * P_.transpose()).transpose();
  x_ = x_ + K * y;

  // Joseph form for P (guarantees positive semi-definite)
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
  // Range used for noise scaling must be camera-relative, same as ekfUpdate.
  double zdx = z(0) - ego_x_;
  double zdy = z(1) - ego_y_;
  double zdz = z(2) - cfg_.gimbal_height;
  double d = std::sqrt(zdx*zdx + zdy*zdy + zdz*zdz);
  double ps = cfg_.r_pos_base + cfg_.r_pos_slope*d;
  double ys = cfg_.r_yaw_base + cfg_.r_yaw_slope*d;
  // CRITICAL FIX: the gate must use the SAME obliquity-inflated R as ekfUpdate.
  // Without the inflation, a slightly oblique plate (very common: a stationary
  // enemy rarely faces you perfectly) has its noisy yaw measurement judged
  // against an unrealistically small variance -> Mahalanobis blows past the
  // threshold -> detection rejected -> TRACKING flickers to TEMP_LOST ->
  // aim.fire (which requires TRACKING) stutters on a perfectly visible target.
  // (Raising maha_threshold to 16.9 only papered over this; the gate noise
  // model itself was inconsistent with the update.)
  double xyz_f = 1.0, yaw_f = 1.0;
  if (zdx*zdx + zdy*zdy > 0.01) {
    double bearing = std::atan2(zdy, zdx);
    double face_a = std::abs(angles::shortest_angular_distance(z(3), bearing));
    double cf = std::cos(face_a);
    xyz_f = 1.0 / std::max(cf*cf, 0.04);
    yaw_f = 1.0 / std::max(std::pow(std::abs(cf), 4.0), 1e-4);
    if (face_a > cfg_.max_oblique_deg * M_PI / 180.0)
      yaw_f = 1e6;
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

void Tracker::initFromDetection(const ArmorDetection & det)
{
  last_yaw_ = 0;
  double yaw = unwrapYaw(det.yaw);
  double r = cfg_.initial_radius;
  radius_ = r;  other_radius_ = r;
  // Seed dz_ from the known physical step between armor pairs.
  // If initial_dz is 0 (default) the behavior is unchanged.
  // If set to e.g. 0.05, faces 1 and 3 immediately get +5cm z offset
  // so the aim is correct from the first frame even before a face jump.
  if (cfg_.initial_dz != 0.0) {
    dz_ = cfg_.initial_dz;
    dz_initialized_ = true;
  } else {
    dz_ = 0;  dz_initialized_ = false;
  }
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

  // ── Same class. Decide identity by spatial proximity. ──
  // Predict where the current target's visible face is right now and compare
  // to the candidate's measured position.
  Eigen::Vector3d pred = armorFromState(x_);
  double dx = candidate.x - pred(0);
  double dy = candidate.y - pred(1);
  double dz = candidate.z - pred(2);
  double spatial_dist = std::sqrt(dx*dx + dy*dy + dz*dz);

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
// ARMOR JUMP — robot rotated, a different face is now visible.
// Snap yaw, swap radii if pair switch, check for divergence.
// =========================================================================
void Tracker::handleArmorJump(const ArmorDetection & det, const rclcpp::Time & now)
{
  double yaw = unwrapYaw(det.yaw);
  double jump = angles::shortest_angular_distance(x_(6), yaw);
  double jump_dir = (jump > 0) ? 1.0 : -1.0;
  const double jump_abs = std::abs(jump);
  // The timing formula below assumes this is a ONE-face (~90°) jump. A ~180°
  // wrap (a face was skipped: occlusion, missed frames) would make
  // (π/2)/dt_jump report HALF the true spin rate and corrupt x_(7) with an
  // 80-100% blend. Only feed the timing estimator with 90°-ish jumps.
  const bool one_face_jump = (jump_abs > M_PI/4 && jump_abs < 3*M_PI/4);

  // ── Spin rate from timing ────────────────────────────────────────────────
  // The most reliable way to estimate vyaw is from the time between consecutive
  // 90° face jumps. At 300 RPM a 90° jump takes exactly π/(2*vyaw) = 50ms.
  // This converges in 2 jumps (~100ms) vs 10-20 EKF frames for normal estimation.
  // Only activate after 2 same-direction jumps to avoid noise and spin reversals.
  if (cfg_.use_vyaw_from_timing && last_jump_time_valid_ && one_face_jump) {
    double dt_jump = (now - last_jump_time_).seconds();
    bool same_dir = (jump_dir * last_jump_dir_ > 0);

    if (same_dir && dt_jump >= cfg_.vyaw_timing_min_dt &&
        dt_jump <= cfg_.vyaw_timing_max_dt)
    {
      consecutive_same_dir_jumps_++;
      if (consecutive_same_dir_jumps_ >= 1) {
        // Direct estimate: each face is π/2 apart, so vyaw = (π/2) / dt
        double vyaw_est = (M_PI / 2.0) / dt_jump * jump_dir;
        // Blend with current EKF estimate: trust timing heavily (80%) after
        // the first confirmation, fully (100%) after the second.
        double blend = (consecutive_same_dir_jumps_ >= 2) ? 1.0 : 0.80;
        x_(7) = blend * vyaw_est + (1.0 - blend) * x_(7);
        // Tighten vyaw covariance — we have a direct measurement now
        P_(7,7) = std::min(P_(7,7), 1.0);
      }
    } else if (!same_dir) {
      // Direction reversal: robot changed spin direction or this is noise.
      // Reset the counter but keep the last jump time.
      consecutive_same_dir_jumps_ = 0;
    }
  }
  last_jump_time_ = now;
  last_jump_time_valid_ = true;
  last_jump_dir_ = jump_dir;

  // Spin reversal: if jump opposes estimated spin and timing didn't just
  // correct it, zero vyaw to avoid runaway prediction.
  if (std::abs(x_(7)) > 0.2 && jump * x_(7) < 0 &&
      consecutive_same_dir_jumps_ == 0) {
    x_(7) = 0;
  }
  x_(6) = yaw;

  // 90° jump = pair switch (different radius and height)
  double ja = std::abs(jump);
  if (ja > M_PI/4 && ja < 3*M_PI/4) {
    // Height step between pairs.
    // The sign flip (dz_ = -dz_) was unreliable: x_(4) at jump time is the
    // z of the face we just LEFT, not the true center, so new_dz was measured
    // from the wrong reference. Over multiple jumps the sign flip + noisy EMA
    // would corrupt dz_ away from the physical value within 2-3 seconds.
    //
    // New approach:
    //   1. Compute the raw height step from this jump: step = x_(4) - det.z
    //   2. Use the ABSOLUTE value for the EMA — the sign is handled separately
    //      by tracking which face index is "high" vs "low".
    //   3. After the EMA update, restore the sign based on whether we jumped
    //      UP (new face is higher → dz_ positive) or DOWN (→ negative).
    //   4. If initial_dz was seeded, protect it: only update if the new
    //      measurement agrees in sign with the seed.
    double raw_step = x_(4) - det.z;  // positive = we jumped DOWN to a lower face
    double abs_step = std::abs(raw_step);

    if (!dz_initialized_) {
      // First jump: take the measurement directly
      dz_ = raw_step;
      dz_initialized_ = true;
    } else {
      // Subsequent jumps: EMA on absolute magnitude, preserve sign from measurement
      // Only update if the new measurement is plausible (within 3x of current value)
      // to reject outliers from noisy PnP at oblique angles.
      double abs_dz = std::abs(dz_);
      if (abs_step < std::max(abs_dz * 3.0, 0.03) && abs_step > 0.005) {
        double new_abs = 0.10 * abs_step + 0.90 * abs_dz;
        // Sign: if raw_step and dz_ agree in sign, keep. If they disagree,
        // only flip if the disagreement is consistent (abs_step > 0.5*abs_dz).
        // This prevents single noisy measurements from flipping the sign.
        if (raw_step * dz_ > 0) {
          dz_ = new_abs * (dz_ > 0 ? 1.0 : -1.0);
        } else if (abs_step > abs_dz * 0.5) {
          // Consistent sign disagreement — the initial seed may have been wrong
          dz_ = new_abs * (raw_step > 0 ? 1.0 : -1.0);
        }
        // else: small disagreement, keep current sign
      }
    }
    x_(4) = det.z;  x_(5) = 0;
    std::swap(radius_, other_radius_);
    x_(8) = radius_;
  }

  // Divergence check: if inferred armor is far from detection, hard reset
  auto inferred = armorFromState(x_);
  Eigen::Vector3d detected(det.x, det.y, det.z);
  if ((inferred - detected).norm() > cfg_.max_match_dist) {
    double r = x_(8);
    x_ << det.x+r*cos(yaw), 0, det.y+r*sin(yaw), 0, det.z, 0, yaw, 0, r;
    P_ = P0_;
    radius_ = cfg_.initial_radius;  other_radius_ = cfg_.initial_radius;
    // Preserve dz_ if it was seeded from initial_dz or learned — resetting
    // to zero on every divergence was causing random high/low misses during
    // spinning because the height step was re-learned from scratch each time.
    if (!dz_initialized_ || std::abs(dz_) < 0.005) {
      dz_ = cfg_.initial_dz;
      dz_initialized_ = (cfg_.initial_dz != 0.0);
    }
    // Reset timing estimator — divergence means the phase is unknown
    last_jump_time_valid_ = false;
    consecutive_same_dir_jumps_ = 0;
  } else {
    // Inflate yaw/vyaw covariance (we bypassed the EKF)
    double s = 2.0;
    P_.row(6)*=s; P_.col(6)*=s;  P_.row(7)*=s; P_.col(7)*=s;
    P_ = (P_+P_.transpose())*0.5;
  }

  // EKF update with jump measurement
  Eigen::Vector4d z(det.x, det.y, det.z, yaw);
  x_ = ekfUpdate(z);

  // Update radius via EMA. Bearing is from ROBOT to armor (not from world origin),
  // otherwise the obliquity check is wrong once ego-motion is active.
  double bearing = std::atan2(det.y - ego_y_, det.x - ego_x_);
  double fa = std::abs(angles::shortest_angular_distance(x_(6), bearing));
  if (fa < cfg_.max_oblique_deg * M_PI/180.0) {
    double rm = (x_(0)-det.x)*cos(x_(6)) + (x_(2)-det.y)*sin(x_(6));
    if (rm > 0.10 && rm < 0.45)
      radius_ = (1-cfg_.radius_ema_alpha)*radius_ + cfg_.radius_ema_alpha*rm;
  }
  x_(8) = radius_;
}

// =========================================================================
// UPDATE — main per-frame function.
// =========================================================================
void Tracker::update(const std::vector<ArmorDetection> & detections, double dt,
                     const rclcpp::Time & now)
{
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
    initFromDetection(*best);
    state_ = DETECTING;
    return;
  }

  lost_thresh_ = std::max(1, (int)(cfg_.lost_timeout / std::max(dt, 0.01)));

  // ── PREDICT ──
  ekfPredict(dt);

  // ── ASSOCIATE: find best matching detection (Mahalanobis gating) ──
  auto pred_armor = armorFromState(x_);
  bool matched = false;
  ArmorDetection best_det{};
  double best_maha = cfg_.maha_threshold;

  for (const auto & det : detections) {
    if (det.class_id != target_id_) continue;
    double pred_yaw = x_(6);
    double uw = pred_yaw + angles::shortest_angular_distance(pred_yaw, det.yaw);
    Eigen::Vector4d z(det.x, det.y, det.z, uw);
    double m = ekfMahalanobis(z);
    if (m < best_maha) { best_maha = m; best_det = det; matched = true; }
  }

  // ── UPDATE or COAST ──
  if (matched) {
    double meas_yaw = unwrapYaw(best_det.yaw);
    double yd = std::abs(angles::shortest_angular_distance(x_(6), meas_yaw));
    Eigen::Vector3d dp(best_det.x, best_det.y, best_det.z);
    double pd = (pred_armor - dp).norm();

    // Obliquity check: bearing from robot (not from origin) to armor.
    double bearing = std::atan2(best_det.y - ego_y_, best_det.x - ego_x_);
    double fa = std::abs(angles::shortest_angular_distance(meas_yaw, bearing));
    bool oblique = fa > cfg_.max_oblique_deg * M_PI/180.0;

    if (pd < cfg_.max_match_dist && (yd < cfg_.yaw_jump_thresh || oblique)) {
      // MATCH — normal EKF update
      Eigen::Vector4d z(best_det.x, best_det.y, best_det.z, meas_yaw);
      x_ = ekfUpdate(z);
      // Radius EMA (skip when oblique)
      if (!oblique) {
        double rm = (x_(0)-best_det.x)*cos(x_(6)) + (x_(2)-best_det.y)*sin(x_(6));
        if (rm > 0.10 && rm < 0.45)
          radius_ = (1-cfg_.radius_ema_alpha)*radius_ + cfg_.radius_ema_alpha*rm;
        x_(8) = radius_;
      }
    } else if (yd > cfg_.yaw_jump_thresh && !oblique) {
      // JUMP — face switch
      handleArmorJump(best_det, now);
    } else {
      matched = false;  // too far — treat as miss
    }
  }

  // ── SAFETY CLAMPS ──
  x_(8) = std::clamp(x_(8), 0.12, 0.40);
  radius_ = x_(8);

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
      }
    }
  } else if (state_ == TRACKING) {
    if (!matched) { state_ = TEMP_LOST; lost_count_ = 1; }
  } else if (state_ == TEMP_LOST) {
    if (matched) { state_ = TRACKING; lost_count_ = 0; }
    else if (++lost_count_ > lost_thresh_) { state_ = LOST; }
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
//   4. Pick face with best fire_window_margin
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

  // Single-face vs four-face mode.
  // When |vyaw| is below vyaw_fire_threshold the spin rate is unreliable —
  // either the robot is stationary, rotating slowly, or just starting to spin.
  // In this case only aim at the currently visible face (fi=0). Predicting
  // phantom faces with wrong vyaw causes random aim jumps.
  // When spinning fast (above threshold) use all four faces so the EKF can
  // pick the best face at impact time.
  const int faces_to_check = (std::abs(x_(7)) >= cfg_.vyaw_fire_threshold) ? FACES : 1;

  struct Face {
    double x = 0, y = 0, z = 0, range = 0, bearing = 0;
    double abs_pitch = 0, flight_time = 0, margin = -1e9, vis = 0;
    bool valid = false;
  };
  Face best;
  bool found = false;

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

    for (int fi = 0; fi < faces_to_check; fi++) {
      double fy = x_(6) + x_(7)*pred_t + fi*(2*M_PI/FACES);
      bool alt = (fi % 2 == 1);
      double r = alt ? other_radius_ : radius_;
      double dzo = alt ? dz_ : 0.0;

      // Predicted center at impact (constant velocity model)
      double cx = x_(0) + x_(1)*pred_t;
      double cy = x_(2) + x_(3)*pred_t;
      double cz = x_(4) + x_(5)*pred_t;

      // Face position in odom
      double fx = cx - r*std::cos(fy);
      double fpy = cy - r*std::sin(fy);
      double fz = cz + dzo;

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

      // Fire window margin: positive = face is aligned with bearing
      double ds = std::min(cfg_.window_ref_dist / std::max(range, 0.5), 2.0);
      double win = cfg_.angular_window * ds;
      double err = std::abs(angles::shortest_angular_distance(bearing, fy));
      double margin = win - err;

      Face c;
      c.x = fx; c.y = fpy; c.z = fz;
      c.range = range; c.bearing = bearing;
      c.abs_pitch = bal.pitch; c.flight_time = bal.flight_time;
      c.margin = margin; c.vis = vis; c.valid = true;

      if (!found || margin > best.margin ||
          (std::abs(margin - best.margin) < 0.01 && vis > best.vis)) {
        best = c; found = true;
      }
    }
    if (found) pred_t = best.flight_time + predictionBias();
  }

  if (!found) return aim;

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

  // Clamp only the relative error used by fire gating/debug. Do not clamp the
  // absolute destination; the microcontroller/gimbal limits should handle that.
  double max_rad = 30.0 * M_PI / 180.0;
  aim.rel_yaw   = std::clamp(aim.rel_yaw, -max_rad, max_rad);
  aim.rel_pitch = std::clamp(aim.rel_pitch, -max_rad, max_rad);

  aim.fire = best.valid &&
    state_ == TRACKING &&
    best.range >= cfg_.min_fire_dist &&
    best.range <= cfg_.max_fire_dist &&
    best.margin >= 0.0;

  return aim;
}

}  // namespace autoaim
