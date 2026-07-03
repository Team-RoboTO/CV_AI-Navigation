#include "autoaim_v2/tracker.hpp"

#include <algorithm>
#include <cmath>

#include "autoaim_v2/gimbal_buffer.hpp"  // norm_angle / ang_diff

namespace aim
{

Tracker::Tracker(const TrackerParams & p) : p_(p)
{
  x_ = Eigen::VectorXd::Zero(11);
  Eigen::VectorXd d(11);
  //    xc    vx   yc    vy   zc    vz  theta omega    r      l      h
  d << 0.05, 1.0, 0.05, 1.0, 0.05, 0.25, 0.09, 64.0, 0.0064, 0.0025, 0.0025;
  P0_ = d.asDiagonal();
  P_ = P0_;
}

Eigen::Vector3d Tracker::armor_pos(const Eigen::VectorXd & x, int i)
{
  const double phi = x(6) + i * M_PI / 2;
  const double r = x(8) + ((i % 2) ? x(9) : 0.0);
  const double z = x(4) + ((i % 2) ? x(10) : 0.0);
  return {x(0) - r * std::cos(phi), x(2) - r * std::sin(phi), z};
}

double Tracker::armor_theta(const Eigen::VectorXd & x, int i)
{
  return norm_angle(x(6) + i * M_PI / 2);
}

Eigen::VectorXd Tracker::predict_state(double dt) const
{
  Eigen::VectorXd x = x_;
  x(0) += x(1) * dt;
  x(2) += x(3) * dt;
  x(4) += x(5) * dt;
  x(6) = norm_angle(x(6) + x(7) * dt);
  return x;
}

void Tracker::ekf_predict(double dt)
{
  dt = std::clamp(dt, 1e-4, 0.2);

  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(11, 11);
  F(0, 1) = dt;
  F(2, 3) = dt;
  F(4, 5) = dt;
  F(6, 7) = dt;

  const double a = dt * dt * dt * dt / 4, b = dt * dt * dt / 2, c = dt * dt;
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(11, 11);
  auto blk = [&](int i, double v) {
    Q(i, i) = a * v;
    Q(i, i + 1) = Q(i + 1, i) = b * v;
    Q(i + 1, i + 1) = c * v;
  };
  blk(0, p_.q_accel_xy);
  blk(2, p_.q_accel_xy);
  blk(4, p_.q_accel_z);
  blk(6, p_.q_alpha);
  Q(8, 8) = p_.q_r * dt;
  Q(9, 9) = p_.q_l * dt;
  Q(10, 10) = p_.q_h * dt;

  x_ = F * x_;
  x_(6) = norm_angle(x_(6));

  if (state_ == TEMP_LOST) {
    // Coast: damp translation velocity (we cannot see it change) but keep
    // omega — a spinner that ducked behind a wall is still spinning.
    const double keep = std::pow(p_.coast_vel_damping, dt);
    x_(1) *= keep;
    x_(3) *= keep;
    x_(5) *= keep;
  }

  P_ = F * P_ * F.transpose() + Q;
  P_ = 0.5 * (P_ + P_.transpose());
}

Eigen::Vector4d Tracker::h_of(const Eigen::VectorXd & x, int face,
                              const Eigen::Vector3d & cam0) const
{
  const Eigen::Vector3d rel = armor_pos(x, face) - cam0;
  const double rho = std::hypot(rel.x(), rel.y());
  return {std::atan2(rel.y(), rel.x()),
          std::atan2(rel.z(), rho),
          rel.norm(),
          norm_angle(x(6) + face * M_PI / 2)};
}

Eigen::Matrix<double, 4, 11> Tracker::H_of(const Eigen::VectorXd & x, int face,
                                           const Eigen::Vector3d & cam0) const
{
  // Chain rule: d[ypd,theta]/dx = J_ypd(rel) * d(armor_xyz)/dx  (+ theta row).
  const double phi = x(6) + face * M_PI / 2;
  const double r = x(8) + ((face % 2) ? x(9) : 0.0);
  const double s = std::sin(phi), c = std::cos(phi);

  Eigen::Matrix<double, 3, 11> Jxyz = Eigen::Matrix<double, 3, 11>::Zero();
  Jxyz(0, 0) = 1;
  Jxyz(1, 2) = 1;
  Jxyz(2, 4) = 1;
  Jxyz(0, 6) = r * s;
  Jxyz(1, 6) = -r * c;
  Jxyz(0, 8) = -c;
  Jxyz(1, 8) = -s;
  if (face % 2) {
    Jxyz(0, 9) = -c;
    Jxyz(1, 9) = -s;
    Jxyz(2, 10) = 1;
  }

  const Eigen::Vector3d rel = armor_pos(x, face) - cam0;
  const double X = rel.x(), Y = rel.y(), Z = rel.z();
  const double rho2 = X * X + Y * Y;
  const double rho = std::sqrt(std::max(rho2, 1e-9));
  const double d2 = rho2 + Z * Z;
  const double d = std::sqrt(std::max(d2, 1e-9));

  Eigen::Matrix3d Jypd;
  Jypd << -Y / rho2, X / rho2, 0,
          -Z * X / (rho * d2), -Z * Y / (rho * d2), rho / d2,
          X / d, Y / d, Z / d;

  Eigen::Matrix<double, 4, 11> H = Eigen::Matrix<double, 4, 11>::Zero();
  H.topRows<3>() = Jypd * Jxyz;
  H(3, 6) = 1;
  return H;
}

Eigen::Matrix4d Tracker::R_of(const ArmorWorld & a) const
{
  // Obliquity: how far the plate is from squarely facing the camera.
  const double obl = std::fabs(ang_diff(a.theta_a, a.yaw));
  const double co = std::max(std::cos(obl), 0.25);

  const double vb = p_.r_bearing * p_.r_bearing;
  double sd = p_.r_dist_base + p_.r_dist_slope * a.dist;
  const double vdist = sd * sd / (co * co);  // depth degrades oblique
  double st = std::max(a.theta_sigma, p_.r_theta_min);
  const double vth = st * st / (co * co);

  Eigen::Matrix4d R = Eigen::Matrix4d::Zero();
  R(0, 0) = vb;
  R(1, 1) = vb;
  R(2, 2) = vdist;
  R(3, 3) = vth;
  return R;
}

int Tracker::associate(const ArmorWorld & a, const Eigen::Vector3d & cam0) const
{
  // Pick the modeled face that best explains the measurement: angular
  // distance in plate angle + bearing (sp_vision-style), then verify with
  // the position gate in the caller.
  int best = -1;
  double best_cost = 1e18;
  for (int i = 0; i < 4; i++) {
    const Eigen::Vector4d zp = h_of(x_, i, cam0);
    const double cost = std::fabs(ang_diff(a.theta_a, zp(3))) +
                        std::fabs(ang_diff(a.yaw, zp(0)));
    if (cost < best_cost) {
      best_cost = cost;
      best = i;
    }
  }
  return best;
}

double Tracker::mahalanobis(const ArmorWorld & a, int face,
                            const Eigen::Vector3d & cam0) const
{
  const Eigen::Vector4d z(a.yaw, a.pitch, a.dist, a.theta_a);
  Eigen::Vector4d nu = z - h_of(x_, face, cam0);
  nu(0) = norm_angle(nu(0));
  nu(1) = norm_angle(nu(1));
  nu(3) = norm_angle(nu(3));

  const auto H = H_of(x_, face, cam0);
  Eigen::Matrix4d S = H * P_ * H.transpose() + R_of(a);
  Eigen::LDLT<Eigen::Matrix4d> ldlt(S);
  if (ldlt.info() != Eigen::Success) return 1e18;
  return nu.dot(ldlt.solve(nu));
}

bool Tracker::ekf_update(const ArmorWorld & a, const Eigen::Vector3d & cam0)
{
  const int face = associate(a, cam0);
  if (face < 0) return false;

  // Position sanity: a measurement very far from every modeled face is a
  // divergence candidate, not an update.
  const double pos_err = (a.pos_world - armor_pos(x_, face)).norm();
  if (pos_err > p_.reinit_pos_gate) return false;

  if (mahalanobis(a, face, cam0) > p_.maha_gate) {
    // RESCUE instead of reject. A position-plausible measurement failing the
    // gate means the filter is confident AND wrong about the plate angle —
    // the classic single-plate ambiguity: lateral center motion and rotation
    // about the center produce identical bearing/distance streams, and once
    // the filter picks the wrong (spinning) interpretation, gate rejections
    // lock it there forever (translation read as a phantom 200+ RPM spin).
    // Snap theta so the associated face matches the measured plate angle,
    // inflate theta/omega covariance, and proceed with the normal update.
    // A genuine face jump after occlusion lands here too — same correct
    // treatment (re-phase to the visible plate, let omega re-learn).
    const double dth = ang_diff(a.theta_a, armor_theta(x_, face));
    x_(6) = norm_angle(x_(6) + dth);
    // Cross-covariances of theta/omega no longer reflect the snapped state.
    for (int i = 0; i < 11; i++) {
      if (i != 6) P_(6, i) = P_(i, 6) = 0.0;
      if (i != 7) P_(7, i) = P_(i, 7) = 0.0;
    }
    P_(6, 6) = std::max(P_(6, 6), 0.05);
    P_(7, 7) = std::max(P_(7, 7), 25.0);
  }

  const Eigen::Vector4d z(a.yaw, a.pitch, a.dist, a.theta_a);
  Eigen::Vector4d nu = z - h_of(x_, face, cam0);
  nu(0) = norm_angle(nu(0));
  nu(1) = norm_angle(nu(1));
  nu(3) = norm_angle(nu(3));

  const auto H = H_of(x_, face, cam0);
  const Eigen::Matrix4d R = R_of(a);
  Eigen::Matrix4d S = H * P_ * H.transpose() + R;
  Eigen::LDLT<Eigen::Matrix4d> ldlt(S);
  if (ldlt.info() != Eigen::Success) return false;

  Eigen::Matrix<double, 11, 4> K = P_ * H.transpose() * ldlt.solve(Eigen::Matrix4d::Identity());
  x_ += K * nu;
  x_(6) = norm_angle(x_(6));

  Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(11, 11) - K * H;
  P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
  P_ = 0.5 * (P_ + P_.transpose());

  last_face_ = face;
  return true;
}

void Tracker::clamp_state()
{
  x_(8) = std::clamp(x_(8), p_.r_min, p_.r_max);
  // r + l must also stay physical.
  x_(9) = std::clamp(x_(9), std::max(-p_.l_abs_max, p_.r_min - x_(8)),
                     std::min(p_.l_abs_max, p_.r_max - x_(8)));
  x_(10) = std::clamp(x_(10), -p_.h_abs_max, p_.h_abs_max);
  if (std::fabs(x_(7)) > p_.omega_abs_max) {
    x_(7) = std::copysign(p_.omega_abs_max, x_(7));
    P_(7, 7) = std::max(P_(7, 7), 4.0);  // let it come back down
  }
}

void Tracker::init_from(const ArmorWorld & a, TimePoint t)
{
  x_ = Eigen::VectorXd::Zero(11);
  x_(0) = a.pos_world.x() + p_.initial_r * std::cos(a.theta_a);
  x_(2) = a.pos_world.y() + p_.initial_r * std::sin(a.theta_a);
  x_(4) = a.pos_world.z();
  x_(6) = a.theta_a;
  x_(8) = p_.initial_r;
  P_ = P0_;
  t_ = t;
  state_ = DETECTING;
  confirm_count_ = 0;
  divergent_count_ = 0;
  temp_lost_since_ = 0;
  generation_++;
  last_face_ = 0;
}

void Tracker::update(const std::vector<ArmorWorld> & armors, TimePoint t,
                     const Eigen::Vector3d & cam0)
{
  matched_count_ = 0;

  if (state_ == LOST) {
    if (armors.empty()) return;
    // 1v1: one enemy. Start on the nearest plate.
    const ArmorWorld * best = &armors.front();
    for (const auto & a : armors)
      if (a.dist < best->dist) best = &a;
    init_from(*best, t);
    return;
  }

  const double dt = seconds(t, t_);
  t_ = t;
  if (dt > 0) ekf_predict(dt);

  int updated = 0;
  int rejected_far = 0;
  if (!armors.empty()) {
    // Update with EVERY plate that matches (two plates of the same car are
    // simultaneously visible ~44% of spin phase — using both conditions the
    // center/radius much better than single-plate updates).
    // Order: nearest first so the better measurement shapes the linearization.
    std::vector<const ArmorWorld *> sorted;
    for (const auto & a : armors) sorted.push_back(&a);
    std::sort(sorted.begin(), sorted.end(),
              [](auto * l, auto * r) { return l->dist < r->dist; });
    for (const auto * a : sorted) {
      if (ekf_update(*a, cam0)) {
        updated++;
      } else {
        const int face = associate(*a, cam0);
        if (face >= 0 &&
            (a->pos_world - armor_pos(x_, face)).norm() > p_.reinit_pos_gate) {
          rejected_far++;
        }
      }
    }
  }
  matched_count_ = updated;
  clamp_state();

  // Divergence handling: consistent far-off detections mean the enemy
  // teleported relative to our belief (typical after a long occlusion —
  // they crossed behind the wall). Re-seed instead of slowly dragging.
  if (updated == 0 && rejected_far > 0) {
    if (++divergent_count_ >= p_.reinit_after) {
      const ArmorWorld * best = &armors.front();
      for (const auto & a : armors)
        if (a.dist < best->dist) best = &a;
      init_from(*best, t);
      return;
    }
  } else if (updated > 0) {
    divergent_count_ = 0;
  }

  // ── State machine ──
  switch (state_) {
    case DETECTING:
      if (updated > 0) {
        if (++confirm_count_ >= p_.confirm_frames) state_ = TRACKING;
      } else {
        if (--confirm_count_ < -3) state_ = LOST;
      }
      break;
    case TRACKING:
      if (updated == 0) {
        state_ = TEMP_LOST;
        temp_lost_since_ = 0;
      }
      break;
    case TEMP_LOST:
      if (updated > 0) {
        state_ = TRACKING;
      } else {
        temp_lost_since_ += dt;
        if (temp_lost_since_ > p_.lost_timeout) state_ = LOST;
      }
      break;
    default:
      break;
  }
}

}  // namespace aim
