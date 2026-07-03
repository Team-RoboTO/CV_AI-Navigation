#include "autoaim_v2/aimer.hpp"

#include <algorithm>
#include <cmath>

#include "autoaim_v2/gimbal_buffer.hpp"  // norm_angle / ang_diff

namespace aim
{
namespace
{
double erf_p(double margin, double sigma)
{
  if (sigma < 1e-9) return margin > 0 ? 1.0 : 0.0;
  return std::erf(margin / (sigma * std::sqrt(2.0)));
}
}  // namespace

Aimer::FaceAim Aimer::aim_face(const Eigen::VectorXd & xs, int face,
                               const Eigen::Vector3d & barrel) const
{
  FaceAim f;
  f.face = face;
  f.pos = Tracker::armor_pos(xs, face);

  const Eigen::Vector3d d = f.pos - barrel;
  f.gd = std::hypot(d.x(), d.y());
  f.dz = d.z();
  f.range = d.norm();
  f.bearing = std::atan2(d.y(), d.x());
  f.obliquity = std::fabs(ang_diff(Tracker::armor_theta(xs, face), f.bearing));
  f.bal = solve_ballistics(p_.ballistics, f.gd, f.dz);
  f.valid = f.bal.valid;
  return f;
}

double Aimer::p_hit_static(const FaceAim & f) const
{
  // Angular dispersion -> meters at the target.
  const double disp = std::hypot(p_.disp_bullet_mrad, p_.disp_system_mrad) * 1e-3;
  const double sigma = disp * f.range;

  const double half_w = 0.5 * p_.plate_w * std::max(std::cos(f.obliquity), 0.15);
  const double half_h =
    0.5 * p_.plate_h * std::cos(p_.armor_pitch_deg * M_PI / 180.0);

  return erf_p(half_w, sigma) * erf_p(half_h, sigma);
}

GimbalCommand Aimer::plan(const Tracker & trk, TimePoint t_now,
                          const GimbalSample & gs_now,
                          const Eigen::Vector2d & ego,
                          const Eigen::Vector2d & ego_vel, AimDebug & dbg)
{
  GimbalCommand cmd;
  dbg = AimDebug{};
  if (!trk.tracking()) return cmd;

  const double pipeline_age = std::clamp(seconds(t_now, trk.stamp()), 0.0, 0.25);
  const double omega = trk.x()(7);
  dbg.omega = omega;

  // Barrel position now (yaw-rotated offset), and its future via ego velocity.
  const double cy = std::cos(gs_now.yaw), sy = std::sin(gs_now.yaw);
  const Eigen::Vector3d barrel_now(
    ego.x() + p_.barrel_x * cy - p_.barrel_y * sy,
    ego.y() + p_.barrel_x * sy + p_.barrel_y * cy,
    p_.gimbal_height + p_.barrel_z);

  // ── Select regime ──
  FireRegime regime;
  const double aomega = std::fabs(omega);
  if (aomega < p_.omega_track) regime = FireRegime::TRACK;
  else if (aomega < p_.omega_timed) regime = FireRegime::SWEEP;
  else regime = FireRegime::TIMED;
  dbg.regime = regime;

  // ── Iterated impact-time prediction ──
  // TRACK/SWEEP: arrival = now + actuation + flight (gimbal must move there).
  // TIMED: arrival = a chosen plate crossing; the crossing is selected FIRST
  // (earliest one reachable given feeder + flight), then we aim at THAT
  // face's crossing point — heights alternate between pairs, so aiming the
  // "next geometric" face while firing at a later crossing puts the pitch on
  // the wrong pair.
  const double t_state_age = seconds(t_now, trk.stamp());
  double horizon = pipeline_age + p_.actuation_latency + 0.12;
  double flight_est = 0.12;
  double cross_time = 0;  // TIMED: selected crossing, seconds after stamp()
  int cross_face = 0;
  FaceAim sel;
  Eigen::VectorXd xs;

  for (int iter = 0; iter < 3; iter++) {
    if (regime == FireRegime::TIMED) {
      // Earliest achievable arrival if the shot were commanded right now.
      const double earliest = t_state_age + p_.feeder_delay + flight_est;
      const double period = 2 * M_PI / std::max(std::fabs(omega), 1e-3);
      const Eigen::VectorXd & x0 = trk.x();

      // Center bearing barely moves over one crossing period — use the
      // center predicted at `earliest` for the LOS.
      Eigen::VectorXd xc = trk.predict_state(earliest);
      const Eigen::Vector3d barrel =
        barrel_now + earliest * Eigen::Vector3d(ego_vel.x(), ego_vel.y(), 0.0);
      const double beta =
        std::atan2(xc(2) - barrel.y(), xc(0) - barrel.x());

      cross_time = 1e18;
      for (int i = 0; i < 4; i++) {
        const double phase = ang_diff(beta, Tracker::armor_theta(x0, i));
        double dt_cross = phase / omega;  // face i angle reaches beta
        while (dt_cross < earliest) dt_cross += period;
        if (dt_cross < cross_time) {
          cross_time = dt_cross;
          cross_face = i;
        }
      }

      // Aim at cross_face's ring point at the crossing time.
      xs = trk.predict_state(cross_time);
      xs(6) = norm_angle(beta - cross_face * M_PI / 2);  // face at LOS
      const Eigen::Vector3d barrel_c =
        barrel_now + cross_time * Eigen::Vector3d(ego_vel.x(), ego_vel.y(), 0.0);
      sel = aim_face(xs, cross_face, barrel_c);
      sel.obliquity = 0.0;  // frontal by construction at crossing
      if (!sel.valid) return cmd;
      flight_est = sel.bal.flight_time;
      horizon = cross_time;
    } else {
      // TRACK / SWEEP: evaluate the 4 predicted faces.
      xs = trk.predict_state(horizon);
      const Eigen::Vector3d barrel =
        barrel_now + horizon * Eigen::Vector3d(ego_vel.x(), ego_vel.y(), 0.0);
      FaceAim best;
      double best_score = -1e18;
      for (int i = 0; i < 4; i++) {
        FaceAim f = aim_face(xs, i, barrel);
        if (!f.valid) continue;
        const double coming = p_.coming_angle_deg * M_PI / 180.0;
        if (f.obliquity > coming) continue;  // effectively invisible at impact

        double score = -f.obliquity;
        if (regime == FireRegime::SWEEP) {
          // Prefer the plate ENTERING the window (sp_vision logic): its
          // signed angle relative to the LOS moves toward zero.
          const double signed_d =
            ang_diff(Tracker::armor_theta(xs, i), f.bearing);
          const bool leaving = (omega > 0) ? (signed_d > p_.leaving_angle_deg * M_PI / 180.0)
                                           : (signed_d < -p_.leaving_angle_deg * M_PI / 180.0);
          if (leaving) score -= 10.0;  // strongly deprioritize leaving plates
        }
        if (score > best_score) {
          best_score = score;
          best = f;
        }
      }
      sel = best;
    }

    if (!sel.valid) return cmd;
    if (regime != FireRegime::TIMED) {
      horizon = pipeline_age + p_.actuation_latency + sel.bal.flight_time;
    }
  }

  dbg.face = sel.face;
  dbg.dist = sel.range;
  dbg.flight_time = sel.bal.flight_time;
  dbg.horizon = horizon;
  dbg.aim_point = sel.pos;

  // ── Command angles (internal convention) ──
  const double yaw_cmd_internal =
    norm_angle(sel.bearing - p_.yaw_offset_deg * M_PI / 180.0);
  const double pitch_cmd_internal =
    sel.bal.pitch - p_.pitch_offset_deg * M_PI / 180.0;

  // Convert to the micro frame: micro yaw is unbounded — send raw + shortest
  // delta (identical to the old pipeline, which the firmware expects).
  const double yaw_delta = ang_diff(yaw_cmd_internal, gs_now.yaw);
  cmd.yaw_micro = gs_now.yaw_raw + p_.yaw_sign * yaw_delta;
  cmd.pitch_micro = p_.pitch_sign * pitch_cmd_internal;
  cmd.valid = true;

  // Feedforward: analytic bearing rate of the car center (world frame).
  {
    const Eigen::VectorXd & x = trk.x();
    const double rx = x(0) - barrel_now.x(), ry = x(2) - barrel_now.y();
    const double rho2 = std::max(rx * rx + ry * ry, 1e-6);
    const double rvx = x(1) - ego_vel.x(), rvy = x(3) - ego_vel.y();
    cmd.ff_yaw_rate = p_.yaw_sign * (rx * rvy - ry * rvx) / rho2;
    cmd.ff_pitch_rate = 0.0;
  }

  // ── Fire lock: is the gimbal physically where we command? ──
  const double micro_pitch_for_lock =
    p_.micro_pitch_lock_opposite_sign ? -gs_now.pitch_raw : gs_now.pitch_raw;
  dbg.lock_err_yaw = ang_diff(cmd.yaw_micro, gs_now.yaw_raw);
  dbg.lock_err_pitch = cmd.pitch_micro - micro_pitch_for_lock;
  dbg.locked = std::fabs(dbg.lock_err_yaw) <= p_.fire_lock_yaw &&
               std::fabs(dbg.lock_err_pitch) <= p_.fire_lock_pitch;

  const bool range_ok =
    sel.range >= p_.min_fire_dist && sel.range <= p_.max_fire_dist;
  const bool can_fire = trk.state() == Tracker::TRACKING && range_ok && dbg.locked;

  // ── Fire policy ──
  if (regime == FireRegime::TIMED) {
    // Schedule the shot so the pellet ARRIVES exactly on the selected plate
    // crossing:  t_fire = t_cross - flight - feeder_delay.
    // Arrival-time sigma: feeder + serial + bullet speed + spin phase.
    const double sig_v = sel.bal.flight_time * p_.bullet_speed_sigma_frac;
    const double sig_phase =
      std::sqrt(std::max(trk.P()(6, 6) + trk.P()(7, 7) * horizon * horizon, 0.0)) /
      std::max(std::fabs(omega), 1e-3);
    const double sig_arr = std::sqrt(
      p_.feeder_sigma * p_.feeder_sigma + sig_v * sig_v +
      p_.serial_quant_sigma * p_.serial_quant_sigma + sig_phase * sig_phase);

    const double half_window =
      (0.5 * p_.plate_w / std::max(trk.x()(8), 0.05)) / std::fabs(omega);
    const double p_time = erf_p(half_window, sig_arr);
    // Vertical still needs to be right.
    const double disp = std::hypot(p_.disp_bullet_mrad, p_.disp_system_mrad) * 1e-3;
    const double p_v = erf_p(0.5 * p_.plate_h *
                             std::cos(p_.armor_pitch_deg * M_PI / 180.0),
                             disp * sel.range);
    dbg.p_hit = p_time * p_v;

    cmd.shoot = false;
    if (can_fire && dbg.p_hit >= p_.p_timed_min) {
      // cross_time was selected in the loop with earliest = feeder + flight,
      // so t_fire_rel >= ~0 by construction.
      const double t_fire_rel =
        cross_time - sel.bal.flight_time - p_.feeder_delay - t_state_age;
      dbg.next_shot_in = t_fire_rel;

      const bool interval_ok =
        !last_shot_valid_ ||
        seconds(t_now, last_shot_) + t_fire_rel >= p_.shot_min_interval;
      // Schedule when the fire moment lands before the next frame (~8.3 ms);
      // the sender precision-sleeps to it. Slightly-late moments (estimator
      // just updated) still fire immediately within half a pulse.
      if (interval_ok && t_fire_rel <= 0.009 && t_fire_rel > -0.5 * p_.shot_pulse) {
        cmd.shoot_scheduled = true;
        cmd.shoot = t_fire_rel <= 0.0;  // already due -> immediate
        cmd.shoot_at =
          t_now + std::chrono::microseconds(
                    static_cast<int64_t>(std::max(t_fire_rel, 0.0) * 1e6));
        last_shot_ = cmd.shoot_at;
        last_shot_valid_ = true;
      }
    }
  } else {
    dbg.p_hit = p_hit_static(sel);
    cmd.shoot = can_fire && dbg.p_hit >= p_.p_spray_min;
  }

  return cmd;
}

}  // namespace aim
