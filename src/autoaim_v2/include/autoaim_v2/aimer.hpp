#ifndef AUTOAIM_V2__AIMER_HPP_
#define AUTOAIM_V2__AIMER_HPP_

#include <Eigen/Dense>

#include "autoaim_v2/ballistics.hpp"
#include "autoaim_v2/tracker.hpp"
#include "autoaim_v2/types.hpp"

namespace aim
{

struct AimerParams
{
  BallisticParams ballistics{};

  // Latency actually spent AFTER the aim computation: serial write + micro
  // loop + gimbal settling into the new reference. Calibrate on the robot
  // (DEPLOYMENT.md); the capture->aim part is measured per frame, not here.
  double actuation_latency = 0.025;   // [s]
  // Feeder chain: shoot flag set -> pellet leaves the muzzle.
  double feeder_delay = 0.035;        // [s] MEASURE (high-fps phone video)
  double feeder_sigma = 0.006;        // [s] shot-to-shot dispersion
  double serial_quant_sigma = 0.0015; // [s] event-driven TX quantization
  double bullet_speed_sigma_frac = 0.02;  // muzzle speed spread (2 %)

  // System angular dispersion (barrel + mechanical repeatability), per axis.
  double disp_bullet_mrad = 3.0;
  double disp_system_mrad = 4.0;

  // Plate hit zone (small armor module).
  double plate_w = 0.135, plate_h = 0.125;
  double armor_pitch_deg = 15.0;

  // ── Fire regimes ──
  double omega_track = 2.0;    // [rad/s] below: aim the visible plate (TRACK)
  double omega_timed = 12.0;   // [rad/s] above: center-hold + timed pulses
  // TRACK/SWEEP full-auto gate; TIMED per-shot gate.
  double p_spray_min = 0.50;
  double p_timed_min = 0.35;
  // SWEEP (sp_vision-style): aim plates entering the window, skip leaving.
  double coming_angle_deg = 55.0;
  double leaving_angle_deg = 20.0;

  double min_fire_dist = 0.3;
  double max_fire_dist = 6.5;

  // Timed-fire scheduling.
  double shot_pulse = 0.020;         // [s] shoot flag high per scheduled shot
  double shot_min_interval = 0.080;  // [s] >= firmware 1/shooting_frequency

  // Barrel muzzle offset from the gimbal pivot, yaw-rotated frame
  // (x fwd, y left, z up) — same meaning as the old pipeline's
  // barrel_offset_* measured from the camera, PLUS the camera lever arm.
  double barrel_x = 0.0, barrel_y = 0.0, barrel_z = -0.05;
  double gimbal_height = 0.42;

  // Boresight calibration (degrees, subtracted from the command).
  double yaw_offset_deg = 0.0, pitch_offset_deg = 0.0;

  // Fire lock: command vs micro feedback tolerance.
  double fire_lock_yaw = 0.05, fire_lock_pitch = 0.04;

  // Micro conventions (identical semantics to the old pipeline).
  double yaw_sign = 1.0, pitch_sign = -1.0;
  bool micro_pitch_lock_opposite_sign = false;
};

enum class FireRegime { NONE, TRACK, SWEEP, TIMED };

struct AimDebug
{
  FireRegime regime = FireRegime::NONE;
  int face = -1;
  double p_hit = 0;
  double omega = 0;
  double dist = 0;
  double flight_time = 0;
  double horizon = 0;         // total predict horizon [s]
  double lock_err_yaw = 0, lock_err_pitch = 0;
  bool locked = false;
  Eigen::Vector3d aim_point{0, 0, 0};
  double next_shot_in = -1;   // [s], TIMED only
};

// Prediction + plate selection + ballistics + fire policy.
// Stateless except for timed-shot bookkeeping.
class Aimer
{
public:
  explicit Aimer(const AimerParams & p) : p_(p) {}

  // t_now: aim-computation time; pipeline latency = t_now - tracker.stamp().
  // gs_now: latest gimbal sample (internal + raw micro angles).
  // ego / ego_vel: robot position and velocity in world.
  GimbalCommand plan(const Tracker & trk, TimePoint t_now,
                     const GimbalSample & gs_now, const Eigen::Vector2d & ego,
                     const Eigen::Vector2d & ego_vel, AimDebug & dbg);

  const AimerParams & params() const { return p_; }
  void set_bullet_speed(double v) { p_.ballistics.bullet_speed = v; }

private:
  struct FaceAim
  {
    int face = -1;
    Eigen::Vector3d pos{0, 0, 0};
    double bearing = 0, gd = 0, dz = 0, range = 0;
    BallisticSolution bal{};
    double obliquity = 0;   // |plate angle - LOS| at impact
    bool valid = false;
  };

  FaceAim aim_face(const Eigen::VectorXd & xs, int face,
                   const Eigen::Vector3d & barrel) const;
  double p_hit_static(const FaceAim & f) const;

  AimerParams p_;
  TimePoint last_shot_{};
  bool last_shot_valid_ = false;
};

}  // namespace aim

#endif  // AUTOAIM_V2__AIMER_HPP_
