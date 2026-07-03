#include <gtest/gtest.h>

#include <random>

#include "autoaim_v2/aimer.hpp"
#include "autoaim_v2/gimbal_buffer.hpp"

using namespace aim;

namespace
{

// Drive a tracker into TRACKING on a synthetic car (noise-free for aimer
// determinism).
struct Rig
{
  TrackerParams tp;
  Tracker trk{[] {
    TrackerParams p;
    p.gimbal_height = 0.42;
    return p;
  }()};
  Eigen::Vector3d cam0{0, 0, 0.42};
  TimePoint t0 = Clock::now();
  double t_sim = 0;

  Eigen::Vector2d c{3.0, 0.0};
  Eigen::Vector2d v{0, 0};
  double z = 0.25, theta = 0.0, omega = 0.0, r = 0.20;

  void run(double duration, double dt = 1.0 / 120)
  {
    std::mt19937 rng(1);
    std::normal_distribution<double> tiny(0, 1e-4);
    int n = static_cast<int>(duration / dt);
    for (int k = 0; k < n; k++) {
      t_sim += dt;
      c += v * dt;
      theta = norm_angle(theta + omega * dt);
      std::vector<ArmorWorld> obs;
      for (int i = 0; i < 4; i++) {
        const double phi = theta + i * M_PI / 2;
        Eigen::Vector3d pos(c.x() - r * std::cos(phi), c.y() - r * std::sin(phi), z);
        Eigen::Vector3d rel = pos - cam0;
        const double bearing = std::atan2(rel.y(), rel.x());
        if (std::fabs(ang_diff(phi, bearing)) > 60 * M_PI / 180) continue;
        ArmorWorld a;
        a.yaw = bearing + tiny(rng);
        a.pitch = std::atan2(rel.z(), std::hypot(rel.x(), rel.y()));
        a.dist = rel.norm();
        a.theta_a = phi;
        a.theta_sigma = 0.03;
        a.pos_world = pos;
        a.pos_cam = rel;
        obs.push_back(a);
      }
      trk.update(obs, now(), cam0);
    }
  }

  TimePoint now() const
  {
    return t0 + std::chrono::microseconds(static_cast<int64_t>(t_sim * 1e6));
  }
};

AimerParams test_aimer_params()
{
  AimerParams ap;
  ap.gimbal_height = 0.42;
  ap.fire_lock_yaw = 0.30;   // generous locks: we test policy, not servo
  ap.fire_lock_pitch = 0.30;
  ap.pitch_sign = -1.0;
  return ap;
}

GimbalSample aligned_gimbal(const Rig & rig)
{
  GimbalSample g;
  g.yaw = std::atan2(rig.c.y(), rig.c.x());
  g.yaw_raw = g.yaw;      // yaw_sign = +1
  g.pitch = 0.0;
  g.pitch_raw = 0.0;
  g.t = rig.now();
  return g;
}

}  // namespace

TEST(Aimer, TrackRegimeFiresOnStaticTarget)
{
  Rig rig;
  rig.run(0.5);
  ASSERT_EQ(rig.trk.state(), Tracker::TRACKING);

  Aimer aimer(test_aimer_params());
  AimDebug dbg;
  auto cmd = aimer.plan(rig.trk, rig.now(), aligned_gimbal(rig),
                        {0, 0}, {0, 0}, dbg);

  ASSERT_TRUE(cmd.valid);
  EXPECT_EQ(dbg.regime, FireRegime::TRACK);
  EXPECT_TRUE(dbg.locked);
  EXPECT_GT(dbg.p_hit, 0.9) << "3 m static plate must be near-certain";
  EXPECT_TRUE(cmd.shoot);
  EXPECT_NEAR(cmd.yaw_micro, 0.0, 0.03);

  // Pitch chain check: the plate front face is at x = c - r = 2.8 m, z 0.25;
  // barrel at z = 0.42 - 0.05 = 0.37. Independent ballistic solve for that
  // geometry gives the internal pitch; micro pitch = pitch_sign * internal.
  {
    const auto & ap = aimer.params();
    const double gd = (rig.c.x() - rig.r) - ap.barrel_x;
    const double dz = rig.z - (ap.gimbal_height + ap.barrel_z);
    auto ref = solve_ballistics(ap.ballistics, gd, dz);
    ASSERT_TRUE(ref.valid);
    EXPECT_LT(ref.pitch, 0.0) << "plate below barrel: solver must aim down";
    EXPECT_NEAR(cmd.pitch_micro, ap.pitch_sign * ref.pitch, 0.004);
  }
}

TEST(Aimer, LeadsTranslatingTarget)
{
  Rig rig;
  rig.v = {0.0, 1.5};  // strafing left at 1.5 m/s
  rig.run(0.6);
  ASSERT_EQ(rig.trk.state(), Tracker::TRACKING);

  Aimer aimer(test_aimer_params());
  AimDebug dbg;
  auto cmd = aimer.plan(rig.trk, rig.now(), aligned_gimbal(rig), {0, 0}, {0, 0}, dbg);
  ASSERT_TRUE(cmd.valid);

  // Bearing must LEAD the current position: horizon ~ (age + actuation +
  // flight) ~ 0.15 s -> lead ~ v*h / d ~ 0.075 rad ahead of current bearing.
  const double bearing_now = std::atan2(rig.c.y(), rig.c.x());
  const double lead = ang_diff(cmd.yaw_micro, bearing_now);
  EXPECT_GT(lead, 0.03) << "must aim ahead of a strafing target";
  EXPECT_LT(lead, 0.15);
}

TEST(Aimer, TimedRegimePulsesOnFastSpinner)
{
  Rig rig;
  rig.omega = 30.0;  // ~286 RPM -> TIMED
  rig.run(0.7);
  ASSERT_EQ(rig.trk.state(), Tracker::TRACKING);
  ASSERT_GT(std::fabs(rig.trk.x()(7)), 20.0);

  Aimer aimer(test_aimer_params());

  // Simulate 1 s of aim iterations at 120 Hz WITHOUT tracker updates
  // (frozen state, advancing clock) and count scheduled shots.
  int pulses = 0;
  TimePoint prev_shot_t{};
  double min_gap = 1e9;
  AimDebug dbg;
  for (int k = 0; k < 120; k++) {
    TimePoint t = rig.now() + std::chrono::microseconds(int64_t(k * 8333.3));
    auto cmd = aimer.plan(rig.trk, t, aligned_gimbal(rig), {0, 0}, {0, 0}, dbg);
    ASSERT_TRUE(cmd.valid);
    EXPECT_EQ(dbg.regime, FireRegime::TIMED);
    if (cmd.shoot_scheduled) {
      pulses++;
      // Scheduled fire moments must be imminent (sub-frame) and non-negative.
      const double lead = seconds(cmd.shoot_at, t);
      EXPECT_GE(lead, 0.0);
      EXPECT_LE(lead, 0.010);

      // CORE INVARIANT: pellet arrival (= shoot_at + feeder + flight) must
      // coincide with a plate crossing the barrel line of sight.
      const double t_arr = seconds(cmd.shoot_at, rig.trk.stamp()) +
                           aimer.params().feeder_delay + dbg.flight_time;
      const auto & x = rig.trk.x();
      const double beta = std::atan2(x(2), x(0));  // barrel ~ origin
      double min_off = 1e9;
      for (int f = 0; f < 4; f++) {
        const double th = x(6) + x(7) * t_arr + f * M_PI / 2;
        min_off = std::min(min_off, std::fabs(ang_diff(th, beta)));
      }
      // 0.12 rad at 30 rad/s = 4 ms arrival error budget.
      EXPECT_LT(min_off, 0.12) << "arrival must be on a plate crossing";

      if (prev_shot_t.time_since_epoch().count() > 0) {
        min_gap = std::min(min_gap, seconds(cmd.shoot_at, prev_shot_t));
      }
      prev_shot_t = cmd.shoot_at;
    }
  }

  // Plates cross the LOS 4*omega/(2pi) ~ 19 times/s; with the 80 ms min
  // interval we expect ~8-12 scheduled shots in 1 s.
  EXPECT_GE(pulses, 4) << "timed fire must schedule shots";
  EXPECT_LE(pulses, 14);
  if (min_gap < 1e9) {
    EXPECT_GE(min_gap, aimer.params().shot_min_interval - 0.010);
  }

  // Center-hold: yaw stays at the center bearing (no plate chasing).
  EXPECT_NEAR(dbg.aim_point.y(), 0.0, 0.06);
}

TEST(Aimer, HoldsFireWhenTempLost)
{
  Rig rig;
  rig.run(0.5);
  // Push the tracker into TEMP_LOST with empty frames.
  for (int k = 0; k < 12; k++) {
    rig.t_sim += 1.0 / 120;
    rig.trk.update({}, rig.now(), rig.cam0);
  }
  ASSERT_EQ(rig.trk.state(), Tracker::TEMP_LOST);

  Aimer aimer(test_aimer_params());
  AimDebug dbg;
  auto cmd = aimer.plan(rig.trk, rig.now(), aligned_gimbal(rig), {0, 0}, {0, 0}, dbg);
  EXPECT_TRUE(cmd.valid) << "keep aiming at the prediction while TEMP_LOST";
  EXPECT_FALSE(cmd.shoot) << "never fire without confirmation";
}
