#include <gtest/gtest.h>

#include <random>

#include "autoaim_v2/gimbal_buffer.hpp"
#include "autoaim_v2/tracker.hpp"

using namespace aim;

namespace
{

struct SimCar
{
  Eigen::Vector2d c{3.0, 0.5};
  Eigen::Vector2d v{0.0, 0.0};
  double z = 0.25;
  double theta = 0.3;
  double omega = 0.0;
  double r0 = 0.20, r1 = 0.23;  // pair radii (l = r1 - r0)
  double h = 0.04;              // odd-pair height offset

  void step(double dt)
  {
    c += v * dt;
    theta = norm_angle(theta + omega * dt);
  }

  // Faces with obliquity < 60 deg produce measurements.
  std::vector<ArmorWorld> observe(const Eigen::Vector3d & cam0, std::mt19937 & rng,
                                  double sig_b = 0.002, double sig_d = 0.03,
                                  double sig_th = 0.035) const
  {
    std::normal_distribution<double> nb(0, sig_b), nd(0, sig_d), nth(0, sig_th);
    std::vector<ArmorWorld> out;
    for (int i = 0; i < 4; i++) {
      const double phi = theta + i * M_PI / 2;
      const double r = (i % 2) ? r1 : r0;
      const double zz = z + ((i % 2) ? h : 0.0);
      Eigen::Vector3d pos(c.x() - r * std::cos(phi), c.y() - r * std::sin(phi), zz);

      Eigen::Vector3d rel = pos - cam0;
      const double bearing = std::atan2(rel.y(), rel.x());
      const double obl = std::fabs(ang_diff(phi, bearing));
      if (obl > 60 * M_PI / 180) continue;

      ArmorWorld a;
      a.class_id = 0;
      a.confidence = 0.9f;
      a.yaw = bearing + nb(rng);
      a.pitch = std::atan2(rel.z(), std::hypot(rel.x(), rel.y())) + nb(rng);
      a.dist = rel.norm() + nd(rng);
      a.theta_a = norm_angle(phi + nth(rng));
      a.theta_sigma = sig_th;
      // Reconstruct the (noisy) cartesian pos the solver would report.
      const double cp = std::cos(a.pitch);
      a.pos_world = cam0 + Eigen::Vector3d(a.dist * cp * std::cos(a.yaw),
                                           a.dist * cp * std::sin(a.yaw),
                                           a.dist * std::sin(a.pitch));
      a.pos_cam = a.pos_world - cam0;  // magnitude only used for range gating
      out.push_back(a);
    }
    return out;
  }
};

TrackerParams test_params()
{
  TrackerParams tp;
  tp.gimbal_height = 0.42;
  return tp;
}

}  // namespace

TEST(Tracker, ConvergesOnSpinner)
{
  std::mt19937 rng(11);
  SimCar car;
  car.omega = 25.0;  // ~240 RPM
  car.v = {0.8, -0.4};

  Tracker trk(test_params());
  const Eigen::Vector3d cam0(0, 0, 0.42);
  const auto t0 = Clock::now();
  const double dt = 1.0 / 120;

  for (int i = 0; i < 90; i++) {  // 0.75 s
    car.step(dt);
    auto obs = car.observe(cam0, rng);
    trk.update(obs, t0 + std::chrono::microseconds(int64_t(i * dt * 1e6)), cam0);
  }

  ASSERT_EQ(trk.state(), Tracker::TRACKING);
  const auto & x = trk.x();
  EXPECT_NEAR(x(7), car.omega, 1.5) << "spin rate";
  EXPECT_NEAR(x(0), car.c.x(), 0.06) << "center x";
  EXPECT_NEAR(x(2), car.c.y(), 0.06) << "center y";
  EXPECT_NEAR(x(1), car.v.x(), 0.45) << "vx";
  EXPECT_NEAR(x(3), car.v.y(), 0.45) << "vy";
  EXPECT_NEAR(x(8), car.r0, 0.05) << "radius";

  // The number that decides hits: predicted plate position 150 ms ahead.
  auto xs = trk.predict_state(0.150);
  SimCar future = car;
  future.step(0.150);
  double best_err = 1e9;
  for (int i = 0; i < 4; i++) {
    // Compare each predicted face against the true future face positions
    // (association-free check).
    Eigen::Vector3d pred = Tracker::armor_pos(xs, i);
    for (int j = 0; j < 4; j++) {
      const double phi = future.theta + j * M_PI / 2;
      const double r = (j % 2) ? future.r1 : future.r0;
      Eigen::Vector3d tru(future.c.x() - r * std::cos(phi),
                          future.c.y() - r * std::sin(phi),
                          future.z + ((j % 2) ? future.h : 0.0));
      best_err = std::min(best_err, (pred - tru).norm());
    }
  }
  EXPECT_LT(best_err, 0.05) << "150 ms plate prediction error";
}

TEST(Tracker, TracksPureTranslation)
{
  std::mt19937 rng(5);
  SimCar car;
  car.omega = 0.0;
  car.v = {1.5, 0.8};
  car.theta = 0.1;

  Tracker trk(test_params());
  const Eigen::Vector3d cam0(0, 0, 0.42);
  const auto t0 = Clock::now();
  const double dt = 1.0 / 120;

  for (int i = 0; i < 90; i++) {
    car.step(dt);
    auto obs = car.observe(cam0, rng);
    trk.update(obs, t0 + std::chrono::microseconds(int64_t(i * dt * 1e6)), cam0);
  }
  ASSERT_EQ(trk.state(), Tracker::TRACKING);
  EXPECT_NEAR(trk.x()(1), car.v.x(), 0.4);
  EXPECT_NEAR(trk.x()(3), car.v.y(), 0.4);
  EXPECT_LT(std::fabs(trk.x()(7)), 2.0);
}

TEST(Tracker, SurvivesOcclusionAndKeepsSpin)
{
  std::mt19937 rng(7);
  SimCar car;
  car.omega = 20.0;

  Tracker trk(test_params());
  const Eigen::Vector3d cam0(0, 0, 0.42);
  const auto t0 = Clock::now();
  const double dt = 1.0 / 120;
  int i = 0;
  for (; i < 60; i++) {
    car.step(dt);
    trk.update(car.observe(cam0, rng),
               t0 + std::chrono::microseconds(int64_t(i * dt * 1e6)), cam0);
  }
  ASSERT_EQ(trk.state(), Tracker::TRACKING);
  const double w_before = trk.x()(7);

  // 300 ms behind a wall: no detections.
  for (; i < 96; i++) {
    car.step(dt);
    trk.update({}, t0 + std::chrono::microseconds(int64_t(i * dt * 1e6)), cam0);
  }
  EXPECT_EQ(trk.state(), Tracker::TEMP_LOST);
  EXPECT_NEAR(trk.x()(7), w_before, 1.0) << "omega must NOT decay behind walls";

  // Reappears: reacquire within a few frames.
  for (; i < 110; i++) {
    car.step(dt);
    trk.update(car.observe(cam0, rng),
               t0 + std::chrono::microseconds(int64_t(i * dt * 1e6)), cam0);
  }
  EXPECT_EQ(trk.state(), Tracker::TRACKING);
}

TEST(Tracker, ReinitsAfterTeleport)
{
  std::mt19937 rng(9);
  SimCar car;
  Tracker trk(test_params());
  const Eigen::Vector3d cam0(0, 0, 0.42);
  const auto t0 = Clock::now();
  const double dt = 1.0 / 120;
  int i = 0;
  for (; i < 40; i++) {
    car.step(dt);
    trk.update(car.observe(cam0, rng),
               t0 + std::chrono::microseconds(int64_t(i * dt * 1e6)), cam0);
  }
  const int gen_before = trk.generation();

  // Enemy crossed behind cover: shows up 2.5 m away.
  car.c += Eigen::Vector2d(-1.0, 2.3);
  for (; i < 60; i++) {
    car.step(dt);
    trk.update(car.observe(cam0, rng),
               t0 + std::chrono::microseconds(int64_t(i * dt * 1e6)), cam0);
  }
  EXPECT_GT(trk.generation(), gen_before) << "should re-seed on the far side";
  EXPECT_NEAR(trk.x()(0), car.c.x(), 0.30);
  EXPECT_NEAR(trk.x()(2), car.c.y(), 0.30);
}
