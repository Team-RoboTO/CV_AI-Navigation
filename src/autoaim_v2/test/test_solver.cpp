#include <gtest/gtest.h>

#include <opencv2/calib3d.hpp>

#include "autoaim_v2/gimbal_buffer.hpp"
#include "autoaim_v2/solver.hpp"

using namespace aim;

namespace
{

// Independent synthetic projector (explicit matrices, no solver internals
// beyond the shared frame conventions documented in DESIGN.md).
struct SynthCam
{
  double fx = 730, fy = 730, cx = 480, cy = 300;

  Eigen::Matrix3d Rz(double a) const
  {
    Eigen::Matrix3d m;
    m << cos(a), -sin(a), 0, sin(a), cos(a), 0, 0, 0, 1;
    return m;
  }
  Eigen::Matrix3d Ry(double a) const
  {
    Eigen::Matrix3d m;
    m << cos(a), 0, sin(a), 0, 1, 0, -sin(a), 0, cos(a);
    return m;
  }
  Eigen::Matrix3d Rx(double a) const
  {
    Eigen::Matrix3d m;
    m << 1, 0, 0, 0, cos(a), -sin(a), 0, sin(a), cos(a);
    return m;
  }

  // camera->world rotation for gimbal (yaw, pitch); axis conv as in DESIGN.md.
  Eigen::Matrix3d R_wc(double yaw, double pitch) const
  {
    return Rz(yaw) * Ry(pitch) * (Rz(-M_PI / 2) * Rx(-M_PI / 2));
  }

  cv::Point2f project(const Eigen::Vector3d & p_world, double yaw, double pitch,
                      const Eigen::Vector3d & cam0) const
  {
    Eigen::Vector3d pc = R_wc(yaw, pitch).transpose() * (p_world - cam0);
    return {static_cast<float>(fx * pc.x() / pc.z() + cx),
            static_cast<float>(fy * pc.y() / pc.z() + cy)};
  }
};

// Plate corners in world for plate at pos with plate-angle theta_a.
std::array<Eigen::Vector3d, 4> plate_corners_world(const Eigen::Vector3d & pos,
                                                   double theta_a, double rho,
                                                   double w, double h)
{
  const double cyw = cos(theta_a), syw = sin(theta_a);
  const double cr = cos(rho), sr = sin(rho);
  Eigen::Matrix3d R;  // armor->world (x inward, y lateral, z up-tilted)
  R << cyw * cr, -syw, cyw * sr, syw * cr, cyw, syw * sr, -sr, 0, cr;

  const double hy = w / 2, hz = h / 2;
  const std::array<Eigen::Vector3d, 4> obj = {
    Eigen::Vector3d(0, hy, hz), Eigen::Vector3d(0, -hy, hz),
    Eigen::Vector3d(0, -hy, -hz), Eigen::Vector3d(0, hy, -hz)};
  std::array<Eigen::Vector3d, 4> out;
  for (int i = 0; i < 4; i++) out[i] = pos + R * obj[i];
  return out;
}

}  // namespace

class SolverTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    SolverParams sp;
    sp.gimbal_height = 0.42;
    solver = std::make_unique<Solver>(sp);
    solver->set_intrinsics(cam.fx, cam.fy, cam.cx, cam.cy);
  }
  SynthCam cam;
  std::unique_ptr<Solver> solver;
};

TEST_F(SolverTest, RecoversFrontalPlate)
{
  const double gy = 0.0, gp = 0.0;
  const Eigen::Vector3d cam0(0, 0, 0.42);
  const Eigen::Vector3d plate(3.0, 0.0, 0.30);
  const double theta_true = 0.0;  // frontal: theta_a == bearing == 0

  ArmorPx px;
  px.class_id = 0;
  px.confidence = 0.9f;
  auto corners = plate_corners_world(plate, theta_true, 15 * M_PI / 180, 0.135, 0.056);
  for (int i = 0; i < 4; i++) px.corners[i] = cam.project(corners[i], gy, gp, cam0);

  auto aw = solver->solve(px, gy, gp, Eigen::Vector2d(0, 0));
  ASSERT_TRUE(aw.has_value());
  EXPECT_NEAR(aw->pos_world.x(), plate.x(), 0.02);
  EXPECT_NEAR(aw->pos_world.y(), plate.y(), 0.01);
  EXPECT_NEAR(aw->pos_world.z(), plate.z(), 0.02);
  EXPECT_NEAR(ang_diff(aw->theta_a, theta_true), 0.0, 0.06);
  EXPECT_NEAR(aw->yaw, 0.0, 0.005);
  EXPECT_NEAR(aw->dist, (plate - cam0).norm(), 0.03);
}

TEST_F(SolverTest, RecoversObliquePlateWithMovingGimbal)
{
  // Gimbal rotated and pitched; plate 30 deg oblique at 2.5 m, off-axis.
  const double gy = 0.35, gp = -0.06;
  const Eigen::Vector3d cam0(0, 0, 0.42);
  const Eigen::Vector3d plate(2.2, 0.9, 0.25);
  const double bearing = std::atan2(plate.y(), plate.x());
  const double theta_true = bearing + 30 * M_PI / 180;

  ArmorPx px;
  auto corners = plate_corners_world(plate, theta_true, 15 * M_PI / 180, 0.135, 0.056);
  for (int i = 0; i < 4; i++) px.corners[i] = cam.project(corners[i], gy, gp, cam0);

  auto aw = solver->solve(px, gy, gp, Eigen::Vector2d(0, 0));
  ASSERT_TRUE(aw.has_value());
  EXPECT_NEAR(aw->pos_world.x(), plate.x(), 0.03);
  EXPECT_NEAR(aw->pos_world.y(), plate.y(), 0.03);
  EXPECT_NEAR(ang_diff(aw->theta_a, theta_true), 0.0, 0.09)
    << "theta search should recover a 30deg-oblique plate to ~2-3 deg";
}

TEST_F(SolverTest, ThetaSearchBeatsRawPnPUnderNoise)
{
  // With 0.7 px corner noise the reprojection search should keep the plate
  // angle within a few degrees RMS.
  const double gy = 0.0, gp = 0.0;
  const Eigen::Vector3d cam0(0, 0, 0.42);
  const Eigen::Vector3d plate(3.5, -0.4, 0.28);
  const double bearing = std::atan2(plate.y(), plate.x());
  const double theta_true = bearing - 20 * M_PI / 180;

  std::srand(3);
  auto noise = [] { return (std::rand() / double(RAND_MAX) - 0.5) * 2 * 0.7; };

  double sum2 = 0;
  const int N = 30;
  for (int k = 0; k < N; k++) {
    ArmorPx px;
    auto corners =
      plate_corners_world(plate, theta_true, 15 * M_PI / 180, 0.135, 0.056);
    for (int i = 0; i < 4; i++) {
      auto p = cam.project(corners[i], gy, gp, cam0);
      px.corners[i] = {static_cast<float>(p.x + noise()),
                       static_cast<float>(p.y + noise())};
    }
    auto aw = solver->solve(px, gy, gp, Eigen::Vector2d(0, 0));
    ASSERT_TRUE(aw.has_value());
    const double e = ang_diff(aw->theta_a, theta_true);
    sum2 += e * e;
  }
  const double rms_deg = std::sqrt(sum2 / N) * 180 / M_PI;
  EXPECT_LT(rms_deg, 5.0) << "plate-angle RMS under noise";
}
