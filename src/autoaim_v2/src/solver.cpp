#include "autoaim_v2/solver.hpp"

#include <cmath>
#include <opencv2/calib3d.hpp>

#include "autoaim_v2/gimbal_buffer.hpp"  // norm_angle / ang_diff

namespace aim
{
namespace
{

Eigen::Matrix3d Rx(double a)
{
  Eigen::Matrix3d m;
  m << 1, 0, 0, 0, std::cos(a), -std::sin(a), 0, std::sin(a), std::cos(a);
  return m;
}
Eigen::Matrix3d Ry(double a)
{
  Eigen::Matrix3d m;
  m << std::cos(a), 0, std::sin(a), 0, 1, 0, -std::sin(a), 0, std::cos(a);
  return m;
}
Eigen::Matrix3d Rz(double a)
{
  Eigen::Matrix3d m;
  m << std::cos(a), -std::sin(a), 0, std::sin(a), std::cos(a), 0, 0, 0, 1;
  return m;
}

// Plate rotation in world for plate angle theta_a (armor x-axis = inward
// normal, azimuth theta_a, tilted down by rho; y lateral-left; z up-tilted).
Eigen::Matrix3d R_armor_world(double theta_a, double rho)
{
  const double cy = std::cos(theta_a), sy = std::sin(theta_a);
  const double cr = std::cos(rho), sr = std::sin(rho);
  Eigen::Matrix3d R;
  R << cy * cr, -sy, cy * sr,
       sy * cr,  cy, sy * sr,
           -sr,   0,      cr;
  return R;
}

}  // namespace

Solver::Solver(const SolverParams & p) : p_(p)
{
  // Camera optical (x right, y down, z fwd) -> gimbal body (x fwd, y left, z up):
  // same axes as the old node's q_conv = RPY(-pi/2, 0, -pi/2).
  Eigen::Matrix3d axis = Rz(-M_PI / 2) * Ry(0) * Rx(-M_PI / 2);
  Eigen::Matrix3d trim = Rz(p.trim_yaw_deg * M_PI / 180.0) *
                         Ry(p.trim_pitch_deg * M_PI / 180.0) *
                         Rx(p.trim_roll_deg * M_PI / 180.0);
  R_gc_ = trim * axis;

  auto make = [](double w, double h) {
    const float hy = static_cast<float>(w / 2), hz = static_cast<float>(h / 2);
    // Order TL, TR, BR, BL — matches the detector keypoints and the old solver.
    return std::vector<cv::Point3f>{
      {0, hy, hz}, {0, -hy, hz}, {0, -hy, -hz}, {0, hy, -hz}};
  };
  obj_small_ = make(p.small_w, p.small_h);
  obj_large_ = make(p.large_w, p.large_h);
}

void Solver::set_intrinsics(double fx, double fy, double cx, double cy,
                            const std::vector<double> & dist)
{
  K_ = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
  if (dist.empty()) {
    dist_ = cv::Mat::zeros(1, 5, CV_64F);
  } else {
    dist_ = cv::Mat(1, static_cast<int>(dist.size()), CV_64F);
    for (size_t i = 0; i < dist.size(); i++) dist_.at<double>(0, static_cast<int>(i)) = dist[i];
  }
  ready_ = true;
}

Eigen::Matrix3d Solver::R_world_cam(double yaw, double pitch) const
{
  return Rz(yaw) * Ry(pitch) * R_gc_;
}

Eigen::Vector3d Solver::cam_origin_world(double yaw, double pitch,
                                         const Eigen::Vector2d & ego) const
{
  Eigen::Vector3d lever = Rz(yaw) * Ry(pitch) * p_.t_cam2gimbal;
  return Eigen::Vector3d(ego.x(), ego.y(), p_.gimbal_height) + lever;
}

double Solver::reproj_cost(const Eigen::Vector3d & plate_world, double theta_a,
                           bool is_large, double yaw, double pitch,
                           const Eigen::Vector2d & ego,
                           const std::array<cv::Point2f, 4> & meas) const
{
  const Eigen::Matrix3d R_wc = R_world_cam(yaw, pitch);
  const Eigen::Vector3d cam0 = cam_origin_world(yaw, pitch, ego);

  const Eigen::Matrix3d R_aw =
    R_armor_world(theta_a, p_.armor_pitch_deg * M_PI / 180.0);
  const Eigen::Matrix3d R_ac = R_wc.transpose() * R_aw;           // armor->cam
  const Eigen::Vector3d t_ac = R_wc.transpose() * (plate_world - cam0);

  cv::Mat R_cv(3, 3, CV_64F), rvec;
  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 3; c++) R_cv.at<double>(r, c) = R_ac(r, c);
  cv::Rodrigues(R_cv, rvec);
  cv::Mat tvec = (cv::Mat_<double>(3, 1) << t_ac.x(), t_ac.y(), t_ac.z());

  std::vector<cv::Point2f> proj;
  cv::projectPoints(is_large ? obj_large_ : obj_small_, rvec, tvec, K_, dist_, proj);

  double cost = 0;
  for (int i = 0; i < 4; i++) cost += cv::norm(proj[i] - meas[i]);
  return cost;
}

double Solver::search_theta(const Eigen::Vector3d & plate_world, bool is_large,
                            double yaw, double pitch, const Eigen::Vector2d & ego,
                            const std::array<cv::Point2f, 4> & meas,
                            double bearing, double & sigma_out) const
{
  const double range = p_.search_range_deg * M_PI / 180.0;
  const double coarse = p_.coarse_step_deg * M_PI / 180.0;

  auto cost_at = [&](double th) {
    return reproj_cost(plate_world, th, is_large, yaw, pitch, ego, meas);
  };

  // Coarse pass around the line of sight (a visible plate cannot face away).
  double best_th = bearing, best_c = 1e18;
  for (double th = bearing - range; th <= bearing + range + 1e-9; th += coarse) {
    double c = cost_at(th);
    if (c < best_c) { best_c = c; best_th = th; }
  }

  // Fine pass +- one coarse step at 1 deg.
  const double fine = 1.0 * M_PI / 180.0;
  double lo = best_th - coarse, hi = best_th + coarse;
  for (double th = lo; th <= hi + 1e-9; th += fine) {
    double c = cost_at(th);
    if (c < best_c) { best_c = c; best_th = th; }
  }

  // Parabolic refinement + curvature-based sigma estimate.
  double cm = cost_at(best_th - fine);
  double cp = cost_at(best_th + fine);
  double denom = cm - 2 * best_c + cp;
  if (denom > 1e-9) {
    double delta = 0.5 * (cm - cp) / denom * fine;
    if (std::fabs(delta) < fine) best_th += delta;
    // cost ~ quadratic: c(th) = c0 + 0.5*k*(th-th0)^2 with k = denom/fine^2.
    // Corner noise ~0.7 px over 4 corners -> cost noise ~1.4 px; sigma where
    // the parabola rises by that much.
    double k = denom / (fine * fine);
    sigma_out = std::clamp(std::sqrt(2.0 * 1.4 / k), 0.02, 0.60);
  } else {
    sigma_out = 0.30;  // flat cost valley: orientation poorly observable
  }

  return norm_angle(best_th);
}

std::optional<ArmorWorld> Solver::solve(const ArmorPx & px, double yaw,
                                        double pitch,
                                        const Eigen::Vector2d & ego) const
{
  if (!ready_) return std::nullopt;

  const std::vector<cv::Point2f> img{px.corners[0], px.corners[1],
                                     px.corners[2], px.corners[3]};

  auto pnp = [&](const std::vector<cv::Point3f> & obj, cv::Mat & rvec,
                 cv::Mat & tvec) -> double {
    bool ok = cv::solvePnP(obj, img, K_, dist_, rvec, tvec, false,
                           cv::SOLVEPNP_IPPE);
    if (!ok) {
      rvec = cv::Mat();
      tvec = cv::Mat();
      ok = cv::solvePnP(obj, img, K_, dist_, rvec, tvec, false,
                        cv::SOLVEPNP_ITERATIVE);
      if (!ok) return 1e18;
    }
    std::vector<cv::Point2f> proj;
    cv::projectPoints(obj, rvec, tvec, K_, dist_, proj);
    double e = 0;
    for (int i = 0; i < 4; i++) e += cv::norm(proj[i] - img[i]);
    return e / 4.0;
  };

  cv::Mat rv_s, tv_s, rv_l, tv_l;
  double err_s = pnp(obj_small_, rv_s, tv_s);
  double err_l = p_.assume_small_armor ? 1e18 : pnp(obj_large_, rv_l, tv_l);
  const bool is_large = err_l < err_s;
  const double err = is_large ? err_l : err_s;
  const cv::Mat & tvec = is_large ? tv_l : tv_s;
  if (err > p_.max_reproj_err_px || tvec.empty()) return std::nullopt;

  Eigen::Vector3d p_cam(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
  if (!p_cam.allFinite() || p_cam.z() < 0.05 || p_cam.z() > 12.0) return std::nullopt;

  const Eigen::Matrix3d R_wc = R_world_cam(yaw, pitch);
  const Eigen::Vector3d cam0 = cam_origin_world(yaw, pitch, ego);
  const Eigen::Vector3d p_world = R_wc * p_cam + cam0;

  ArmorWorld out;
  out.class_id = px.class_id;
  out.confidence = px.confidence;
  out.is_large = is_large;
  out.pos_world = p_world;
  out.pos_cam = p_cam;
  out.reproj_err = err;

  const Eigen::Vector3d rel = p_world - cam0;
  out.dist = rel.norm();
  out.yaw = std::atan2(rel.y(), rel.x());
  out.pitch = std::atan2(rel.z(), std::hypot(rel.x(), rel.y()));

  out.theta_a = search_theta(p_world, is_large, yaw, pitch, ego, px.corners,
                             out.yaw, out.theta_sigma);
  return out;
}

}  // namespace aim
