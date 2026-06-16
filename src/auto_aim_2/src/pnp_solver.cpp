#include "auto_aim/pnp_solver.hpp"

#include <opencv2/calib3d.hpp>
#include <limits>

namespace auto_aim
{

void PnPSolver::init(const std::array<double, 9> & K, const std::vector<double> & dist)
{
  K_ = cv::Mat(3, 3, CV_64F, const_cast<double *>(K.data())).clone();
  dist_ = cv::Mat(1, static_cast<int>(dist.size()), CV_64F, const_cast<double *>(dist.data())).clone();

  auto make_pts_tl_tr_br_bl = [](double w_mm, double h_mm) {
    const double hy = w_mm / 2.0 / 1000.0;  // lateral half-width [m]
    const double hz = h_mm / 2.0 / 1000.0;  // vertical half-height [m]

    // Object frame convention kept compatible with the original package:
    //   x = plate normal, y = lateral, z = vertical.
    // For a camera-facing plate, the point order below corresponds to image
    // order TL, TR, BR, BL.
    return std::vector<cv::Point3f>{
      {0.0f,  static_cast<float>( hy), static_cast<float>( hz)},  // TL
      {0.0f,  static_cast<float>(-hy), static_cast<float>( hz)},  // TR
      {0.0f,  static_cast<float>(-hy), static_cast<float>(-hz)},  // BR
      {0.0f,  static_cast<float>( hy), static_cast<float>(-hz)}   // BL
    };
  };

  pts_small_tl_tr_br_bl_ = make_pts_tl_tr_br_bl(SMALL_W, SMALL_H);
  pts_large_tl_tr_br_bl_ = make_pts_tl_tr_br_bl(LARGE_W, LARGE_H);
  ready_ = true;
}

double PnPSolver::solveWithModel(
  const std::vector<cv::Point3f> & obj_pts,
  const std::vector<cv::Point2f> & img_pts,
  cv::Mat & rvec, cv::Mat & tvec) const
{
  if (!ready_ || img_pts.size() != 4 || obj_pts.size() != 4) return 1e9;

  // IPPE is designed for planar targets and is usually more stable for armor plates.
  // However it can fail on degenerate keypoint configurations (near-collinear points,
  // very small image area, partial occlusion, motion blur). When IPPE fails we fall
  // back to SOLVEPNP_ITERATIVE, which is more permissive but less accurate.
  // This costs at most one extra solver call per failure (~1 ms) and only on the
  // problematic frames, while greatly improving robustness at close range and
  // oblique viewing angles.
  bool ok = cv::solvePnP(obj_pts, img_pts, K_, dist_, rvec, tvec,
                         false, cv::SOLVEPNP_IPPE);
  if (!ok) {
    // Reset output mats — solvePnP may have written partial data.
    rvec = cv::Mat();
    tvec = cv::Mat();
    ok = cv::solvePnP(obj_pts, img_pts, K_, dist_, rvec, tvec,
                      false, cv::SOLVEPNP_ITERATIVE);
    if (!ok) return 1e9;
  }

  std::vector<cv::Point2f> proj;
  cv::projectPoints(obj_pts, rvec, tvec, K_, dist_, proj);
  double sum = 0.0;
  for (size_t i = 0; i < img_pts.size(); ++i) {
    sum += cv::norm(proj[i] - img_pts[i]);
  }
  return sum / static_cast<double>(img_pts.size());
}

bool PnPSolver::solveKeypoints(
  const std::array<cv::Point2f, 4> & corners_tl_tr_br_bl,
  cv::Mat & rvec, cv::Mat & tvec, bool & is_large,
  double * reproj_error, double max_reproj_error) const
{
  const std::vector<cv::Point2f> img_pts = {
    corners_tl_tr_br_bl[0],  // TL
    corners_tl_tr_br_bl[1],  // TR
    corners_tl_tr_br_bl[2],  // BR
    corners_tl_tr_br_bl[3]   // BL
  };

  cv::Mat rv_s, tv_s, rv_l, tv_l;
  const double err_s = solveWithModel(pts_small_tl_tr_br_bl_, img_pts, rv_s, tv_s);
  const double err_l = solveWithModel(pts_large_tl_tr_br_bl_, img_pts, rv_l, tv_l);

  if (err_s > 1e8 && err_l > 1e8) return false;

  const bool choose_small = err_s <= err_l;
  const double err = choose_small ? err_s : err_l;
  if (reproj_error) *reproj_error = err;

  if (choose_small) {
    rvec = rv_s;
    tvec = tv_s;
    is_large = false;
  } else {
    rvec = rv_l;
    tvec = tv_l;
    is_large = true;
  }

  return err <= max_reproj_error;
}

bool PnPSolver::solve(
  double cx, double cy, double w, double h, double lr,
  cv::Mat & rvec, cv::Mat & tvec, bool & is_large, double max_err) const
{
  // Reconstruct bbox corners in the same TL,TR,BR,BL order used by the keypoint path.
  const float lx = static_cast<float>(cx - lr * w / 2.0);
  const float rx = static_cast<float>(cx + lr * w / 2.0);
  const float ty = static_cast<float>(cy - lr * h / 2.0);
  const float by = static_cast<float>(cy + lr * h / 2.0);

  const std::array<cv::Point2f, 4> corners = {
    cv::Point2f(lx, ty),  // TL
    cv::Point2f(rx, ty),  // TR
    cv::Point2f(rx, by),  // BR
    cv::Point2f(lx, by)   // BL
  };

  return solveKeypoints(corners, rvec, tvec, is_large, nullptr, max_err);
}

}  // namespace auto_aim
