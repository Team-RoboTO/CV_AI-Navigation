#include "auto_aim/pnp_solver.hpp"

#include <opencv2/calib3d.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

namespace auto_aim
{

void PnPSolver::init(const std::array<double, 9> & K, const std::vector<double> & dist)
{
  K_ = cv::Mat(3, 3, CV_64F, const_cast<double *>(K.data())).clone();
  dist_ = cv::Mat(1, (int)dist.size(), CV_64F, const_cast<double *>(dist.data())).clone();

  // Armor model in the local plate frame: X = plate normal (out of plate
  // toward viewer when face-on), Y = image-left when face-on, Z = up.
  //   TL = (0, +hy, +hz)   image-left,  image-up
  //   TR = (0, -hy, +hz)   image-right, image-up
  //   BR = (0, -hy, -hz)   image-right, image-down
  //   BL = (0, +hy, -hz)   image-left,  image-down
  // The 2D inputs to PnP come from ArmorKeypoint.msg and are in the same
  // TL,TR,BR,BL order; index-by-index correspondence with this array.
  auto make_pts_tl_tr_br_bl = [](double w_mm, double h_mm) {
    double hy = w_mm / 2.0 / 1000.0, hz = h_mm / 2.0 / 1000.0;
    return std::vector<cv::Point3f>{
      {0, (float)hy, (float)hz}, {0, (float)-hy, (float)hz},
      {0, (float)-hy, (float)-hz}, {0, (float)hy, (float)-hz}};
  };

  pts_small_ = make_pts_tl_tr_br_bl(SMALL_W, SMALL_H);
  pts_large_ = make_pts_tl_tr_br_bl(LARGE_W, LARGE_H);
  ready_ = true;
}

double PnPSolver::solveWithModel(
  const std::vector<cv::Point3f> & obj_pts,
  const std::vector<cv::Point2f> & img_pts,
  cv::Mat & rvec,
  cv::Mat & tvec,
  bool refine_lm) const
{
  if (!ready_ || obj_pts.size() != 4 || img_pts.size() != 4) {
    return std::numeric_limits<double>::infinity();
  }

  if (!cv::solvePnP(obj_pts, img_pts, K_, dist_, rvec, tvec, false, cv::SOLVEPNP_IPPE)) {
    return std::numeric_limits<double>::infinity();
  }
  if (refine_lm) {
    cv::solvePnPRefineLM(obj_pts, img_pts, K_, dist_, rvec, tvec);
  }

  std::vector<cv::Point2f> proj;
  cv::projectPoints(obj_pts, rvec, tvec, K_, dist_, proj);
  double sum = 0.0;
  for (size_t i = 0; i < img_pts.size(); ++i) {
    sum += cv::norm(proj[i] - img_pts[i]);
  }
  return sum / static_cast<double>(img_pts.size());
}

PnPResult PnPSolver::solveKeypoints(
  const std::array<cv::Point2f, 4> & corners_tl_tr_br_bl,
  double max_reproj_err,
  bool refine_lm,
  double max_reproj_err_norm,
  double min_depth_m,
  double max_depth_m,
  PnPSolver::ArmorSizeMode force_size) const
{
  PnPResult out;
  out.source = PnPResult::Source::KEYPOINT;

  std::vector<cv::Point2f> img_pts;
  img_pts.reserve(4);
  for (const auto & p : corners_tl_tr_br_bl) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
      out.reject_reason = PnPResult::REASON_NONFINITE;
      return out;
    }
    img_pts.push_back(p);
  }

  out.image_points = {
    corners_tl_tr_br_bl[0].x, corners_tl_tr_br_bl[0].y,
    corners_tl_tr_br_bl[1].x, corners_tl_tr_br_bl[1].y,
    corners_tl_tr_br_bl[2].x, corners_tl_tr_br_bl[2].y,
    corners_tl_tr_br_bl[3].x, corners_tl_tr_br_bl[3].y};

  const auto minmax_x = std::minmax_element(
    img_pts.begin(), img_pts.end(),
    [](const auto & a, const auto & b) { return a.x < b.x; });
  const auto minmax_y = std::minmax_element(
    img_pts.begin(), img_pts.end(),
    [](const auto & a, const auto & b) { return a.y < b.y; });
  const double w = static_cast<double>(minmax_x.second->x - minmax_x.first->x);
  const double h = static_cast<double>(minmax_y.second->y - minmax_y.first->y);
  if (w < 4.0 || h < 4.0) {
    out.reject_reason = PnPResult::REASON_DEGEN_BBOX;
    return out;
  }

  cv::Mat rv_s, tv_s, rv_l, tv_l;
  const double err_s = solveWithModel(pts_small_, img_pts, rv_s, tv_s, refine_lm);
  const double err_l = solveWithModel(pts_large_, img_pts, rv_l, tv_l, refine_lm);
  out.small_reproj_err = err_s;
  out.large_reproj_err = err_l;
  out.size_margin = (std::isfinite(err_s) && std::isfinite(err_l))
    ? std::abs(err_s - err_l)
    : std::numeric_limits<double>::infinity();
  if (!std::isfinite(err_s) && !std::isfinite(err_l)) {
    out.reject_reason = PnPResult::REASON_SOLVER_FAIL;
    return out;
  }

  if (force_size == ArmorSizeMode::SMALL || (force_size == ArmorSizeMode::AUTO && err_s <= err_l)) {
    out.rvec = rv_s; out.tvec = tv_s;
    out.is_large = false;
    out.reproj_err = err_s;
  } else {
    out.rvec = rv_l; out.tvec = tv_l;
    out.is_large = true;
    out.reproj_err = err_l;
  }
  out.solver_ok = std::isfinite(out.reproj_err);

  const double diag = std::sqrt(w * w + h * h);
  out.reproj_err_norm = (diag > 1.0) ? (out.reproj_err / diag) : out.reproj_err;

  if (!out.solver_ok) {
    out.reject_reason = PnPResult::REASON_SOLVER_FAIL;
    return out;
  }
  if (out.reproj_err > max_reproj_err) {
    out.reject_reason = PnPResult::REASON_REPROJ_ABS;
    return out;
  }
  if (max_reproj_err_norm > 0.0 && out.reproj_err_norm > max_reproj_err_norm) {
    out.reject_reason = PnPResult::REASON_REPROJ_NORM;
    return out;
  }
  const double depth = (out.tvec.empty() || out.tvec.rows < 3) ? 0.0
                       : out.tvec.at<double>(2);
  if (min_depth_m > 0.0 && depth < min_depth_m) {
    out.reject_reason = PnPResult::REASON_DEPTH_TOO_CLOSE;
    return out;
  }
  if (max_depth_m > 0.0 && depth > max_depth_m) {
    out.reject_reason = PnPResult::REASON_DEPTH_TOO_FAR;
    return out;
  }

  out.ok = true;
  out.reject_reason = PnPResult::REASON_OK;
  return out;
}

}  // namespace auto_aim
