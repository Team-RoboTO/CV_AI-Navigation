// ============================================================================
// pnp_solver.cpp — Perspective-n-Point solver for armor plate 3D pose.
//
// ENTRYPOINTS:
//   constructor              → build 3D armor model points, store camera params
//   solvePnP                 → recover 3D pose from 2D light-bar corners (IPPE)
//   calculateDistanceToCenter → pixel distance from detection to image center
// ============================================================================
#include "auto_aim_targeting/measurement/pnp_solver.hpp"

#include <opencv2/calib3d.hpp>
#include <vector>

namespace rm_auto_aim
{

// ---------------------------------------------------------------------------
// Constructor — build the 3D armor plate model points and store camera params.
//
// We pre-compute the two sets of 3D model points once here (not per frame)
// because they depend only on fixed physical armor dimensions.
//
// 3D model coordinate frame (armor body frame, right-hand convention):
//   X-axis: points OUTWARD from the front face of the armor (depth direction)
//           X=0 because all points lie in the frontal plane of the plate.
//   Y-axis: points LEFT when facing the armor (horizontal)
//   Z-axis: points UPWARD
//
// Point ordering — clockwise starting from BOTTOM-LEFT when facing the plate:
//   P0: bottom-left  (Y=+half_y, Z=-half_z)
//   P1: top-left     (Y=+half_y, Z=+half_z)
//   P2: top-right    (Y=-half_y, Z=+half_z)
//   P3: bottom-right (Y=-half_y, Z=-half_z)
//
// This ordering must exactly match the 2D image point ordering in solvePnP():
//   left_light.bottom → left_light.top → right_light.top → right_light.bottom
//
// WHY .clone(): cv::Mat normally shares data with the source array.  The input
// arrays (camera_matrix, dist_coeffs) are stack-allocated — they go out of scope
// after the constructor returns.  .clone() makes a deep copy that outlives them.
// ---------------------------------------------------------------------------
PnPSolver::PnPSolver(
  const std::array<double, 9> & camera_matrix, const std::vector<double> & dist_coeffs)
: camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone()),
  dist_coeffs_(cv::Mat(1, static_cast<int>(dist_coeffs.size()), CV_64F, const_cast<double *>(dist_coeffs.data())).clone())
{
  // Convert half-dimensions from mm to metres
  constexpr double small_half_y = SMALL_ARMOR_WIDTH  / 2.0 / 1000.0;  // 0.070 m
  constexpr double small_half_z = SMALL_ARMOR_HEIGHT / 2.0 / 1000.0;  // 0.0625 m
  constexpr double large_half_y = LARGE_ARMOR_WIDTH  / 2.0 / 1000.0;  // 0.1175 m
  constexpr double large_half_z = LARGE_ARMOR_HEIGHT / 2.0 / 1000.0;  // 0.0635 m

  // Build 4-point model for SMALL armor (clockwise from bottom-left in body frame)
  // X=0 because all points lie on the flat frontal surface of the plate.
  this->small_armor_points_.emplace_back(cv::Point3f(0,  small_half_y, -small_half_z));  // BL
  this->small_armor_points_.emplace_back(cv::Point3f(0,  small_half_y,  small_half_z));  // TL
  this->small_armor_points_.emplace_back(cv::Point3f(0, -small_half_y,  small_half_z));  // TR
  this->small_armor_points_.emplace_back(cv::Point3f(0, -small_half_y, -small_half_z));  // BR

  // Build 4-point model for LARGE armor (same ordering, different dimensions)
  this->large_armor_points_.emplace_back(cv::Point3f(0,  large_half_y, -large_half_z));  // BL
  this->large_armor_points_.emplace_back(cv::Point3f(0,  large_half_y,  large_half_z));  // TL
  this->large_armor_points_.emplace_back(cv::Point3f(0, -large_half_y,  large_half_z));  // TR
  this->large_armor_points_.emplace_back(cv::Point3f(0, -large_half_y, -large_half_z));  // BR
}

// ---------------------------------------------------------------------------
// solvePnP — recover 3D pose from 2D light-bar corners.
//
// Inputs:
//   armor.left_light  / right_light — 2D pixel coordinates of the two bars
//   armor.type                      — SMALL or LARGE (selects 3D model)
//
// Outputs (on success):
//   rvec — rotation vector (Rodrigues notation); convert with cv::Rodrigues()
//           to get the 3×3 rotation matrix from armor body → camera frame.
//   tvec — [x, y, z] translation of armor plate CENTER in camera frame [m].
//           In RealSense camera_color_optical_frame: x-right, y-down, z-forward.
//
// Algorithm — IPPE (cv::SOLVEPNP_IPPE):
//   Exploits planarity of the 4 points for a fast, accurate closed-form solution.
//   Returns the better of two candidate solutions (handles the "flip ambiguity"
//   that exists for any planar object viewed at oblique angles).
//   Preferred over ITERATIVE here because armor IS planar → IPPE is optimal.
//
// 2D → 3D correspondence (must match model point ordering above):
//   image[0] = left_light.bottom  ↔  model[0] = BL  (bottom-left)
//   image[1] = left_light.top     ↔  model[1] = TL  (top-left)
//   image[2] = right_light.top    ↔  model[2] = TR  (top-right)
//   image[3] = right_light.bottom ↔  model[3] = BR  (bottom-right)
//
// Returns false if OpenCV's solver fails (e.g. degenerate input geometry).
// ---------------------------------------------------------------------------
bool PnPSolver::solvePnP(const Armor & armor, cv::Mat & rvec, cv::Mat & tvec)
{
  // Collect 2D image points in the same clockwise order as the 3D model
  std::vector<cv::Point2f> image_armor_points;
  image_armor_points.emplace_back(armor.left_light.bottom);   // BL
  image_armor_points.emplace_back(armor.left_light.top);      // TL
  image_armor_points.emplace_back(armor.right_light.top);     // TR
  image_armor_points.emplace_back(armor.right_light.bottom);  // BR

  // Select 3D model matching the detected armor size
  auto object_points = (armor.type == ArmorType::SMALL) ? this->small_armor_points_ : this->large_armor_points_;

  // Run PnP with IPPE — well-suited for planar objects (no iterative seed needed)
  return cv::solvePnP(
    object_points, image_armor_points, this->camera_matrix_, this->dist_coeffs_, rvec, tvec,
    false,               // useExtrinsicGuess=false: start from scratch each frame
    cv::SOLVEPNP_IPPE);  // IPPE: fast, accurate, handles planar degeneracy
}

// ---------------------------------------------------------------------------
// calculateDistanceToCenter — distance (pixels) from a point to image center.
//
// The principal point (cx, cy) is stored in the camera intrinsic matrix at
// positions (0,2) and (1,2).  This distance is used by the tracker's init()
// function to prefer detections near the image center:
//   - Less lens distortion at center → more accurate PnP result
//   - Center detections are more likely to be the intended target
// ---------------------------------------------------------------------------
float PnPSolver::calculateDistanceToCenter(const cv::Point2f & image_point)
{
  float cx = this->camera_matrix_.at<double>(0, 2);  // principal point X [px]
  float cy = this->camera_matrix_.at<double>(1, 2);  // principal point Y [px]
  return cv::norm(image_point - cv::Point2f(cx, cy));
}

}  // namespace rm_auto_aim
