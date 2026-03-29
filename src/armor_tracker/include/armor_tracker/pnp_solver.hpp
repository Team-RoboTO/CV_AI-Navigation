#ifndef ARMOR_TRACKER__PNP_SOLVER_HPP_
#define ARMOR_TRACKER__PNP_SOLVER_HPP_

#include <opencv2/core.hpp>

#include <array>
#include <vector>

#include "armor_tracker/armor.hpp"

namespace rm_auto_aim
{

// ---------------------------------------------------------------------------
// PnPSolver — recover the 3D pose of an armor plate from four 2D image points.
//
// Background: Perspective-n-Point (PnP)
//   PnP solves for an object's 3D pose if we know:
//     1. The 3D coordinates of some points on the object.
//     2. The 2D image coordinates where those points appear.
//     3. The camera intrinsics and distortion coefficients.
//
//   The output is:
//     rvec — a Rodrigues rotation vector describing the orientation.
//     tvec — a translation vector giving the object position in camera frame.
//
// Why this class exists in our project
//   The detector does NOT give us perfect light-bar keypoints yet.  It gives an
//   axis-aligned bounding box, and the tracker approximates the four light-bar
//   endpoints from that box.  This approximation is imperfect, so we need a PnP
//   stage that is both robust and honest about model mismatch.
//
// Planar solver choice: IPPE
//   The armor is a planar target.  IPPE is specialized for planar objects and is
//   therefore a very good fit here.
//
// Important robustness improvement
//   Because a slanted LARGE armor can look narrow in an axis-aligned 2D box, we
//   do NOT rely only on the bbox aspect ratio anymore.
//
//   Instead, solveBestArmorType():
//     - solves PnP with the SMALL model,
//     - solves PnP with the LARGE model,
//     - computes reprojection error for both,
//     - and keeps the hypothesis with the lower error.
//
//   Reprojection error = average pixel disagreement between:
//     - the 2D points we observed, and
//     - the 2D points predicted by the recovered 3D pose.
// ---------------------------------------------------------------------------
class PnPSolver
{
public:
  PnPSolver(
    const std::array<double, 9> & camera_matrix,
    const std::vector<double> & distortion_coefficients);

  // Solve PnP using the armor.type already stored in the Armor object.
  // This legacy method is kept for compatibility with old code paths.
  bool solvePnP(const Armor & armor, cv::Mat & rvec, cv::Mat & tvec) const;

  // Solve both physical armor hypotheses and keep the one with the lower
  // reprojection error.
  //
  // Outputs:
  //   best_type          — selected physical armor size
  //   rvec, tvec         — pose for the winning hypothesis
  //   reprojection_error — optional average pixel error of the winning fit
  bool solveBestArmorType(
    const Armor & armor,
    ArmorType & best_type,
    cv::Mat & rvec,
    cv::Mat & tvec,
    double * reprojection_error = nullptr) const;

  float calculateDistanceToCenter(const cv::Point2f & image_point) const;

private:
  std::vector<cv::Point2f> getImageArmorPoints(const Armor & armor) const;
  const std::vector<cv::Point3f> & getObjectPoints(ArmorType type) const;

  double computeReprojectionError(
    const std::vector<cv::Point3f> & object_points,
    const std::vector<cv::Point2f> & image_points,
    const cv::Mat & rvec,
    const cv::Mat & tvec) const;

  bool solveWithModel(
    const std::vector<cv::Point3f> & object_points,
    const std::vector<cv::Point2f> & image_points,
    cv::Mat & rvec,
    cv::Mat & tvec) const;

  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;

  static constexpr float SMALL_ARMOR_WIDTH = 140.0F;
  static constexpr float SMALL_ARMOR_HEIGHT = 125.0F;
  static constexpr float LARGE_ARMOR_WIDTH = 235.0F;
  static constexpr float LARGE_ARMOR_HEIGHT = 127.0F;

  std::vector<cv::Point3f> small_armor_points_;
  std::vector<cv::Point3f> large_armor_points_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_TRACKER__PNP_SOLVER_HPP_
