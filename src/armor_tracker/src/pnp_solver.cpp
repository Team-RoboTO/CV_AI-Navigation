#include "armor_tracker/pnp_solver.hpp"

#include <opencv2/calib3d.hpp>

#include <limits>
#include <vector>

namespace rm_auto_aim
{

PnPSolver::PnPSolver(
  const std::array<double, 9> & camera_matrix,
  const std::vector<double> & distortion_coefficients)
: camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone()),
  dist_coeffs_(cv::Mat(
      1, static_cast<int>(distortion_coefficients.size()), CV_64F,
      const_cast<double *>(distortion_coefficients.data())).clone())
{
  // Convert the official armor dimensions from millimetres to metres once.
  constexpr double small_half_y = SMALL_ARMOR_WIDTH / 2.0 / 1000.0;
  constexpr double small_half_z = SMALL_ARMOR_HEIGHT / 2.0 / 1000.0;
  constexpr double large_half_y = LARGE_ARMOR_WIDTH / 2.0 / 1000.0;
  constexpr double large_half_z = LARGE_ARMOR_HEIGHT / 2.0 / 1000.0;

  // 3D point ordering:
  //   bottom-left -> top-left -> top-right -> bottom-right
  // All points lie on the armor plane, so X = 0 for every point.
  small_armor_points_.emplace_back(0.0F, static_cast<float>(small_half_y), static_cast<float>(-small_half_z));
  small_armor_points_.emplace_back(0.0F, static_cast<float>(small_half_y), static_cast<float>( small_half_z));
  small_armor_points_.emplace_back(0.0F, static_cast<float>(-small_half_y), static_cast<float>( small_half_z));
  small_armor_points_.emplace_back(0.0F, static_cast<float>(-small_half_y), static_cast<float>(-small_half_z));

  large_armor_points_.emplace_back(0.0F, static_cast<float>(large_half_y), static_cast<float>(-large_half_z));
  large_armor_points_.emplace_back(0.0F, static_cast<float>(large_half_y), static_cast<float>( large_half_z));
  large_armor_points_.emplace_back(0.0F, static_cast<float>(-large_half_y), static_cast<float>( large_half_z));
  large_armor_points_.emplace_back(0.0F, static_cast<float>(-large_half_y), static_cast<float>(-large_half_z));
}

std::vector<cv::Point2f> PnPSolver::getImageArmorPoints(const Armor & armor) const
{
  return {
    armor.left_light.bottom,
    armor.left_light.top,
    armor.right_light.top,
    armor.right_light.bottom,
  };
}

const std::vector<cv::Point3f> & PnPSolver::getObjectPoints(ArmorType type) const
{
  return (type == ArmorType::SMALL) ? small_armor_points_ : large_armor_points_;
}

bool PnPSolver::solveWithModel(
  const std::vector<cv::Point3f> & object_points,
  const std::vector<cv::Point2f> & image_points,
  cv::Mat & rvec,
  cv::Mat & tvec) const
{
  return cv::solvePnP(
    object_points,
    image_points,
    camera_matrix_,
    dist_coeffs_,
    rvec,
    tvec,
    false,
    cv::SOLVEPNP_IPPE);
}

bool PnPSolver::solvePnP(const Armor & armor, cv::Mat & rvec, cv::Mat & tvec) const
{
  const auto image_points = getImageArmorPoints(armor);
  const auto & object_points = getObjectPoints(armor.type);
  return solveWithModel(object_points, image_points, rvec, tvec);
}

double PnPSolver::computeReprojectionError(
  const std::vector<cv::Point3f> & object_points,
  const std::vector<cv::Point2f> & image_points,
  const cv::Mat & rvec,
  const cv::Mat & tvec) const
{
  std::vector<cv::Point2f> projected_points;
  cv::projectPoints(object_points, rvec, tvec, camera_matrix_, dist_coeffs_, projected_points);

  if (projected_points.size() != image_points.size() || projected_points.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  double sum = 0.0;
  for (size_t i = 0; i < image_points.size(); ++i) {
    sum += cv::norm(projected_points[i] - image_points[i]);
  }
  return sum / static_cast<double>(image_points.size());
}

bool PnPSolver::solveBestArmorType(
  const Armor & armor,
  ArmorType & best_type,
  cv::Mat & rvec,
  cv::Mat & tvec,
  double * reprojection_error) const
{
  const auto image_points = getImageArmorPoints(armor);

  cv::Mat rvec_small, tvec_small;
  cv::Mat rvec_large, tvec_large;

  const bool small_ok = solveWithModel(small_armor_points_, image_points, rvec_small, tvec_small);
  const bool large_ok = solveWithModel(large_armor_points_, image_points, rvec_large, tvec_large);

  if (!small_ok && !large_ok) {
    return false;
  }

  double small_error = std::numeric_limits<double>::infinity();
  double large_error = std::numeric_limits<double>::infinity();

  if (small_ok) {
    small_error = computeReprojectionError(small_armor_points_, image_points, rvec_small, tvec_small);
  }
  if (large_ok) {
    large_error = computeReprojectionError(large_armor_points_, image_points, rvec_large, tvec_large);
  }

  if (small_error <= large_error) {
    best_type = ArmorType::SMALL;
    rvec = rvec_small;
    tvec = tvec_small;
    if (reprojection_error != nullptr) {
      *reprojection_error = small_error;
    }
  } else {
    best_type = ArmorType::LARGE;
    rvec = rvec_large;
    tvec = tvec_large;
    if (reprojection_error != nullptr) {
      *reprojection_error = large_error;
    }
  }

  return true;
}

float PnPSolver::calculateDistanceToCenter(const cv::Point2f & image_point) const
{
  const float cx = static_cast<float>(camera_matrix_.at<double>(0, 2));
  const float cy = static_cast<float>(camera_matrix_.at<double>(1, 2));
  return cv::norm(image_point - cv::Point2f(cx, cy));
}

}  // namespace rm_auto_aim
