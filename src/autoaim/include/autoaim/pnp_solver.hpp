#ifndef AUTOAIM__PNP_SOLVER_HPP_
#define AUTOAIM__PNP_SOLVER_HPP_

#include <opencv2/core.hpp>
#include <vector>
#include <array>

namespace autoaim
{

/// Converts 2D armor observations to 3D poses via Perspective-n-Point.
/// Supports both legacy YOLO bbox corners and YOLO-pose keypoint corners.
struct PnPSolver
{
  // Real armor plate dimensions [mm] — from RoboMaster rules.
  // IMPORTANT: These must match the physical rectangle represented by your labels.
  // If you annotated only the luminous sticker/number region, change these values.
  static constexpr double SMALL_W = 140.0, SMALL_H = 125.0;
  static constexpr double LARGE_W = 235.0, LARGE_H = 127.0;

  PnPSolver() = default;

  /// Initialize with camera intrinsics (3x3 K matrix flat, distortion coeffs).
  void init(const std::array<double, 9> & K, const std::vector<double> & dist);

  /// Legacy solve from a YOLO bounding box.
  /// cx,cy = bbox center [px], w,h = bbox size [px], lr = light_ratio shrink factor.
  bool solve(double cx, double cy, double w, double h, double lr,
             cv::Mat & rvec, cv::Mat & tvec, bool & is_large,
             double max_reproj_error = 25.0) const;

  /// Preferred solve from 4 YOLO-pose keypoints in image pixels.
  /// Keypoint order MUST be TL, TR, BR, BL.
  ///
  /// rvec_alt / has_alt (optional): a single armor plate is planar, so IPPE
  /// returns TWO pose solutions whose ROTATION (yaw) differs while the position
  /// is essentially the same. When rvec_alt is non-null it is filled with the
  /// SECOND solution's rotation and has_alt is set true (false when there is
  /// only one solution, e.g. the ITERATIVE fallback). The caller uses rvec_alt
  /// only to recover the alternate yaw for temporal disambiguation; tvec (the
  /// reliable position) is always taken from the best solution.
  bool solveKeypoints(const std::array<cv::Point2f, 4> & corners_tl_tr_br_bl,
                      cv::Mat & rvec, cv::Mat & tvec, bool & is_large,
                      double * reproj_error = nullptr,
                      double max_reproj_error = 25.0,
                      cv::Mat * rvec_alt = nullptr,
                      bool * has_alt = nullptr) const;

  bool ready() const { return ready_; }

private:
  /// Solve one plate model. Fills rvec/tvec with the best (lowest-reprojection)
  /// solution and, when rvec_alt is non-null, the second IPPE solution's
  /// rotation (has_alt set accordingly). Returns mean reprojection error [px].
  double solveWithModel(const std::vector<cv::Point3f> & obj_pts,
                        const std::vector<cv::Point2f> & img_pts,
                        cv::Mat & rvec, cv::Mat & tvec,
                        cv::Mat * rvec_alt = nullptr,
                        bool * has_alt = nullptr) const;

  cv::Mat K_, dist_;
  // Object points in TL,TR,BR,BL order for direct YOLO-pose corner use.
  std::vector<cv::Point3f> pts_small_tl_tr_br_bl_;
  std::vector<cv::Point3f> pts_large_tl_tr_br_bl_;
  bool ready_ = false;
};

}  // namespace autoaim
#endif
