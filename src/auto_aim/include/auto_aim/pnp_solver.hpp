#ifndef AUTO_AIM__PNP_SOLVER_HPP_
#define AUTO_AIM__PNP_SOLVER_HPP_

#include <opencv2/core.hpp>
#include <array>
#include <vector>

namespace auto_aim
{

// Result of one PnP attempt. The solver fills the geometry fields whether
// or not the solve succeeded; consumers should always check `ok`.
struct PnPResult
{
  // Reason the result was rejected, or REASON_OK on success. Mirrors the
  // PNP_* constants in msg/AutoAimDebug.msg so /auto_aim/debug consumers
  // can see exactly which gate triggered.
  enum RejectReason : uint8_t {
    REASON_OK              = 0,
    REASON_DEGEN_BBOX      = 1,
    REASON_NONFINITE       = 2,
    REASON_BAD_LIGHT_RATIO = 3,
    REASON_SOLVER_FAIL     = 4,
    REASON_REPROJ_ABS      = 5,
    REASON_REPROJ_NORM     = 6,
    REASON_DEPTH_TOO_CLOSE = 7,
    REASON_DEPTH_TOO_FAR   = 8,
  };

  bool   ok            = false;  // solvePnP succeeded AND every health gate passed
  bool   solver_ok     = false;  // solvePnP itself returned a valid pose
  bool   is_large      = false;  // true if the large armor model fit better
  RejectReason reject_reason = REASON_OK;
  cv::Mat rvec;
  cv::Mat tvec;
  double reproj_err     = 0.0;   // mean pixel error of the chosen model
  double reproj_err_norm = 0.0;  // reproj_err / bbox diagonal
  // Synthetic image corners that PnP was fed (light-bar inscribed rectangle).
  // x0 y0 x1 y1 x2 y2 x3 y3
  std::array<float, 8> image_points{};
};

// Converts 2D YOLO bounding boxes into 3D armor poses with PnP. Tries the
// small and large armor models, keeps the lower reprojection error.
//
// This is bbox-PnP: the four image points are the corners of an inscribed
// rectangle scaled by `light_ratio`. It is approximate by construction; the
// reprojection error reflects how well the rectangle fits the synthetic
// corners, not how well the rectangle fits the actual armor.
class PnPSolver
{
public:
  // Real armor plate dimensions [mm], from the RoboMaster rules.
  static constexpr double SMALL_W = 140.0, SMALL_H = 125.0;
  static constexpr double LARGE_W = 235.0, LARGE_H = 127.0;

  PnPSolver() = default;

  // Initialize from camera intrinsics K (row-major 3x3) and distortion d.
  void init(const std::array<double, 9> & K, const std::vector<double> & dist);

  // Solve PnP from a YOLO bbox.
  //
  //   cx, cy            - bbox center [px]
  //   w, h              - bbox size [px]
  //   light_ratio       - shrink factor applied to both width and height to
  //                       approximate the light-bar inscribed rectangle
  //   max_reproj_err    - reject the result if the mean pixel error exceeds
  //                       this threshold (after picking the better of small/
  //                       large armor models)
  //   refine_lm         - run solvePnPRefineLM after IPPE for an extra LM
  //                       iteration. Default false; P4 enables behind a flag.
  //   require_min_size  - bbox must have width and height >= this many pixels
  //                       to be considered (P4 hardening)
  //   max_reproj_err_norm - reject when (reproj_err / bbox_diag) exceeds
  //                       this. Pass 0.0 to disable the normalized gate
  //                       (only the absolute max_reproj_err applies).
  //   min_depth_m / max_depth_m - reject when tvec.z is outside this band.
  //                       Pass 0.0 (min) and a large value (max) to disable.
  PnPResult solve(double cx, double cy, double w, double h, double light_ratio,
                  double max_reproj_err = 10.0,
                  bool refine_lm = false,
                  double require_min_size = 4.0,
                  double max_reproj_err_norm = 0.0,
                  double min_depth_m = 0.0,
                  double max_depth_m = 1.0e6) const;

  // Solve PnP from four YOLO-pose armor corners in image pixels.
  //
  // The keypoint order must be TL, TR, BR, BL. This is the preferred path for
  // YOLOv26-pose because it uses the actual learned armor corners instead of
  // reconstructing a synthetic rectangle from the bbox.
  PnPResult solveKeypoints(
    const std::array<cv::Point2f, 4> & corners_tl_tr_br_bl,
    double max_reproj_err = 15.0,
    bool refine_lm = false,
    double max_reproj_err_norm = 0.0,
    double min_depth_m = 0.0,
    double max_depth_m = 1.0e6) const;

  bool ready() const { return ready_; }

private:
  double solveWithModel(
    const std::vector<cv::Point3f> & obj_pts,
    const std::vector<cv::Point2f> & img_pts,
    cv::Mat & rvec,
    cv::Mat & tvec,
    bool refine_lm) const;

  cv::Mat K_, dist_;
  std::vector<cv::Point3f> pts_small_, pts_large_;
  std::vector<cv::Point3f> pts_small_tl_tr_br_bl_, pts_large_tl_tr_br_bl_;
  bool ready_ = false;
};

}  // namespace auto_aim
#endif
