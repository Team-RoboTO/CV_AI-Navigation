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

  // Which measurement model produced this PnP. Mirrors MEAS_SOURCE_* in
  // msg/AutoAimDebug.msg. Only KEYPOINT is currently in use; the enum exists
  // so a future bbox fallback would be visible in /auto_aim/debug without
  // changing the schema again.
  enum class Source : uint8_t {
    NONE     = 0,
    KEYPOINT = 1,
  };

  bool   ok            = false;  // solvePnP succeeded AND every health gate passed
  bool   solver_ok     = false;  // solvePnP itself returned a valid pose
  bool   is_large      = false;  // true if the large armor model fit better
  Source source        = Source::NONE;
  RejectReason reject_reason = REASON_OK;
  cv::Mat rvec;
  cv::Mat tvec;
  double reproj_err     = 0.0;   // mean pixel error of the chosen model
  double reproj_err_norm = 0.0;  // reproj_err / keypoint AABB diagonal
  double small_reproj_err = 0.0;
  double large_reproj_err = 0.0;
  double size_margin = 0.0;      // abs(small-large), pixels
  // Image corners that PnP was fed (in TL,TR,BR,BL order).
  // x0 y0 x1 y1 x2 y2 x3 y3
  std::array<float, 8> image_points{};
};

// Converts four YOLO-pose armor corners into a 3D armor pose. Tries the
// small and large armor models, keeps the lower reprojection error.
//
// Keypoint order is fixed by the ArmorKeypoint.msg contract:
//   [0]=TL, [1]=TR, [2]=BR, [3]=BL  (image pixels, x right, y down).
class PnPSolver
{
public:
  // Real armor plate dimensions [mm], from the RoboMaster rules.
  static constexpr double SMALL_W = 140.0, SMALL_H = 125.0;
  static constexpr double LARGE_W = 235.0, LARGE_H = 127.0;

  PnPSolver() = default;

  enum class ArmorSizeMode : uint8_t {
    AUTO = 0,
    SMALL = 1,
    LARGE = 2,
  };

  // Initialize from camera intrinsics K (row-major 3x3) and distortion d.
  void init(const std::array<double, 9> & K, const std::vector<double> & dist);

  // Solve PnP from four YOLO-pose armor corners in image pixels.
  //
  // The keypoint order must be TL, TR, BR, BL.
  PnPResult solveKeypoints(
    const std::array<cv::Point2f, 4> & corners_tl_tr_br_bl,
    double max_reproj_err = 15.0,
    bool refine_lm = false,
    double max_reproj_err_norm = 0.0,
    double min_depth_m = 0.0,
    double max_depth_m = 1.0e6,
    ArmorSizeMode force_size = ArmorSizeMode::AUTO) const;

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
  bool ready_ = false;
};

}  // namespace auto_aim
#endif
