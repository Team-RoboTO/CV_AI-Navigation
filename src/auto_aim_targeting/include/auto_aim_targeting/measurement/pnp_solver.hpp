#ifndef AUTO_AIM_TARGETING__MEASUREMENT__PNP_SOLVER_HPP_
#define AUTO_AIM_TARGETING__MEASUREMENT__PNP_SOLVER_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <opencv2/core.hpp>

// STD
#include <array>
#include <vector>

#include "auto_aim_targeting/measurement/armor_model.hpp"

namespace rm_auto_aim
{

// ---------------------------------------------------------------------------
// PnPSolver — recovers 3D pose from a single armor plate detection.
//
// Background — Perspective-n-Point (PnP):
//   PnP solves for the rotation R and translation t of an object in 3D given:
//     (1) Known 3D model points   — the actual geometry of the object
//     (2) Observed 2D image points — where those 3D points appear in the camera
//     (3) Camera intrinsics        — focal length, principal point, distortion
//
//   The result (rvec, tvec) gives us:
//     tvec = [x, y, z]     position of the armor plate CENTER in camera frame [m]
//     rvec = Rodrigues vec  rotation from object to camera frame
//
//   From rvec we extract the yaw (rotation around the vertical axis), which
//   tells us which direction the armor plate is facing.
//
// Algorithm — IPPE (Iterative Pose from Planar Correspondences, cv::SOLVEPNP_IPPE):
//   IPPE is specialized for PLANAR objects (all 4 points co-planar).
//   It computes two closed-form solutions analytically, then refines the best
//   one.  Compared to the iterative general solver (SOLVEPNP_ITERATIVE):
//     - Much faster (one linear pass vs iterative refinement)
//     - Returns two candidate solutions (planar ambiguity)
//     - More accurate for our problem since the armor IS planar
//
// Coordinate frame (armor body frame, OpenCV convention):
//   X-axis: into the armor (depth direction)
//   Y-axis: to the left of the armor face (horizontal)
//   Z-axis: upward
//   Origin: center of the armor plate
//
// The 4 model points follow a clockwise ordering starting from bottom-left:
//   P0: bottom-left   P1: top-left   P2: top-right   P3: bottom-right
//
// These are matched to the 4 image points:
//   left_light.bottom → left_light.top → right_light.top → right_light.bottom
//
// After PnP, the resulting rvec/tvec is in camera_color_optical_frame.
// The ArmorTrackerNode transforms this to the odom (world) frame via TF2.
// ---------------------------------------------------------------------------
class PnPSolver
{
public:
  // Construct with camera intrinsics from the /camera_info ROS topic.
  // camera_matrix: row-major 3×3 intrinsic matrix [fx 0 cx; 0 fy cy; 0 0 1]
  // distortion_coefficients: lens distortion coefficients (k1,k2,p1,p2[,k3...])
  PnPSolver(
    const std::array<double, 9> & camera_matrix,
    const std::vector<double> & distortion_coefficients);

  // Solve PnP for a detected armor plate.
  // Input:  armor — 2D light-bar corner points + plate type
  // Output: rvec  — rotation vector (Rodrigues) from armor body to camera frame
  //         tvec  — translation vector: armor center position in camera frame [m]
  // Returns true on success, false if OpenCV solvePnP fails.
  bool solvePnP(const Armor & armor, cv::Mat & rvec, cv::Mat & tvec);

  // Returns the Euclidean distance (pixels) from an image point to the
  // camera principal point (cx, cy).  Used in tracker init to prefer
  // detections near image center (better PnP accuracy, less lens distortion).
  float calculateDistanceToCenter(const cv::Point2f & image_point);

private:
  cv::Mat camera_matrix_;  // 3×3 intrinsic matrix (double precision, CV_64F)
  cv::Mat dist_coeffs_;    // Distortion coefficients (1×n, double precision)

  // Physical dimensions of RoboMaster armor plates.
  // Unit: mm — converted to metres during construction (÷ 1000).
  // SMALL = AM02 plate (standard/sentry robots)
  // LARGE = AM12 plate (hero robot)
  static constexpr float SMALL_ARMOR_WIDTH  = 140;   // horizontal span of light bars [mm]
  static constexpr float SMALL_ARMOR_HEIGHT = 125;   // vertical span of light bars [mm]
  static constexpr float LARGE_ARMOR_WIDTH  = 235;   // horizontal span of light bars [mm]
  static constexpr float LARGE_ARMOR_HEIGHT = 127;   // vertical span of light bars [mm]

  // 3D model points for each armor type (in the armor body frame, metres).
  // Four points, clockwise from bottom-left: BL, TL, TR, BR.
  std::vector<cv::Point3f> small_armor_points_;
  std::vector<cv::Point3f> large_armor_points_;
};

}  // namespace rm_auto_aim

#endif  // AUTO_AIM_TARGETING__MEASUREMENT__PNP_SOLVER_HPP_
