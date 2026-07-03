#ifndef AUTOAIM_V2__SOLVER_HPP_
#define AUTOAIM_V2__SOLVER_HPP_

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <optional>
#include <vector>

#include "autoaim_v2/types.hpp"

namespace aim
{

struct SolverParams
{
  // Small/large armor lightbar-corner geometry [m] (same as old pnp_solver).
  double small_w = 0.135, small_h = 0.056;
  double large_w = 0.225, large_h = 0.056;
  // 1v1 infantry vs infantry: the enemy only has small plates. Skipping the
  // large-model PnP saves a solve and removes misclassification noise.
  bool assume_small_armor = true;

  // Physical back-tilt of the plates (RoboMaster standard: 15 deg).
  double armor_pitch_deg = 15.0;

  // Reprojection yaw search: +/- range around the line-of-sight bearing.
  double search_range_deg = 80.0;
  double coarse_step_deg = 4.0;

  double max_reproj_err_px = 25.0;

  // Camera lever arm in the gimbal body frame [m] (x fwd, y left, z up).
  // 0,0,0 reproduces the old pipeline's "camera at the pivot" assumption.
  Eigen::Vector3d t_cam2gimbal{0, 0, 0};
  // Small mounting-misalignment trim (deg), applied camera->gimbal:
  // R_trim = Rz(trim_yaw)*Ry(trim_pitch)*Rx(trim_roll).
  double trim_yaw_deg = 0, trim_pitch_deg = 0, trim_roll_deg = 0;

  double gimbal_height = 0.42;  // pivot above ground [m]
};

// Geometry pipeline: pixels -> camera frame (PnP) -> world frame (gimbal
// angles at capture time) -> plate angle via reprojection grid search.
//
// The plate angle from raw PnP orientation is noisy (+-10..20 deg at range,
// bimodal near frontal). Fixing the position and searching the yaw of a
// 15deg-tilted plate model over +-80deg recovers it to ~1-2 deg (DESIGN.md §4).
class Solver
{
public:
  explicit Solver(const SolverParams & p);

  void set_intrinsics(double fx, double fy, double cx, double cy,
                      const std::vector<double> & dist = {});
  bool ready() const { return ready_; }

  // yaw/pitch: gimbal angles (internal convention) at the image capture time.
  // ego: robot position in the world frame (ego-motion integration).
  std::optional<ArmorWorld> solve(const ArmorPx & px, double yaw, double pitch,
                                  const Eigen::Vector2d & ego) const;

  // Exposed for tests.
  Eigen::Matrix3d R_world_cam(double yaw, double pitch) const;
  Eigen::Vector3d cam_origin_world(double yaw, double pitch,
                                   const Eigen::Vector2d & ego) const;
  double reproj_cost(const Eigen::Vector3d & plate_world, double theta_a,
                     bool is_large, double yaw, double pitch,
                     const Eigen::Vector2d & ego,
                     const std::array<cv::Point2f, 4> & meas) const;

private:
  double search_theta(const Eigen::Vector3d & plate_world, bool is_large,
                      double yaw, double pitch, const Eigen::Vector2d & ego,
                      const std::array<cv::Point2f, 4> & meas,
                      double bearing, double & sigma_out) const;

  SolverParams p_;
  bool ready_ = false;
  cv::Mat K_, dist_;
  Eigen::Matrix3d R_gc_;            // camera -> gimbal (axis conv + trim)
  std::vector<cv::Point3f> obj_small_, obj_large_;
};

}  // namespace aim

#endif  // AUTOAIM_V2__SOLVER_HPP_
