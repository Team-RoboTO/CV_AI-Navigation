#ifndef AUTOAIM_V2__TYPES_HPP_
#define AUTOAIM_V2__TYPES_HPP_

#include <Eigen/Dense>
#include <array>
#include <chrono>
#include <cstdint>
#include <opencv2/core.hpp>

namespace aim
{

using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

inline double seconds(TimePoint a, TimePoint b)  // a - b
{
  return std::chrono::duration<double>(a - b).count();
}

// ── Detector output: one armor plate in pixel space ────────────────────────
struct ArmorPx
{
  int class_id = 0;                       // 0 blue, 1 grey, 2 red
  float confidence = 0.f;
  std::array<cv::Point2f, 4> corners{};   // TL, TR, BR, BL (lightbar corners)
};

struct DetectionFrame
{
  TimePoint t_capture{};                  // ZED capture time, steady clock
  std::vector<ArmorPx> armors;
};

// ── Solver output: one plate located and oriented in the world frame ───────
// World frame = odom-like: origin at gimbal pivot ground projection at boot,
// x forward at gimbal yaw 0, z up. "Plate angle" theta_a is defined so that
//   car_center = plate_pos + r * [cos(theta_a), sin(theta_a), 0]
// i.e. for a plate squarely facing us, theta_a equals the bearing from the
// camera to the plate (same convention as sp_vision and the old tracker).
struct ArmorWorld
{
  int class_id = 0;
  float confidence = 0.f;
  bool is_large = false;
  Eigen::Vector3d pos_world{0, 0, 0};   // plate center [m]
  Eigen::Vector3d pos_cam{0, 0, 0};     // plate center in camera frame [m]
  double theta_a = 0.0;                 // plate angle [rad], convention above
  double theta_sigma = 0.3;             // 1-sigma of theta_a from the search
  double reproj_err = 0.0;              // px
  double yaw = 0, pitch = 0, dist = 0;  // spherical coords of pos rel. camera
};

// ── Gimbal / robot status sample (from the 1 kHz serial stream) ────────────
struct GimbalSample
{
  TimePoint t{};
  double yaw = 0;         // internal convention [rad] (sign-corrected)
  double pitch = 0;       // internal convention [rad] (sign-corrected)
  double yaw_raw = 0;     // exactly as received from the micro
  double pitch_raw = 0;   // exactly as received from the micro
};

struct RobotStatus
{
  double vx = 0, vy = 0;        // chassis velocity feedback [m/s]
  int color = -1;               // 0 we-are-RED, 1 we-are-BLUE
  int game_progress = 0;        // 4 = in match
  double hp = 0;
  double reserved9 = 0;         // fw-defined (heat / bullet speed, see docs)
  bool valid = false;
};

// ── Command to the micro ────────────────────────────────────────────────────
struct GimbalCommand
{
  double yaw_micro = 0;     // absolute target, micro yaw frame (unbounded)
  double pitch_micro = 0;   // absolute target, micro pitch convention
  bool shoot = false;       // immediate shoot flag (TRACK/SWEEP full-auto)
  // TIMED regime: precise-schedule a single shot at this steady-clock time.
  // The sender sleeps until shoot_at and emits a one-packet shoot pulse —
  // sub-ms accuracy instead of frame-quantized (~±6 ms = ~38 mm @300 RPM).
  bool shoot_scheduled = false;
  TimePoint shoot_at{};
  double ff_yaw_rate = 0;   // optional feedforward (TX[4]) [rad/s]
  double ff_pitch_rate = 0; // optional feedforward (TX[5]) [rad/s]
  bool valid = false;       // false -> hold current angles, never shoot
};

}  // namespace aim

#endif  // AUTOAIM_V2__TYPES_HPP_
