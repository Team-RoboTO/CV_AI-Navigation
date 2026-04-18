#ifndef AUTO_AIM_TARGETING__PLANNING_TYPES_HPP_
#define AUTO_AIM_TARGETING__PLANNING_TYPES_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <cmath>
#include <limits>
#include <string>

#include "auto_aim_targeting/tracking/tracker.hpp"

namespace rm_auto_aim
{

enum class AimMode
{
  VISIBLE_DIRECT,
  PREDICTED_DIRECT,
  INDIRECT
};

struct PredictedCenter
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct FaceGeometry
{
  int face_index = 0;
  bool alternate_pair = false;
  double radius = 0.0;
  double dz_offset = 0.0;
  double face_yaw = 0.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double range = 0.0;
  double ground_dist = 0.0;
  double bearing = 0.0;
  double visibility = 0.0;   // cos(oblique), clamped to [0, 1]
};

struct BallisticSolution
{
  double pitch = 0.0;
  double flight_time = 0.0;
  // Reachable means the required pitch is within the physical limits of the robot's gimbal.
  bool reachable = false;
  // Valid means the physics equation successfully produced a real number (i.e. target is not completely out of theoretical range).
  bool valid = false;
};

// ShotPlan represents a complete evaluation of what it would take to hit a specific armor face.
// It is the core object passed around to determine the "best" shot.
struct ShotPlan
{
  int tracker_id = -1;
  int face_index = -1;
  AimMode mode = AimMode::PREDICTED_DIRECT;
  // Indirect mode: firing ahead to hit a spinning plate as it rotates into view.
  bool indirect = false;
  // TRUE if equations can mathematically solve the trajectory (ignoring physical hardware bounds).
  bool ballistic_valid = false;
  // TRUE if the solved pitch is mechanically possible for the turret to reach.
  bool reachable = false;
  // TRUE if the tracking measurement is older than our allowed latency threshold.
  bool measurement_stale = true;
  bool tracking_valid = false;
  bool temp_lost = false;

  double range = std::numeric_limits<double>::infinity();
  double predict_time = 0.0;      // time horizon from now until bullet impact
  double flight_time = 0.0;
  double absolute_yaw = 0.0;      // target bearing in Odometry frame
  double absolute_pitch = 0.0;    // required tilt in Odometry frame
  double relative_yaw = 0.0;
  double relative_pitch = 0.0;
  // In indirect mode: difference between when the armor aligns and when the bullet arrives.
  double timing_residual = 0.0;
  // How much spare time/angle we have inside our "angular_window" to guarantee a hit. Positive means good to fire!
  double fire_window_margin = -std::numeric_limits<double>::infinity();
  double sigma_angular = std::numeric_limits<double>::infinity();
  // 1.0 = perfectly facing us, 0.0 = completely sideways (90 deg).
  double visibility = 0.0;
  // Real age of the measurement, accounting for hardware capture and transport delays.
  double measurement_age = std::numeric_limits<double>::infinity();
  double score = std::numeric_limits<double>::infinity();

  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct CostWeights
{
  double range = 0.30;
  double flight_time = 0.15;
  double uncertainty = 0.35;
  double slew = 0.25;
  double switch_target = 0.35;
  double switch_face = 0.20;
  double staleness = 0.25;
  double temp_lost = 0.70;
  double low_visibility = 0.35;
  double negative_margin = 1.00;
};

struct AimDecision
{
  TrackSnapshot track{};
  ShotPlan plan{};
};

struct PlanningContext
{
  double time_bias = 0.0;
  double gimbal_response_delay = 0.0;
  double gimbal_height = 0.0;
  double bullet_speed = 25.0;
  double angular_window = 0.09;
  double angular_window_ref_dist = 3.0;
  double min_fire_dist = 0.5;
  double max_fire_dist = 10.0;
  double max_measurement_age = 0.10;
  double current_yaw = 0.0;
  double current_pitch = 0.0;
  double max_gimbal_yaw_rate = 6.0;
  double max_gimbal_pitch_rate = 4.0;
  double indirect_vyaw_threshold = 3.0;
  double indirect_timing_tolerance = 0.02;
  int indirect_max_candidates = 8;
  double visibility_exponent = 2.0;
  int previous_tracker_id = -1;
  int previous_face_index = -1;
  bool previous_indirect_mode = false;
  double transport_delay = 0.0;
};

inline PredictedCenter predictCenter(
  const TrackSnapshot & target,
  double dt)
{
  PredictedCenter c;
  c.x = target.position.x + target.velocity.x * dt + 0.5 * target.acceleration.x * dt * dt;
  c.y = target.position.y + target.velocity.y * dt + 0.5 * target.acceleration.y * dt * dt;
  c.z = target.position.z + target.velocity.z * dt + 0.5 * target.acceleration.z * dt * dt;
  return c;
}

inline double crossRangeSigma(
  const TrackSnapshot & target,
  double bearing)
{
  const double sin_b = std::sin(bearing);
  const double cos_b = std::cos(bearing);
  return std::sqrt(std::max(
    target.position_variance_x * sin_b * sin_b +
    target.position_variance_y * cos_b * cos_b, 1e-10));
}

}  // namespace rm_auto_aim

#endif  // AUTO_AIM_TARGETING__PLANNING_TYPES_HPP_
