#ifndef AUTO_AIM_TARGETING__CONFIG_HPP_
#define AUTO_AIM_TARGETING__CONFIG_HPP_

#include <set>
#include <string>

#include "auto_aim_targeting/planning/ballistics_solver.hpp"
#include "auto_aim_targeting/planning_types.hpp"
#include "auto_aim_targeting/tracking/tracker.hpp"

namespace rm_auto_aim
{

struct MeasurementConfig
{
  double max_armor_distance = 10.0;
  std::set<std::string> target_classes{};
  double light_ratio = 0.85;
  double bbox_padding_y = 80.0;
  std::string target_frame = "odom";
};

struct TrackingRuntimeConfig
{
  TrackerConfig tracker{};
  double lost_time_thres = 0.3;
  int max_trackers = 5;
  double new_tracker_min_dist = 0.5;
  double process_noise_position = 5.0;
  double process_noise_yaw = 10.0;
  double process_noise_radius = 1e-6;
  double refresh_frequency = 30.0;
};

struct PoseConfig
{
  double yaw_sign = 1.0;
  double pitch_sign = 1.0;
  std::string pose_source = "none";
  double gimbal_height = 0.325;
  double camera_offset_x = 0.107;
  double camera_offset_z = 0.136;
  double self_orientation_timeout = 0.2;
  double pose_timeout = 0.15;
};

struct BallisticsConfig
{
  double bullet_speed = 25.0;
  double gravity = 9.8;
  double linear_drag_coeff = 0.01;
  double quadratic_drag_coeff = 0.01;
  bool use_quadratic_drag = false;
  double gimbal_pitch_max = 0.524;
  double gimbal_pitch_min = -0.524;
};

struct EngagementConfig
{
  double time_bias_initial = 0.08;
  double time_bias_alpha = 0.35;
  double gimbal_response_delay = 0.0;
  double min_fire_dist = 0.5;
  double max_fire_dist = 10.0;
  double angular_window = 0.09;
  double angular_window_ref_dist = 3.0;
  double max_measurement_age = 0.10;
  double max_gimbal_yaw_rate = 6.0;
  double max_gimbal_pitch_rate = 4.0;
  double indirect_vyaw_threshold = 3.0;
  double indirect_timing_tolerance = 0.02;
  int indirect_max_candidates = 8;
  double oblique_exponent = 2.0;
  double latency_gate_sigma = 2.5;
  int latency_warmup_samples = 5;
  CostWeights cost_weights{};
};

struct VisualizationConfig
{
  double cmd_smooth_alpha = 0.4;
  double max_cmd_pitch_angle_deg = 30.0;
  double max_cmd_yaw_angle_deg = 180.0;
};

struct TargetingConfig
{
  MeasurementConfig measurement{};
  TrackingRuntimeConfig tracking{};
  PoseConfig pose{};
  BallisticsConfig ballistics{};
  EngagementConfig engagement{};
  VisualizationConfig visualization{};

  BallisticsParams ballisticsParams() const
  {
    BallisticsParams params;
    params.gravity = this->ballistics.gravity;
    params.linear_drag_coeff = this->ballistics.linear_drag_coeff;
    params.quadratic_drag_coeff = this->ballistics.quadratic_drag_coeff;
    params.use_quadratic_drag = this->ballistics.use_quadratic_drag;
    params.pitch_min = this->ballistics.gimbal_pitch_min;
    params.pitch_max = this->ballistics.gimbal_pitch_max;
    return params;
  }
};

}  // namespace rm_auto_aim

#endif  // AUTO_AIM_TARGETING__CONFIG_HPP_
