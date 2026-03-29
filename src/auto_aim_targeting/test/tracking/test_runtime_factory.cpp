#include <gtest/gtest.h>

#include "auto_aim_targeting/tracking/tracker_runtime_factory.hpp"

namespace rm_auto_aim
{
namespace
{

TrackingRuntimeConfig makeTrackingConfig()
{
  TrackingRuntimeConfig config;
  auto & tracker = config.tracker;
  tracker.max_match_distance = 0.15;
  tracker.max_track_range = 6.0;
  tracker.tracking_threshold = 5;
  tracker.initial_close_pair_orbit_radius = 0.22;
  tracker.initial_far_pair_orbit_radius = 0.30;
  tracker.radius_filter_process_noise = 3.3e-8;
  tracker.radius_filter_measurement_noise = 0.0004;
  tracker.radius_filter_initial_covariance = 0.0064;
  tracker.radius_adaptation_max_range = 4.0;
  tracker.height_offset_smoothing_factor = 0.05;
  tracker.radius_yaw_uncertainty_scale = 50.0;
  tracker.max_yaw_rate = 15.0;
  tracker.mahalanobis_match_gate = 13.3;
  tracker.mahalanobis_jump_gate = 20.0;
  tracker.max_yaw_oblique_deg = 65.0;
  tracker.use_secondary_face_fusion = true;
  tracker.secondary_noise_multiplier = 2.0;
  tracker.secondary_mahalanobis_gate = 13.3;
  tracker.position_noise_base = 0.04;
  tracker.position_noise_slope = 0.03;
  tracker.yaw_noise_base = 0.05;
  tracker.yaw_noise_slope = 0.002;
  tracker.yaw_obliquity_exponent = 4.0;
  tracker.position_decay_baseline = 0.95;
  tracker.yaw_decay_baseline = 0.95;
  tracker.coast_decay_multiplier = 0.85;
  tracker.position_overshoot_threshold = 0.10;
  tracker.yaw_overshoot_threshold = 0.15;
  tracker.acceleration_smoothing_factor = 0.3;
  tracker.refresh_frequency = 30.0;
  tracker.stationary_measurement_threshold = 0.03;
  tracker.stationary_innovation_threshold = 0.05;
  tracker.stationary_speed_threshold = 0.35;
  tracker.stationary_yaw_rate_threshold = 0.60;
  tracker.stationary_required_frames = 4;
  tracker.stationary_velocity_collapse_factor = 0.25;
  tracker.stationary_zero_velocity_threshold = 0.05;
  tracker.visible_direct_obliquity_threshold_deg = 60.0;
  config.process_noise_position = 5.0;
  config.process_noise_yaw = 10.0;
  config.process_noise_radius = 1e-6;
  config.refresh_frequency = 30.0;
  return config;
}

TEST(TrackerFactoryTest, CreatesTrackerWithConfiguredEkf)
{
  TrackerFactory factory(makeTrackingConfig(), []() { return 1.0 / 30.0; });
  auto tracker = factory.create();
  tracker->ekf.setState(Eigen::VectorXd::Zero(9));

  EXPECT_EQ(tracker->tracking_thres, 5);
  EXPECT_NEAR(tracker->ekf.getVariance(0), 0.1, 1e-9);
  EXPECT_NEAR(tracker->ekf.getVariance(7), 3.0, 1e-9);
}

}  // namespace
}  // namespace rm_auto_aim
