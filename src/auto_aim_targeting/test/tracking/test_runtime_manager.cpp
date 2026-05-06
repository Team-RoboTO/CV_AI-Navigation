#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <memory>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "auto_aim_targeting/tracking/ekf.hpp"
#include "auto_aim_targeting/tracking/tracker_manager.hpp"
#include "auto_aim_targeting/tracking/tracker_runtime_factory.hpp"

namespace rm_auto_aim
{
namespace
{

TrackerConfig makeConfig()
{
  TrackerConfig config{};
  config.max_match_distance = 0.15;
  config.max_track_range = 6.0;
  config.tracking_threshold = 5;
  config.initial_close_pair_orbit_radius = 0.22;
  config.initial_far_pair_orbit_radius = 0.30;
  config.radius_filter_process_noise = 3.3e-8;
  config.radius_filter_measurement_noise = 0.0004;
  config.radius_filter_initial_covariance = 0.0064;
  config.radius_adaptation_max_range = 4.0;
  config.height_offset_smoothing_factor = 0.05;
  config.radius_yaw_uncertainty_scale = 50.0;
  config.max_yaw_rate = 15.0;
  config.mahalanobis_match_gate = 13.3;
  config.mahalanobis_jump_gate = 20.0;
  config.max_yaw_oblique_deg = 65.0;
  config.use_secondary_face_fusion = true;
  config.secondary_noise_multiplier = 2.0;
  config.secondary_mahalanobis_gate = 13.3;
  config.position_noise_base = 0.04;
  config.position_noise_slope = 0.03;
  config.yaw_noise_base = 0.05;
  config.yaw_noise_slope = 0.002;
  config.yaw_obliquity_exponent = 4.0;
  config.position_decay_baseline = 0.95;
  config.yaw_decay_baseline = 0.95;
  config.coast_decay_multiplier = 0.85;
  config.position_overshoot_threshold = 0.10;
  config.yaw_overshoot_threshold = 0.15;
  config.acceleration_smoothing_factor = 0.3;
  config.refresh_frequency = 30.0;
  config.stationary_measurement_threshold = 0.03;
  config.stationary_innovation_threshold = 0.05;
  config.stationary_speed_threshold = 0.35;
  config.stationary_yaw_rate_threshold = 0.60;
  config.stationary_required_frames = 4;
  config.stationary_velocity_collapse_factor = 0.25;
  config.stationary_zero_velocity_threshold = 0.05;
  config.visible_direct_obliquity_threshold_deg = 60.0;
  return config;
}

TrackingRuntimeConfig makeRuntimeConfig()
{
  TrackingRuntimeConfig config;
  config.tracker = makeConfig();
  config.process_noise_position = 5.0;
  config.process_noise_yaw = 10.0;
  config.process_noise_radius = 1e-6;
  config.refresh_frequency = 30.0;
  return config;
}

ExtendedKalmanFilter makeDummyEkf()
{
  const auto f = [](const Eigen::VectorXd & x) { return x; };
  const auto h = [](const Eigen::VectorXd & x) { return x.head(4); };
  const auto jf = [](const Eigen::VectorXd &) { return Eigen::MatrixXd::Identity(9, 9); };
  const auto jh = [](const Eigen::VectorXd &) {
    Eigen::MatrixXd out = Eigen::MatrixXd::Zero(4, 9);
    out.block<4, 4>(0, 0) = Eigen::Matrix4d::Identity();
    return out;
  };
  const auto uq = []() { return Eigen::MatrixXd::Identity(9, 9) * 1e-3; };
  const auto ur = [](const Eigen::VectorXd &) { return Eigen::MatrixXd::Identity(4, 4) * 1e-3; };
  return ExtendedKalmanFilter(f, h, jf, jh, uq, ur, Eigen::MatrixXd::Identity(9, 9));
}

TEST(TrackManagerTest, ResolveMatchedFaceKeepsMeasuredFaceIdentity)
{
  // Robot center at (3.0, 0, 0) with yaw=0, r=0.22.
  // Face 0 (yaw=0) sits at (3.0 - 0.22, 0, 0) = (2.78, 0, 0), facing camera.
  Eigen::VectorXd state(9);
  state << 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.22;

  auto_aim_interfaces::msg::Armor armor;
  armor.pose.position.x = 2.78;
  armor.pose.position.y = 0.0;
  armor.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.0);
  armor.pose.orientation = tf2::toMsg(q);

  const auto binding = resolveMatchedFace(
    state, ArmorsNum::NORMAL_4, 0.30, 0.0, armor, 60.0);

  ASSERT_TRUE(binding.valid);
  EXPECT_EQ(binding.face_index, 0);
  EXPECT_FALSE(binding.alternate_pair);
  EXPECT_NEAR(binding.position.x, armor.pose.position.x, 1e-9);
  EXPECT_NEAR(binding.position.y, armor.pose.position.y, 1e-9);
  EXPECT_NEAR(binding.position.z, armor.pose.position.z, 1e-9);
  EXPECT_NEAR(binding.yaw, 0.0, 1e-6);
  EXPECT_TRUE(binding.observable);
  EXPECT_TRUE(binding.fresh);
}

TEST(TrackManagerTest, PredictMatchedFacePreservesStoredFaceIdentity)
{
  Tracker tracker(makeConfig());
  tracker.tracker_id = 3;
  tracker.tracker_state = Tracker::TRACKING;
  tracker.tracked_armors_num = ArmorsNum::NORMAL_4;
  tracker.target_state = Eigen::VectorXd::Zero(9);
  tracker.target_state(0) = 3.22;
  tracker.target_state(4) = 0.10;
  tracker.target_state(7) = 1.0;
  tracker.matched_face_.valid = true;
  tracker.matched_face_.face_index = 0;
  tracker.matched_face_.alternate_pair = false;
  tracker.matched_face_.radius = 0.22;
  tracker.matched_face_.dz_offset = 0.0;
  tracker.matched_face_.yaw = 0.0;
  tracker.matched_face_.fresh = true;

  const auto predicted = tracker.predictMatchedFace(0.2);

  ASSERT_TRUE(predicted.valid);
  EXPECT_EQ(predicted.face_index, 0);
  EXPECT_NEAR(predicted.yaw, 0.2, 1e-9);
  EXPECT_NEAR(predicted.position.x, 3.22 - 0.22 * std::cos(0.2), 1e-9);
  EXPECT_NEAR(predicted.position.y, -0.22 * std::sin(0.2), 1e-9);
}

TEST(TrackManagerTest, CollectSnapshotsSkipsLostTrackers)
{
  std::map<int, std::unique_ptr<Tracker>> trackers;

  auto active = std::make_unique<Tracker>(makeConfig());
  active->tracker_id = 11;
  active->tracker_state = Tracker::TRACKING;
  active->tracked_id = "3";
  active->tracked_armors_num = ArmorsNum::NORMAL_4;
  active->target_state = Eigen::VectorXd::Zero(9);
  active->ekf = makeDummyEkf();
  active->ekf.setState(active->target_state);
  active->matched_face_.valid = true;
  active->matched_face_.fresh = true;
  trackers.emplace(11, std::move(active));

  auto lost = std::make_unique<Tracker>(makeConfig());
  lost->tracker_id = 22;
  lost->tracker_state = Tracker::LOST;
  trackers.emplace(22, std::move(lost));

  const auto snapshots = Trackers::collectSnapshots(trackers, 1.0 / 30.0);

  ASSERT_EQ(snapshots.size(), 1u);
  EXPECT_EQ(snapshots.front().tracker_id, 11);
  EXPECT_TRUE(snapshots.front().measurement_fresh);
}

TEST(TrackManagerTest, TrackingThresholdPromotesOnNthMatch)
{
  TrackerFactory factory(makeRuntimeConfig(), []() { return 1.0 / 30.0; });
  auto tracker = factory.create();
  tracker->tracking_thres = 2;

  auto_aim_interfaces::msg::Armor armor;
  armor.number = "3";
  armor.type = "small";
  armor.pose.position.x = 2.78;
  armor.pose.position.y = 0.0;
  armor.pose.position.z = 0.0;
  armor.distance_to_image_center = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.0);
  armor.pose.orientation = tf2::toMsg(q);

  tracker->initFromArmor(armor);
  ASSERT_EQ(tracker->tracker_state, Tracker::DETECTING);

  auto armors = std::make_shared<auto_aim_interfaces::msg::Armors>();
  armors->header.stamp = rclcpp::Time(1, 0, RCL_ROS_TIME);
  armors->armors.push_back(armor);

  tracker->update(armors);
  EXPECT_EQ(tracker->tracker_state, Tracker::DETECTING);

  armors->header.stamp = rclcpp::Time(2, 0, RCL_ROS_TIME);
  tracker->update(armors);
  EXPECT_EQ(tracker->tracker_state, Tracker::TRACKING);
}

}  // namespace
}  // namespace rm_auto_aim
