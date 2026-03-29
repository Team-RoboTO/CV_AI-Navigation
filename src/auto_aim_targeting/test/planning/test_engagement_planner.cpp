#include <gtest/gtest.h>

#include <cmath>

#include "auto_aim_targeting/planning/shot_planner.hpp"
#include "auto_aim_targeting/planning/ballistics_solver.hpp"
#include "auto_aim_targeting/planning/engagement_planner.hpp"

namespace rm_auto_aim
{
namespace
{

TrackSnapshot makeBaseSnapshot()
{
  TrackSnapshot snapshot;
  snapshot.tracker_id = 7;
  snapshot.tracker_state = Tracker::TRACKING;
  snapshot.tracking = true;
  snapshot.temp_lost = false;
  snapshot.id = "3";
  snapshot.armors_num = 4;
  snapshot.position.x = 3.22;
  snapshot.position.y = 0.0;
  snapshot.position.z = 0.10;
  snapshot.velocity.x = 0.0;
  snapshot.velocity.y = 0.0;
  snapshot.velocity.z = 0.0;
  snapshot.acceleration.x = 0.0;
  snapshot.acceleration.y = 0.0;
  snapshot.acceleration.z = 0.0;
  snapshot.yaw = 0.0;
  snapshot.v_yaw = 0.0;
  snapshot.radius_1 = 0.22;
  snapshot.radius_2 = 0.30;
  snapshot.dz = 0.10;
  snapshot.v_yaw_variance = 0.01;
  snapshot.position_variance_x = 0.01;
  snapshot.position_variance_y = 0.01;
  snapshot.position_variance_z = 0.01;
  snapshot.last_measurement_stamp = rclcpp::Time(1, 0, RCL_ROS_TIME);
  snapshot.measurement_fresh = true;
  snapshot.matched_face.valid = true;
  snapshot.matched_face.face_index = 0;
  snapshot.matched_face.alternate_pair = false;
  snapshot.matched_face.position.x = 3.0;
  snapshot.matched_face.position.y = 0.0;
  snapshot.matched_face.position.z = 0.10;
  snapshot.matched_face.yaw = 0.0;
  snapshot.matched_face.radius = 0.22;
  snapshot.matched_face.dz_offset = 0.0;
  snapshot.matched_face.visibility = 1.0;
  snapshot.matched_face.obliquity = 0.0;
  snapshot.matched_face.observable = true;
  snapshot.matched_face.fresh = true;
  return snapshot;
}

PlanningContext makeContext()
{
  PlanningContext ctx;
  ctx.time_bias = 0.0;
  ctx.gimbal_response_delay = 0.0;
  ctx.gimbal_height = 0.0;
  ctx.bullet_speed = 25.0;
  ctx.angular_window = 0.09;
  ctx.angular_window_ref_dist = 3.0;
  ctx.min_fire_dist = 0.5;
  ctx.max_fire_dist = 10.0;
  ctx.max_measurement_age = 0.10;
  ctx.current_yaw = 0.0;
  ctx.current_pitch = 0.0;
  ctx.max_gimbal_yaw_rate = 6.0;
  ctx.max_gimbal_pitch_rate = 4.0;
  ctx.indirect_vyaw_threshold = 3.0;
  ctx.indirect_timing_tolerance = 0.02;
  ctx.indirect_max_candidates = 8;
  ctx.visibility_exponent = 2.0;
  ctx.previous_tracker_id = -1;
  ctx.previous_indirect_mode = false;
  ctx.transport_delay = 0.0;
  return ctx;
}

TEST(EngagementPlannerTest, PrefersVisibleDirectWhenMatchedFaceIsFresh)
{
  BallisticsSolver solver(BallisticsParams{});
  ShotPlanner predictor(solver);
  EngagementPlanner planner(predictor, CostWeights{});

  const auto snapshot = makeBaseSnapshot();
  const auto plan = planner.selectBestPlan({snapshot}, makeContext(), rclcpp::Time(1, 0, RCL_ROS_TIME));

  ASSERT_TRUE(plan.has_value());
  EXPECT_EQ(plan->plan.mode, AimMode::VISIBLE_DIRECT);
  EXPECT_EQ(plan->plan.tracker_id, snapshot.tracker_id);
  EXPECT_EQ(plan->plan.face_index, snapshot.matched_face.face_index);
  EXPECT_EQ(plan->track.tracker_id, snapshot.tracker_id);
}

TEST(EngagementPlannerTest, FallsBackToPredictedDirectWhenMatchedFaceIsStale)
{
  BallisticsSolver solver(BallisticsParams{});
  ShotPlanner predictor(solver);
  EngagementPlanner planner(predictor, CostWeights{});

  auto snapshot = makeBaseSnapshot();
  snapshot.matched_face.fresh = false;
  snapshot.measurement_fresh = false;

  const auto plan = planner.selectBestPlan({snapshot}, makeContext(), rclcpp::Time(1, 0, RCL_ROS_TIME));

  ASSERT_TRUE(plan.has_value());
  EXPECT_EQ(plan->plan.mode, AimMode::PREDICTED_DIRECT);
}

TEST(EngagementPlannerTest, UsesIndirectModeForFastSpinningTarget)
{
  BallisticsSolver solver(BallisticsParams{});
  ShotPlanner predictor(solver);
  EngagementPlanner planner(predictor, CostWeights{});

  auto snapshot = makeBaseSnapshot();
  snapshot.v_yaw = 4.5;
  snapshot.matched_face.fresh = false;
  snapshot.measurement_fresh = false;

  const auto plan = planner.selectBestPlan({snapshot}, makeContext(), rclcpp::Time(1, 0, RCL_ROS_TIME));

  ASSERT_TRUE(plan.has_value());
  EXPECT_EQ(plan->plan.mode, AimMode::INDIRECT);
  EXPECT_TRUE(plan->plan.indirect);
}

}  // namespace
}  // namespace rm_auto_aim
