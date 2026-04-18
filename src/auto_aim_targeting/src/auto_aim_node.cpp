// ============================================================================
// auto_aim_node.cpp — ROS 2 component node that orchestrates the full auto-aim pipeline.
//
// ENTRYPOINT: AutoAimNode constructor (loaded via rclcpp_components).
//
// The main loop runs in armorsCallback(), triggered by each YOLO detection frame:
//   armorsCallback → buildObservation → processFrame → publish
//
// processFrame executes the pipeline in order:
//   1. predictAll         (EKF predict all trackers)
//   2. assignDetections   (Mahalanobis assignment)
//   3. updateMatched      (EKF update with assigned detections)
//   4. spawnFromUnmatched
//   5. pruneLost
//   6. selectBestPlan            (multi-objective engagement planning)
//   7. evaluate                  (fire gate safety check)
//   8. buildAimResult + publish
// ============================================================================
#include "auto_aim_targeting/auto_aim_node.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace rm_auto_aim
{

// ============================================================================
// Constructor
// ============================================================================

AutoAimNode::AutoAimNode(const rclcpp::NodeOptions & options)
: Node("auto_aim_targeting", options)
{
  RCLCPP_INFO(get_logger(), "Starting AutoAimNode");
  loadConfig();
  createRosInterfaces();
  buildPipeline();
}

// ============================================================================
// Runtime path (stepdown: caller before callee)
// ============================================================================

void AutoAimNode::armorsCallback(
  const vision_msgs::msg::Detection2DArray::ConstSharedPtr detection_msg)
{
  if (!isPipelineReady()) {
    return;
  }

  gimbal_pose_adapter_->updateForFrame(detection_msg->header.stamp);
  const GimbalPoseState pose_state = gimbal_pose_adapter_->currentPoseState(now());
  const Observation input = detection_converter_->buildObservation(detection_msg, pose_state);
  processFrame(input);
}

void AutoAimNode::processFrame(const Observation & input)
{
  std::lock_guard<std::mutex> lock(this->tracker_mutex_);

  this->have_seen_frame_ = true;
  this->last_frame_steady_time_ = std::chrono::steady_clock::now();

  Observation safe_input = input;
  if (safe_input.armors == nullptr) {
    auto empty_armors = std::make_shared<auto_aim_interfaces::msg::Armors>();
    empty_armors->header.stamp = input.stamp;
    empty_armors->header.frame_id = config_.measurement.target_frame;
    safe_input.armors = empty_armors;
  }
  if (safe_input.detection_msg == nullptr) {
    auto empty_detections = std::make_shared<vision_msgs::msg::Detection2DArray>();
    empty_detections->header = safe_input.armors->header;
    safe_input.detection_msg = empty_detections;
  }

  updateTimestep(safe_input.stamp);
  Trackers::predictAll(trackers_, dt_, config_.tracking.lost_time_thres);
  const auto detection_owners = Trackers::assignDetections(
    trackers_, safe_input.armors, config_.tracking.tracker.mahalanobis_jump_gate);
  Trackers::updateMatched(trackers_, safe_input.armors, detection_owners);
  Trackers::spawnFromUnmatched(
    trackers_,
    next_tracker_id_,
    safe_input.armors,
    detection_owners,
    config_.tracking.max_trackers,
    config_.tracking.new_tracker_min_dist,
    config_.tracking.new_tracker_assumed_radius,
    [this]() { return tracker_factory_->create(); },
    safe_input.stamp,
    get_logger());
  Trackers::pruneLost(trackers_, get_logger());

  const auto snapshots = Trackers::collectSnapshots(trackers_, dt_);
  std::optional<AimDecision> decision;
  if (!snapshots.empty()) {
    latency_compensator_->updateFromFrameStamp(now(), safe_input.stamp);
    const PlanningContext ctx = makePlanningContext(
      config_,
      safe_input.pose_state,
      latency_compensator_->transportDelay(now(), safe_input.stamp),
      previous_tracker_id_,
      previous_face_index_,
      indirect_mode_active_,
      latency_compensator_->currentBias());
    decision = engagement_planner_->selectBestPlan(snapshots, ctx, safe_input.stamp);
  }

  best_tracker_id_ = decision.has_value() ? decision->plan.tracker_id : -1;
  const bool has_pose_source = config_.pose.pose_source != "none";
  const FireGateResult fire_result = decision.has_value()
    ? fire_gate_->evaluate(decision->plan, safe_input.pose_state.fresh, has_pose_source)
    : FireGateResult{};
  AimResult output = output_publisher_->buildAimResult(
    safe_input, trackers_, best_tracker_id_, decision, fire_result);
  output_publisher_->publish(output);

  if (decision.has_value()) {
    previous_tracker_id_ = decision->plan.tracker_id;
    previous_face_index_ = decision->plan.face_index;
    indirect_mode_active_ = decision->plan.indirect;
  } else {
    previous_tracker_id_ = -1;
    previous_face_index_ = -1;
    indirect_mode_active_ = false;
  }
}

void AutoAimNode::detectorWatchdogCallback()
{
  if (!isPipelineReady()) {
    return;
  }

  const rclcpp::Time stamp = now();
  const auto steady_now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(this->tracker_mutex_);
  if (this->have_seen_frame_ &&
      std::chrono::duration<double>(steady_now - this->last_frame_steady_time_).count() <=
      this->config_.engagement.detector_stall_timeout)
  {
    return;
  }

  this->publishHoldOutput(stamp);
}

void AutoAimNode::publishHoldOutput(const rclcpp::Time & stamp)
{
  Observation input;
  input.stamp = stamp;
  input.pose_state = this->gimbal_pose_adapter_->currentPoseState(stamp);
  input.armors = std::make_shared<auto_aim_interfaces::msg::Armors>();
  input.armors->header.stamp = stamp;
  input.armors->header.frame_id = this->config_.measurement.target_frame;
  auto detections = std::make_shared<vision_msgs::msg::Detection2DArray>();
  detections->header = input.armors->header;
  input.detection_msg = detections;

  this->best_tracker_id_ = -1;
  const std::optional<AimDecision> no_decision;
  AimResult output = this->output_publisher_->buildAimResult(
    input, this->trackers_, this->best_tracker_id_, no_decision, FireGateResult{});
  this->output_publisher_->publish(output);
  this->previous_tracker_id_ = -1;
  this->previous_face_index_ = -1;
  this->indirect_mode_active_ = false;
}

void AutoAimNode::updateTimestep(const rclcpp::Time & stamp)
{
  if (last_time_.nanoseconds() > 0) {
    dt_ = std::min(std::max((stamp - last_time_).seconds(), 0.01), 0.10);
  }
  last_time_ = stamp;
}

bool AutoAimNode::isPipelineReady() const
{
  return detection_converter_ != nullptr &&
         gimbal_pose_adapter_ != nullptr &&
         engagement_planner_ != nullptr &&
         tracker_factory_ != nullptr &&
         latency_compensator_ != nullptr &&
         fire_gate_ != nullptr &&
         output_publisher_ != nullptr;
}

// ============================================================================
// Leaf callbacks
// ============================================================================

void AutoAimNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info)
{
  if (detection_converter_ != nullptr) {
    detection_converter_->setCameraInfo(camera_info);
  }
  cam_info_sub_.reset();
}

void AutoAimNode::microPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  if (gimbal_pose_adapter_ != nullptr) {
    gimbal_pose_adapter_->onMicroPose(msg);
  }
}

void AutoAimNode::cameraImuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  if (gimbal_pose_adapter_ != nullptr) {
    gimbal_pose_adapter_->onCameraImu(msg);
  }
}

// ============================================================================
// Initialization: config loading
// ============================================================================

void AutoAimNode::loadConfig()
{
  loadMeasurementConfig();
  loadTrackingConfig();
  loadPoseConfig();
  loadBallisticsConfig();
  loadEngagementConfig();
  loadVisualizationConfig();
}

void AutoAimNode::loadMeasurementConfig()
{
  config_.measurement.max_armor_distance = declare_parameter("max_armor_distance", 10.0);
  auto target_classes_vec = declare_parameter<std::vector<std::string>>(
    "target_classes", std::vector<std::string>{"3"});
  config_.measurement.target_classes =
    std::set<std::string>(target_classes_vec.begin(), target_classes_vec.end());
  config_.measurement.target_classes.erase("1");
  config_.measurement.light_ratio = declare_parameter("light_ratio", 0.85);
  config_.measurement.bbox_padding_y = declare_parameter("bbox_padding_y", 80.0);
  config_.measurement.pnp_max_reprojection_error =
    declare_parameter("pnp.max_reprojection_error", 10.0);
  config_.measurement.target_frame = declare_parameter("target_frame", "odom");
}

void AutoAimNode::loadTrackingConfig()
{
  auto & tracker = config_.tracking.tracker;
  tracker.max_match_distance = declare_parameter("tracker.max_match_distance", 0.15);
  tracker.max_track_range = declare_parameter("tracker.max_track_range", 6.0);
  tracker.tracking_threshold = declare_parameter("tracker.tracking_thres", 5);
  config_.tracking.lost_time_thres = declare_parameter("tracker.lost_time_thres", 0.3);
  tracker.initial_close_pair_orbit_radius = declare_parameter("tracker.initial_r1", 0.22);
  tracker.initial_far_pair_orbit_radius = declare_parameter("tracker.initial_r2", 0.30);
  tracker.radius_filter_process_noise = declare_parameter("tracker.r_kf_Q", 3.3e-8);
  tracker.radius_filter_measurement_noise = declare_parameter("tracker.r_kf_R", 0.0004);
  tracker.radius_filter_initial_covariance = declare_parameter("tracker.r_kf_P_init", 0.0064);
  tracker.radius_adaptation_max_range = declare_parameter("tracker.r_adapt_max_dist", 4.0);
  tracker.height_offset_smoothing_factor = declare_parameter("tracker.dz_adapt_alpha", 0.05);
  tracker.radius_yaw_uncertainty_scale = declare_parameter("tracker.r_yaw_uncertainty_scale", 50.0);
  tracker.max_yaw_rate = declare_parameter("tracker.v_yaw_max", 15.0);
  tracker.mahalanobis_match_gate = declare_parameter("tracker.maha_match_threshold", 13.3);
  tracker.mahalanobis_jump_gate = declare_parameter("tracker.maha_jump_threshold", 20.0);
  config_.tracking.max_trackers = declare_parameter("tracker.max_trackers", 5);
  config_.tracking.new_tracker_min_dist = declare_parameter("tracker.new_tracker_min_dist", 0.5);
  config_.tracking.new_tracker_assumed_radius =
    declare_parameter("tracker.new_tracker_assumed_radius", 0.26);

  tracker.position_decay_baseline = declare_parameter("ekf.xyz_damping_alpha", 0.95);
  tracker.yaw_decay_baseline = declare_parameter("ekf.yaw_damping_alpha", 0.95);
  tracker.coast_decay_multiplier = declare_parameter("ekf.coast_damping_factor", 0.85);
  tracker.position_overshoot_threshold = declare_parameter("ekf.damping_innov_threshold", 0.10);
  tracker.yaw_overshoot_threshold = declare_parameter("ekf.yaw_innov_threshold", 0.15);
  config_.tracking.refresh_frequency = declare_parameter("ekf.ref_frequency", 30.0);
  tracker.refresh_frequency = config_.tracking.refresh_frequency;
  tracker.acceleration_smoothing_factor = declare_parameter("ekf.accel_ema_alpha", 0.3);
  config_.tracking.process_noise_position = declare_parameter("ekf.sigma2_q_xyz", 5.0);
  config_.tracking.process_noise_yaw = declare_parameter("ekf.sigma2_q_yaw", 10.0);
  config_.tracking.process_noise_radius = declare_parameter("ekf.sigma2_q_r", 1e-6);
  tracker.position_noise_base = declare_parameter("ekf.r_xyz_base", 0.04);
  tracker.position_noise_slope = declare_parameter("ekf.r_xyz_slope", 0.03);
  tracker.yaw_noise_base = declare_parameter("ekf.r_yaw_base", 0.05);
  tracker.yaw_noise_slope = declare_parameter("ekf.r_yaw_slope", 0.002);
  tracker.yaw_obliquity_exponent = declare_parameter("ekf.r_yaw_angle_power", 4.0);
  tracker.max_yaw_oblique_deg = declare_parameter("ekf.max_yaw_oblique_deg", 65.0);
  tracker.use_secondary_face_fusion = declare_parameter("ekf.secondary_face_fusion", true);
  tracker.secondary_noise_multiplier = declare_parameter("ekf.secondary_r_inflation", 2.0);
  tracker.secondary_mahalanobis_gate = declare_parameter("ekf.secondary_maha_threshold", 13.3);
  tracker.stationary_measurement_threshold =
    declare_parameter("tracker.stationary_measurement_threshold", 0.03);
  tracker.stationary_innovation_threshold =
    declare_parameter("tracker.stationary_innovation_threshold", 0.05);
  tracker.stationary_speed_threshold =
    declare_parameter("tracker.stationary_speed_threshold", 0.35);
  tracker.stationary_yaw_rate_threshold =
    declare_parameter("tracker.stationary_yaw_rate_threshold", 0.60);
  tracker.stationary_required_frames =
    declare_parameter("tracker.stationary_required_frames", 4);
  tracker.stationary_velocity_collapse_factor =
    declare_parameter("tracker.stationary_velocity_collapse_factor", 0.25);
  tracker.stationary_zero_velocity_threshold =
    declare_parameter("tracker.stationary_zero_velocity_threshold", 0.05);
  tracker.visible_direct_obliquity_threshold_deg =
    declare_parameter("tracker.visible_direct_obliquity_threshold_deg", 60.0);
}

void AutoAimNode::loadPoseConfig()
{
  config_.pose.yaw_sign = declare_parameter("gimbal.yaw_sign", 1.0);
  config_.pose.pitch_sign = declare_parameter("gimbal.pitch_sign", 1.0);
  config_.pose.gimbal_height = declare_parameter("gimbal.height", 0.325);
  config_.pose.camera_offset_x = declare_parameter("camera_offset_x", 0.107);
  config_.pose.camera_offset_z = declare_parameter("camera_offset_z", 0.136);
  config_.pose.self_orientation_timeout =
    declare_parameter("tracker.self_orientation_timeout", 0.2);
  config_.pose.pose_source = declare_parameter("pose_source", "none");
  config_.pose.pose_timeout = declare_parameter("micro_pose_timeout", 0.15);
}

void AutoAimNode::loadBallisticsConfig()
{
  config_.ballistics.bullet_speed = declare_parameter("bullet_speed", 25.0);
  config_.ballistics.gravity = declare_parameter("gravity", 9.8);
  config_.ballistics.linear_drag_coeff = declare_parameter("linear_drag_coeff", 0.01);
  config_.ballistics.quadratic_drag_coeff = declare_parameter("quadratic_drag_coeff", 0.01);
  config_.ballistics.use_quadratic_drag = declare_parameter("use_quadratic_drag", false);
  config_.ballistics.gimbal_pitch_max = declare_parameter("gimbal_pitch_max", 0.524);
  config_.ballistics.gimbal_pitch_min = declare_parameter("gimbal_pitch_min", -0.524);
}

void AutoAimNode::loadEngagementConfig()
{
  config_.engagement.time_bias_initial = declare_parameter("time_bias", 0.08);
  config_.engagement.time_bias_alpha = declare_parameter("time_bias_alpha", 0.35);
  config_.engagement.gimbal_response_delay = declare_parameter("gimbal_response_delay", 0.0);
  config_.engagement.min_fire_dist = declare_parameter("min_fire_dist", 0.5);
  config_.engagement.max_fire_dist = declare_parameter("max_fire_dist", 10.0);
  config_.engagement.angular_window = declare_parameter("angular_window", 0.09);
  config_.engagement.angular_window_ref_dist = declare_parameter("angular_window_ref_dist", 3.0);
  config_.engagement.max_measurement_age = declare_parameter("max_measurement_age", 0.10);
  config_.engagement.detector_stall_timeout = declare_parameter("detector_stall_timeout", 0.20);
  config_.engagement.max_gimbal_yaw_rate = declare_parameter("max_gimbal_yaw_rate", 6.0);
  config_.engagement.max_gimbal_pitch_rate = declare_parameter("max_gimbal_pitch_rate", 4.0);
  config_.engagement.fire_yaw_tolerance = declare_parameter("fire_yaw_tolerance", 0.03);
  config_.engagement.fire_pitch_tolerance = declare_parameter("fire_pitch_tolerance", 0.03);
  config_.engagement.indirect_vyaw_threshold = declare_parameter("indirect_vyaw_threshold", 3.0);
  config_.engagement.indirect_timing_tolerance =
    declare_parameter("indirect_timing_tolerance", 0.02);
  config_.engagement.indirect_max_candidates = declare_parameter("indirect_max_candidates", 8);
  config_.engagement.oblique_exponent = declare_parameter("oblique_exponent", 2.0);
  config_.engagement.latency_gate_sigma = declare_parameter("latency_gate_sigma", 2.5);
  config_.engagement.latency_warmup_samples = declare_parameter("latency_warmup_samples", 5);
  config_.engagement.pose_source_is_none = config_.pose.pose_source == "none";
  config_.engagement.allow_fire_without_pose =
    declare_parameter("debug.allow_fire_without_pose", false);

  auto & cost = config_.engagement.cost_weights;
  cost.range = declare_parameter("cost.range", cost.range);
  cost.flight_time = declare_parameter("cost.flight_time", cost.flight_time);
  cost.uncertainty = declare_parameter("cost.uncertainty", cost.uncertainty);
  cost.slew = declare_parameter("cost.slew", cost.slew);
  cost.switch_target = declare_parameter("cost.switch_target", cost.switch_target);
  cost.switch_face = declare_parameter("cost.switch_face", cost.switch_face);
  cost.staleness = declare_parameter("cost.staleness", cost.staleness);
  cost.temp_lost = declare_parameter("cost.temp_lost", cost.temp_lost);
  cost.low_visibility = declare_parameter("cost.low_visibility", cost.low_visibility);
  cost.negative_margin = declare_parameter("cost.negative_margin", cost.negative_margin);
}

void AutoAimNode::loadVisualizationConfig()
{
  config_.visualization.max_cmd_pitch_angle_deg = declare_parameter("max_cmd_angle", 30.0);
  config_.visualization.max_cmd_yaw_angle_deg = declare_parameter("max_cmd_yaw_angle", 180.0);
}

// ============================================================================
// Initialization: ROS interfaces
// ============================================================================

void AutoAimNode::createRosInterfaces()
{
  createServices();
  createTfInfrastructure();
  createPublishers();
  createSubscriptions();
  detector_watchdog_timer_ = create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&AutoAimNode::detectorWatchdogCallback, this));
}

void AutoAimNode::createServices()
{
  reset_tracker_srv_ = create_service<std_srvs::srv::Trigger>(
    "/tracker/reset",
    [this](
      const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response)
    {
      std::lock_guard<std::mutex> lock(this->tracker_mutex_);
      trackers_.clear();
      next_tracker_id_ = 0;
      best_tracker_id_ = -1;
      previous_tracker_id_ = -1;
      previous_face_index_ = -1;
      indirect_mode_active_ = false;
      response->success = true;
    });
}

void AutoAimNode::createTfInfrastructure()
{
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(),
    get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  tf2_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
}

void AutoAimNode::createPublishers()
{
  auto target_qos = rclcpp::SensorDataQoS().reliability(rclcpp::ReliabilityPolicy::Reliable);
  auto info_pub = create_publisher<auto_aim_interfaces::msg::TrackerInfo>("/tracker/info", 10);
  auto targets_pub = create_publisher<auto_aim_interfaces::msg::Targets>("/tracker/targets", target_qos);
  auto optimal_bbox_pub = create_publisher<vision_msgs::msg::Detection2D>(
    "/detections_output/optimal_target", rclcpp::SensorDataQoS());
  auto cmd_pub = create_publisher<auto_aim_interfaces::msg::GimbalCmd>(
    "/tracker/cmd_gimbal", rclcpp::SensorDataQoS());
  auto twist_pub = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(10));
  auto trajectory_marker_pub = create_publisher<visualization_msgs::msg::Marker>(
    "/trajectory/marker", rclcpp::QoS(10));
  auto marker_pub = create_publisher<visualization_msgs::msg::MarkerArray>("/tracker/marker", 10);

  output_publisher_ = std::make_unique<TargetingOutputPublisher>(
    config_,
    std::move(info_pub),
    std::move(targets_pub),
    std::move(optimal_bbox_pub),
    std::move(cmd_pub),
    std::move(twist_pub),
    std::move(trajectory_marker_pub),
    std::move(marker_pub));
}

void AutoAimNode::createSubscriptions()
{
  cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera_info",
    rclcpp::SensorDataQoS(),
    std::bind(&AutoAimNode::cameraInfoCallback, this, std::placeholders::_1));

  armors_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
    "/detector/armors",
    rclcpp::SensorDataQoS(),
    std::bind(&AutoAimNode::armorsCallback, this, std::placeholders::_1));

  if (config_.pose.pose_source == "micro_pose") {
    micro_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/micro_pose",
      rclcpp::SensorDataQoS(),
      std::bind(&AutoAimNode::microPoseCallback, this, std::placeholders::_1));
  } else if (config_.pose.pose_source == "camera_imu") {
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data",
      rclcpp::SensorDataQoS(),
      std::bind(&AutoAimNode::cameraImuCallback, this, std::placeholders::_1));
  }
}

// ============================================================================
// Initialization: pipeline construction
// ============================================================================

void AutoAimNode::buildPipeline()
{
  detection_converter_ = std::make_unique<DetectionConverter>(
    config_.measurement, tf2_buffer_, get_logger());
  gimbal_pose_adapter_ = std::make_unique<GimbalPoseAdapter>(
    config_.pose,
    config_.measurement.target_frame,
    tf2_buffer_,
    tf2_broadcaster_,
    get_logger(),
    get_clock());
  tracker_factory_ = std::make_unique<TrackerFactory>(
    config_.tracking, [this]() { return dt_; });
  latency_compensator_ = std::make_unique<LatencyCompensator>(
    config_.engagement.time_bias_initial,
    config_.engagement.time_bias_alpha,
    config_.engagement.latency_gate_sigma,
    config_.engagement.latency_warmup_samples);
  fire_gate_ = std::make_unique<FireGate>(config_.engagement);
  ballistics_solver_ = std::make_unique<BallisticsSolver>(config_.ballisticsParams());
  shot_planner_ = std::make_unique<ShotPlanner>(*ballistics_solver_);
  engagement_planner_ = std::make_unique<EngagementPlanner>(
    *shot_planner_, config_.engagement.cost_weights);
}

}  // namespace rm_auto_aim

RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::AutoAimNode)
