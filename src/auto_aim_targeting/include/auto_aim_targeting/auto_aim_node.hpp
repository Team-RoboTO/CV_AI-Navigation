#ifndef AUTO_AIM_TARGETING__AUTO_AIM_NODE_HPP_
#define AUTO_AIM_TARGETING__AUTO_AIM_NODE_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "auto_aim_targeting/planning/shot_planner.hpp"
#include "auto_aim_targeting/planning/ballistics_solver.hpp"
#include "auto_aim_targeting/planning/engagement_planner.hpp"
#include "auto_aim_targeting/planning/fire_gate.hpp"
#include "auto_aim_targeting/planning/latency_compensator.hpp"
#include "auto_aim_targeting/measurement/detection_converter.hpp"
#include "auto_aim_targeting/io/output_publisher.hpp"
#include "auto_aim_targeting/types.hpp"
#include "auto_aim_targeting/io/gimbal_pose_adapter.hpp"
#include "auto_aim_targeting/config.hpp"
#include "auto_aim_targeting/tracking/tracker_manager.hpp"
#include "auto_aim_targeting/tracking/tracker_runtime_factory.hpp"

namespace rm_auto_aim
{

class AutoAimNode : public rclcpp::Node
{
public:
  explicit AutoAimNode(const rclcpp::NodeOptions & options);

private:
  // --- Runtime path (stepdown: caller before callee) ---
  void armorsCallback(const vision_msgs::msg::Detection2DArray::ConstSharedPtr detection_msg);
  void processFrame(const Observation & input);
  void detectorWatchdogCallback();
  void publishHoldOutput(const rclcpp::Time & stamp);
  void updateTimestep(const rclcpp::Time & stamp);
  bool isPipelineReady() const;

  // --- Leaf callbacks ---
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info);
  void microPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void cameraImuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);

  // --- Initialization (called once from constructor) ---
  void loadConfig();
  void loadMeasurementConfig();
  void loadTrackingConfig();
  void loadPoseConfig();
  void loadBallisticsConfig();
  void loadEngagementConfig();
  void loadVisualizationConfig();

  void createRosInterfaces();
  void createServices();
  void createTfInfrastructure();
  void createPublishers();
  void createSubscriptions();

  void buildPipeline();

  // --- State ---
  TargetingConfig config_{};
  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};
  double dt_ = 1.0 / 30.0;

  std::mutex tracker_mutex_;
  Trackers::TrackerStore trackers_{};
  int next_tracker_id_ = 0;
  int best_tracker_id_ = -1;
  int previous_tracker_id_ = -1;
  bool indirect_mode_active_ = false;
  std::chrono::steady_clock::time_point last_frame_steady_time_{};
  bool have_seen_frame_ = false;

  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr armors_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr micro_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_tracker_srv_;
  rclcpp::TimerBase::SharedPtr detector_watchdog_timer_;

  std::unique_ptr<DetectionConverter> detection_converter_;
  std::unique_ptr<GimbalPoseAdapter> gimbal_pose_adapter_;
  std::unique_ptr<TrackerFactory> tracker_factory_;
  std::unique_ptr<LatencyCompensator> latency_compensator_;
  std::unique_ptr<FireGate> fire_gate_;
  std::unique_ptr<TargetingOutputPublisher> output_publisher_;
  std::unique_ptr<BallisticsSolver> ballistics_solver_;
  std::unique_ptr<ShotPlanner> shot_planner_;
  std::unique_ptr<EngagementPlanner> engagement_planner_;
};

}  // namespace rm_auto_aim

#endif  // AUTO_AIM_TARGETING__AUTO_AIM_NODE_HPP_
