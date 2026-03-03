#ifndef ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_
#define ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_

// ROS
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// STD
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "armor_tracker/pnp_solver.hpp"
#include "armor_tracker/tracker.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/tracker_info.hpp"

namespace rm_auto_aim
{
using tf2_filter = tf2_ros::MessageFilter<vision_msgs::msg::Detection2DArray>;
class ArmorTrackerNode : public rclcpp::Node
{
public:
  explicit ArmorTrackerNode(const rclcpp::NodeOptions & options);

private:
  void armorsCallback(const vision_msgs::msg::Detection2DArray::ConstSharedPtr armors_ptr);

  void publishMarkers(const auto_aim_interfaces::msg::Target & target_msg);

  // Gimbal TF
  void microPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void broadcastGimbalTF(const rclcpp::Time & stamp);

  // Maximum allowable armor distance in the XOY plane
  double max_armor_distance_;

  // YOLO class IDs to track (e.g. "3" = red_armor); grey/dead ("1") always excluded
  std::set<std::string> target_classes_;

  // The time when the last message was received
  rclcpp::Time last_time_;
  double dt_ = 1.0 / 30.0;

  // Armor tracker
  double s2qxyz_, s2qyaw_, s2qr_;
  double r_xyz_base_, r_xyz_slope_;   // R_pos(d) = (base + slope*d)²
  double r_yaw_base_, r_yaw_slope_;   // R_yaw(d) = (base + slope*d)²
  double lost_time_thres_;
  std::unique_ptr<Tracker> tracker_;

  // Camera info
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  cv::Point2f cam_center_;
  std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;
  std::unique_ptr<PnPSolver> pnp_solver_;
  // Reset tracker service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_tracker_srv_;

  // Subscriber with tf2 message_filter
  std::string target_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr armors_sub_;

  // Tracker info publisher
  rclcpp::Publisher<auto_aim_interfaces::msg::TrackerInfo>::SharedPtr info_pub_;

  // Publisher
  rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr target_pub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2D>::SharedPtr optimal_bbox_pub_;

  // Adaptive velocity damping
  double xyz_damping_alpha_;          // current effective value (updated each frame)
  double yaw_damping_alpha_;          // current effective value
  double xyz_damping_alpha_base_;     // from parameter (0.95)
  double yaw_damping_alpha_base_;     // from parameter (0.98)
  double coast_damping_factor_;       // TEMP_LOST multiplier (0.85)
  double damping_innov_threshold_;    // position_diff threshold (0.10m)
  double yaw_innov_threshold_;       // yaw innovation threshold for overshoot damping (rad)

  // PnP light-bar ratio (bbox → light-bar scaling)
  double light_ratio_;

  // Letterbox padding Y offset (pixels)
  double bbox_padding_y_;

  // Dynamic TF from gimbal feedback
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr micro_pose_sub_;
  double gimbal_yaw_, gimbal_pitch_;
  double gimbal_height_, yaw_sign_, pitch_sign_;

  // IMU-based chassis rotation estimation
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);

  double chassis_yaw_ = 0.0;
  double chassis_pitch_ = 0.0;
  double imu_wz_filtered_ = 0.0;
  double imu_wy_filtered_ = 0.0;
  double prev_gimbal_yaw_ = 0.0;
  double prev_gimbal_pitch_ = 0.0;
  rclcpp::Time prev_imu_time_;
  bool imu_initialized_ = false;
  bool imu_active_ = false;
  rclcpp::Time last_imu_time_;

  // IMU parameters
  bool enable_imu_compensation_ = true;
  double imu_gyro_alpha_ = 0.3;
  double imu_timeout_ = 0.1;
  double imu_yaw_axis_sign_ = -1.0;
  double imu_pitch_axis_sign_ = -1.0;

  // Visualization marker publisher
  visualization_msgs::msg::Marker position_marker_;
  visualization_msgs::msg::Marker linear_v_marker_;
  visualization_msgs::msg::Marker angular_v_marker_;
  visualization_msgs::msg::Marker armor_marker_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_
