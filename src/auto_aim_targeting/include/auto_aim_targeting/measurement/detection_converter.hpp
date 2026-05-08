#ifndef AUTO_AIM_TARGETING__MEASUREMENT__DETECTION_CONVERTER_HPP_
#define AUTO_AIM_TARGETING__MEASUREMENT__DETECTION_CONVERTER_HPP_

#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include <rclcpp/logger.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/buffer.h>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "auto_aim_targeting/types.hpp"
#include "auto_aim_targeting/measurement/armor_model.hpp"
#include "auto_aim_targeting/measurement/pnp_solver.hpp"
#include "auto_aim_targeting/config.hpp"

namespace rm_auto_aim
{

bool correctArmorYawAgainstBearing(
  auto_aim_interfaces::msg::Armor & armor,
  double max_oblique_rad);

class DetectionConverter
{
public:
  DetectionConverter(
    MeasurementConfig config,
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer,
    rclcpp::Logger logger);

  bool setCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info);
  bool ready() const { return this->pnp_solver_ != nullptr; }
  Observation buildObservation(
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr & detection_msg,
    const GimbalPoseState & pose_state) const;

private:
  auto_aim_interfaces::msg::Armors::SharedPtr detectArmors(
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr & detection_msg,
    std::vector<size_t> & detection_indices) const;
  std::optional<Armor> buildArmorFromDetection(
    const vision_msgs::msg::Detection2D & detection) const;
  std::optional<auto_aim_interfaces::msg::Armor> solveArmorPose(
    const Armor & armor) const;
  void transformToTrackingFrame(auto_aim_interfaces::msg::Armors::SharedPtr & armors) const;
  void correctYawConventions(auto_aim_interfaces::msg::Armors::SharedPtr & armors) const;
  void filterImplausibleDetections(
    auto_aim_interfaces::msg::Armors::SharedPtr & armors,
    std::vector<size_t> & detection_indices) const;

  MeasurementConfig config_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  rclcpp::Logger logger_;
  std::unique_ptr<PnPSolver> pnp_solver_;
};

}  // namespace rm_auto_aim

#endif  // AUTO_AIM_TARGETING__MEASUREMENT__DETECTION_CONVERTER_HPP_
