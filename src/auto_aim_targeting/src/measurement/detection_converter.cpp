// ============================================================================
// detection_converter.cpp — Converts raw YOLO detections into 3D armors.
//
// ENTRYPOINTS:
//   setCameraInfo    → one-time init with camera intrinsics (creates PnP solver)
//   buildObservation → per-frame pipeline: detect → PnP solve → TF → filter
//
// HELPERS (stepdown):
//   detectArmors                 → bbox → PnP → Armor messages
//     buildArmorFromDetection    → YOLO bbox extraction, class filter, light-bar corners
//     solveArmorPose             → PnP solve, Rodrigues→quaternion, validation
//   transformToTrackingFrame     → camera frame → odom frame via TF2
//   filterImplausibleDetections  → reject impossible heights / distances
// ============================================================================
#include "auto_aim_targeting/measurement/detection_converter.hpp"

#include <Eigen/Eigen>

#include <algorithm>
#include <cmath>

#include <opencv2/calib3d.hpp>
#include <rclcpp/logging.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "auto_aim_targeting/measurement/armor_model.hpp"

namespace rm_auto_aim
{

DetectionConverter::DetectionConverter(
  MeasurementConfig config,
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer,
  rclcpp::Logger logger)
: config_(std::move(config)),
  tf2_buffer_(std::move(tf2_buffer)),
  logger_(std::move(logger))
{
}

void DetectionConverter::setCameraInfo(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info)
{
  this->pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
}

Observation DetectionConverter::buildObservation(
  const vision_msgs::msg::Detection2DArray::ConstSharedPtr & detection_msg,
  const GimbalPoseState & pose_state) const
{
  Observation input;
  input.detection_msg = detection_msg;
  input.stamp = detection_msg->header.stamp;
  input.pose_state = pose_state;
  input.armors = this->detectArmors(detection_msg, input.detection_indices);
  this->transformToTrackingFrame(input.armors);
  this->filterImplausibleDetections(input.armors, input.detection_indices);
  return input;
}

auto_aim_interfaces::msg::Armors::SharedPtr DetectionConverter::detectArmors(
  const vision_msgs::msg::Detection2DArray::ConstSharedPtr & detection_msg,
  std::vector<size_t> & detection_indices) const
{
  auto_aim_interfaces::msg::Armors armors_msg;
  armors_msg.header = detection_msg->header;
  if (armors_msg.header.frame_id.empty()) {
    armors_msg.header.frame_id = "camera_color_optical_frame";
  }

  if (this->pnp_solver_ == nullptr) {
    return std::make_shared<auto_aim_interfaces::msg::Armors>(armors_msg);
  }

  size_t detection_idx = 0;
  for (const auto & detection : detection_msg->detections) {
    auto armor_obj = this->buildArmorFromDetection(detection);
    if (!armor_obj) {
      ++detection_idx;
      continue;
    }

    auto armor_msg = this->solveArmorPose(*armor_obj);
    if (!armor_msg) {
      ++detection_idx;
      continue;
    }

    armors_msg.armors.emplace_back(*armor_msg);
    detection_indices.push_back(detection_idx);
    ++detection_idx;
  }

  return std::make_shared<auto_aim_interfaces::msg::Armors>(armors_msg);
}

std::optional<Armor> DetectionConverter::buildArmorFromDetection(
  const vision_msgs::msg::Detection2D & detection) const
{
  const auto center_x = detection.bbox.center.position.x;
  const auto center_y = detection.bbox.center.position.y - this->config_.bbox_padding_y;
  const auto width = detection.bbox.size_x;
  const auto height = detection.bbox.size_y;
  if (height < 1.0 || width < 1.0) {
    return std::nullopt;
  }

  if (!detection.results.empty()) {
    const auto & class_id = detection.results[0].hypothesis.class_id;
    if (this->config_.target_classes.find(class_id) == this->config_.target_classes.end()) {
      return std::nullopt;
    }
  }

  Armor armor_obj;
  armor_obj.center = cv::Point2f(center_x, center_y);
  armor_obj.type = ((width / height) > 1.5) ? ArmorType::LARGE : ArmorType::SMALL;
  armor_obj.number = detection.results.empty() ? "unknown" : detection.results[0].hypothesis.class_id;
  armor_obj.confidence = detection.results.empty() ? 0.0 : detection.results[0].hypothesis.score;
  const float lr = static_cast<float>(this->config_.light_ratio);
  const float lx = center_x - lr * width / 2;
  const float rx = center_x + lr * width / 2;
  const float ty = center_y - lr * height / 2;
  const float by = center_y + lr * height / 2;

  armor_obj.left_light.center = cv::Point2f(lx, center_y);
  armor_obj.left_light.top = cv::Point2f(lx, ty);
  armor_obj.left_light.bottom = cv::Point2f(lx, by);
  armor_obj.right_light.center = cv::Point2f(rx, center_y);
  armor_obj.right_light.top = cv::Point2f(rx, ty);
  armor_obj.right_light.bottom = cv::Point2f(rx, by);

  return armor_obj;
}

std::optional<auto_aim_interfaces::msg::Armor> DetectionConverter::solveArmorPose(
  const Armor & armor) const
{
  cv::Mat rvec, tvec;
  const bool success = this->pnp_solver_->solvePnP(armor, rvec, tvec);
  if (!success) {
    return std::nullopt;
  }

  auto_aim_interfaces::msg::Armor armor_msg;
  armor_msg.type = ARMOR_TYPE_STR[static_cast<int>(armor.type)];
  armor_msg.number = armor.number;
  armor_msg.pose.position.x = tvec.at<double>(0);
  armor_msg.pose.position.y = tvec.at<double>(1);
  armor_msg.pose.position.z = tvec.at<double>(2);

  cv::Mat rotation_matrix;
  cv::Rodrigues(rvec, rotation_matrix);
  tf2::Matrix3x3 tf2_rotation_matrix(
    rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
    rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0),
    rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
    rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
    rotation_matrix.at<double>(2, 2));
  tf2::Quaternion tf2_q;
  tf2_rotation_matrix.getRotation(tf2_q);
  const double q_len = tf2_q.length();
  if (!std::isfinite(q_len) || q_len < 1e-6) {
    return std::nullopt;
  }
  tf2_q.normalize();
  armor_msg.pose.orientation = tf2::toMsg(tf2_q);
  armor_msg.distance_to_image_center = this->pnp_solver_->calculateDistanceToCenter(armor.center);

  return armor_msg;
}

void DetectionConverter::transformToTrackingFrame(
  auto_aim_interfaces::msg::Armors::SharedPtr & armors) const
{
  if (armors->armors.empty()) {
    return;
  }

  geometry_msgs::msg::TransformStamped tf_stamped;
  try {
    tf_stamped = this->tf2_buffer_->lookupTransform(
      this->config_.target_frame,
      armors->header.frame_id,
      armors->header.stamp,
      rclcpp::Duration::from_seconds(0.01));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(this->logger_, "TF2 error: %s", ex.what());
    armors->armors.clear();
    return;
  }

  for (auto & armor : armors->armors) {
    geometry_msgs::msg::PoseStamped ps_in, ps_out;
    ps_in.header = armors->header;
    ps_in.pose = armor.pose;
    tf2::doTransform(ps_in, ps_out, tf_stamped);
    armor.pose = ps_out.pose;
  }
  armors->header.frame_id = this->config_.target_frame;
}

void DetectionConverter::filterImplausibleDetections(
  auto_aim_interfaces::msg::Armors::SharedPtr & armors,
  std::vector<size_t> & detection_indices) const
{
  auto & armors_vec = armors->armors;
  for (size_t i = 0; i < armors_vec.size();) {
    auto & armor = armors_vec[i];
    if (
      std::abs(armor.pose.position.z) > 1.2 ||
      Eigen::Vector2d(armor.pose.position.x, armor.pose.position.y).norm() >
      this->config_.max_armor_distance)
    {
      armors_vec.erase(armors_vec.begin() + static_cast<long>(i));
      detection_indices.erase(detection_indices.begin() + static_cast<long>(i));
    } else {
      ++i;
    }
  }
}

}  // namespace rm_auto_aim
