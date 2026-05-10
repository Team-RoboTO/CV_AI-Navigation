#include "auto_aim/frame_transformer.hpp"

#include <angles/angles.h>
#include <opencv2/calib3d.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

namespace auto_aim
{

void FrameTransformer::updateFromImu(double imu_yaw, double imu_pitch)
{
  // gimbal/head orientation in the micro reference frame.
  tf2::Quaternion q_gimbal;
  q_gimbal.setRPY(0.0, imu_pitch, imu_yaw);

  // camera optical frame convention (constant).
  tf2::Quaternion q_conv;
  q_conv.setRPY(-M_PI / 2.0, 0.0, -M_PI / 2.0);

  rotation_ = q_gimbal * q_conv;
  translation_ = tf2::Vector3(0, 0, gimbal_height_);
  ready_ = true;
}

FrameTransformer::TransformedDetection FrameTransformer::apply(
  const cv::Mat & rvec, const cv::Mat & tvec, double max_oblique_rad) const
{
  TransformedDetection out;

  // tvec to camera-frame Vector3.
  tf2::Vector3 p_cam(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));

  // Rodrigues to a tf2 quaternion via 3x3 rotation matrix.
  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);
  tf2::Matrix3x3 tf_rm(
    rmat.at<double>(0, 0), rmat.at<double>(0, 1), rmat.at<double>(0, 2),
    rmat.at<double>(1, 0), rmat.at<double>(1, 1), rmat.at<double>(1, 2),
    rmat.at<double>(2, 0), rmat.at<double>(2, 1), rmat.at<double>(2, 2));
  tf2::Quaternion q_armor;
  tf_rm.getRotation(q_armor);
  if (q_armor.length() < 1e-6) return out;
  q_armor.normalize();

  // camera optical -> odom.
  tf2::Vector3 p_odom = tf2::quatRotate(rotation_, p_cam) + translation_;
  tf2::Quaternion q_odom = rotation_ * q_armor;
  double r, p, y;
  tf2::Matrix3x3(q_odom).getRPY(r, p, y);

  // tracker yaw points from the armor plate toward the robot center. PnP can
  // flip the plate normal, so clamp it against the bearing before it gets
  // into the EKF.
  double bearing_inward = std::atan2(p_odom.y(), p_odom.x());
  double yaw_inward = y;
  if (std::abs(angles::shortest_angular_distance(bearing_inward, yaw_inward)) > M_PI / 2.0) {
    yaw_inward = angles::normalize_angle(yaw_inward + M_PI);
  }
  if (std::abs(angles::shortest_angular_distance(bearing_inward, yaw_inward)) > max_oblique_rad) {
    yaw_inward = bearing_inward;
  }

  out.x = p_odom.x();
  out.y = p_odom.y();
  out.z = p_odom.z();
  out.yaw = yaw_inward;
  out.valid = true;
  return out;
}

}  // namespace auto_aim
