#ifndef AUTO_AIM__FRAME_TRANSFORMER_HPP_
#define AUTO_AIM__FRAME_TRANSFORMER_HPP_

#include <opencv2/core.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

namespace auto_aim
{

// Holds the camera-to-odom transform derived from the microcontroller IMU
// and applies it to PnP outputs. The transform is rebuilt on each /micro_imu
// callback; detection callbacks read the cached rotation and translation.
//
// Reference frames:
//   camera optical: standard ROS convention, +Z forward, +X right, +Y down
//   gimbal body:    +X forward, +Y left, +Z up
//   odom:           same orientation as gimbal at startup, fixed in world
//
// The IMU yaw and pitch are already pre-multiplied by yaw_sign / pitch_sign
// before being passed in; this class does not flip signs.
class FrameTransformer
{
public:
  // Result of transforming one PnP output into odom.
  struct TransformedDetection
  {
    double x = 0, y = 0, z = 0;   // armor center in odom [m]
    double yaw = 0;               // armor face normal direction [rad]
    bool   valid = false;
  };

  FrameTransformer() = default;

  // Set the gimbal height [m] used as the camera-to-odom translation.
  void setGimbalHeight(double h) { gimbal_height_ = h; }

  // Update the cached camera-to-odom transform from the latest IMU yaw/pitch.
  // Both are in radians, in the microcontroller convention (already
  // multiplied by yaw_sign / pitch_sign upstream).
  void updateFromImu(double imu_yaw, double imu_pitch);

  // Return true once updateFromImu has been called at least once.
  bool ready() const { return ready_; }

  // Cached rotation/translation from camera optical to odom.
  const tf2::Quaternion & rotation() const { return rotation_; }
  const tf2::Vector3 & translation() const { return translation_; }

  // Apply the cached transform to a single PnP detection.
  //   tvec   - PnP translation [m] in the camera optical frame
  //   rvec   - PnP Rodrigues rotation [rad]
  //   max_oblique_rad - if the armor yaw differs from the bearing by more
  //                     than this, the yaw is clamped to the bearing
  //                     (degenerate plate normal case)
  TransformedDetection apply(const cv::Mat & rvec, const cv::Mat & tvec,
                             double max_oblique_rad) const;

private:
  double gimbal_height_ = 0.325;
  tf2::Quaternion rotation_{0, 0, 0, 1};
  tf2::Vector3 translation_{0, 0, 0};
  bool ready_ = false;
};

}  // namespace auto_aim
#endif
