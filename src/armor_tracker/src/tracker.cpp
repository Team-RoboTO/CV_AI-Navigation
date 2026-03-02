#include "armor_tracker/tracker.hpp"

#include <angles/angles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>

#include <rclcpp/logger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// STD
#include <cfloat>
#include <memory>
#include <string>
#include <vector>

namespace rm_auto_aim
{
Tracker::Tracker(double max_match_distance, double max_track_range)
: tracker_state(LOST),
  tracked_id(std::string("")),
  info_position_diff(0.0),
  info_yaw_diff(0.0),
  measurement(Eigen::VectorXd::Zero(4)),
  target_state(Eigen::VectorXd::Zero(9)),
  max_match_distance_(max_match_distance),
  max_match_yaw_diff_(M_PI / 3.0),
  max_track_range_(max_track_range)
{
}

void Tracker::init(const Armors::SharedPtr & armors_msg)
{
  if (armors_msg->armors.empty()) {
    return;
  }

  // Simply choose the armor that is closest to image center
  double min_distance = DBL_MAX;
  tracked_armor = armors_msg->armors[0];
  for (const auto & armor : armors_msg->armors) {
    if (armor.distance_to_image_center < min_distance) {
      min_distance = armor.distance_to_image_center;
      tracked_armor = armor;
    }
  }

  initEKF(tracked_armor);
  RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"), "Init EKF!");

  tracked_id = tracked_armor.number;
  tracker_state = DETECTING;

  updateArmorsNum(tracked_armor);
}

void Tracker::update(const Armors::SharedPtr & armors_msg)
{
  // KF predict
  Eigen::VectorXd ekf_prediction = ekf.predict();
  RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"), "EKF predict");

  bool matched = false;
  // Use KF prediction as default target state if no matched armor is found
  target_state = ekf_prediction;

  if (!armors_msg->armors.empty()) {
    // Collect all same-ID armors
    std::vector<Armor> same_id_armors;
    auto predicted_position = getArmorPositionFromState(ekf_prediction);

    for (const auto & armor : armors_msg->armors) {
      if (armor.number == tracked_id) {
        same_id_armors.push_back(armor);
      }
    }

    if (!same_id_armors.empty()) {
      Armor selected_armor;

      if (same_id_armors.size() > 1) {
        // Multiple faces visible: pick the one closest to the EKF prediction.
        // This keeps tracking the same face through the overlap window instead
        // of proactively switching to the face-on armor (which triggers
        // handleArmorJump and disrupts the EKF state for 7-21 frames).
        // Matches upstream rm_auto_aim behavior.
        double min_dist = DBL_MAX;
        for (const auto & armor : same_id_armors) {
          auto p = armor.pose.position;
          Eigen::Vector3d pos(p.x, p.y, p.z);
          double dist = (predicted_position - pos).norm();
          if (dist < min_dist) {
            min_dist = dist;
            selected_armor = armor;
          }
        }
      } else {
        selected_armor = same_id_armors[0];
      }

      // Compute position and yaw differences for the selected armor
      auto p = selected_armor.pose.position;
      Eigen::Vector3d position_vec(p.x, p.y, p.z);
      double position_diff = (predicted_position - position_vec).norm();
      double measured_yaw = orientationToYaw(selected_armor.pose.orientation);
      double yaw_innov_signed =
        angles::shortest_angular_distance(ekf_prediction(6), measured_yaw);
      double yaw_diff = std::abs(yaw_innov_signed);

      // Store tracker info
      info_position_diff = position_diff;
      info_yaw_diff = yaw_diff;
      info_yaw_innov_signed = yaw_innov_signed;
      info_position_innov = Eigen::Vector3d(p.x, p.y, p.z) - predicted_position;
      tracked_armor = selected_armor;

      if (position_diff < max_match_distance_ && yaw_diff < max_match_yaw_diff_) {
        // Matched armor found
        matched = true;
        measurement = Eigen::Vector4d(p.x, p.y, p.z, measured_yaw);
        target_state = ekf.update(measurement);
        RCLCPP_DEBUG(rclcpp::get_logger("armor_tracker"), "EKF update");
      } else if (yaw_diff > max_match_yaw_diff_) {
        // Yaw jumped: spinning (single face) or proactive face switch (multi-face)
        handleArmorJump(selected_armor);
        matched = true;
      } else {
        // No matched armor found
        RCLCPP_WARN(rclcpp::get_logger("armor_tracker"), "No matched armor found!");
      }
    }
  }

  // Prevent radius from spreading
  if (target_state(8) < 0.12) {
    target_state(8) = 0.12;
    ekf.setState(target_state);
  } else if (target_state(8) > 0.4) {
    target_state(8) = 0.4;
    ekf.setState(target_state);
  }

  // Clamp v_yaw to physically plausible range.
  // RoboMaster robots cannot spin faster than ~15 rad/s (~143 rpm);
  // values beyond this indicate EKF divergence from a bad detection.
  if (std::abs(target_state(7)) > 15.0) {
    target_state(7) = std::copysign(15.0, target_state(7));
    ekf.setState(target_state);
    ekf.resetCovariance();
    RCLCPP_WARN(rclcpp::get_logger("armor_tracker"), "v_yaw clamped — covariance reset");
  }

  // Tracking state machine
  if (tracker_state == DETECTING) {
    if (matched) {
      detect_count_++;
      if (detect_count_ > tracking_thres) {
        detect_count_ = 0;
        tracker_state = TRACKING;
      }
    } else {
      detect_count_ = 0;
      tracker_state = LOST;
    }
  } else if (tracker_state == TRACKING) {
    if (!matched) {
      tracker_state = TEMP_LOST;
      lost_count_++;
    }
  } else if (tracker_state == TEMP_LOST) {
    if (!matched) {
      lost_count_++;
      if (lost_count_ > lost_thres) {
        lost_count_ = 0;
        tracker_state = LOST;
      }
    } else {
      tracker_state = TRACKING;
      lost_count_ = 0;
    }
  }

  // Drop target if it is beyond max tracking range
  if (tracker_state != LOST) {
    double xc = target_state(0), yc = target_state(2), za = target_state(4);
    double range = std::sqrt(xc * xc + yc * yc + za * za);
    if (range > max_track_range_) {
      tracker_state = LOST;
      RCLCPP_WARN(rclcpp::get_logger("armor_tracker"),
        "Target beyond max range (%.1fm > %.1fm) — LOST", range, max_track_range_);
    }
  }
}

void Tracker::initEKF(const Armor & a)
{
  double xa = a.pose.position.x;
  double ya = a.pose.position.y;
  double za = a.pose.position.z;
  last_yaw_ = 0;
  double yaw = orientationToYaw(a.pose.orientation);

  // Set initial position at 0.2m behind the target
  target_state = Eigen::VectorXd::Zero(9);
  double r = 0.26;
  double xc = xa + r * cos(yaw);
  double yc = ya + r * sin(yaw);
  dz = 0, another_r = r;
  target_state << xc, 0, yc, 0, za, 0, yaw, 0, r;

  ekf.setState(target_state);
}

// Color-based YOLO class IDs (0=blue,1=grey,2=purple,3=red) cannot distinguish
// robot types (hero, sentry, balance, outpost). Always assume NORMAL_4 so
// yaw-jump thresholds and trajectory face-spacing are correct for standard robots.
// BALANCE_2 / OUTPOST_3 enums are kept for future use when robot-type detection
// is available (e.g. via a separate classifier or communication from the lower computer).
void Tracker::updateArmorsNum(const Armor & armor)
{
  (void)armor;  // unused until robot-type detection is available
  tracked_armors_num = ArmorsNum::NORMAL_4;
  max_match_yaw_diff_ = M_PI / 3.0;  // ~60°
}

void Tracker::handleArmorJump(const Armor & current_armor)
{
  double yaw = orientationToYaw(current_armor.pose.orientation);
  // Check if jump direction agrees with current v_yaw.
  // If the robot reversed spin, v_yaw sign will disagree with the jump direction.
  double jump_direction = angles::shortest_angular_distance(target_state(6), yaw);
  if (jump_direction * target_state(7) < 0) {
    // Spin reversal detected — zero v_yaw to prevent EKF from extrapolating in wrong direction
    target_state(7) = 0.0;
    RCLCPP_WARN(rclcpp::get_logger("armor_tracker"), "Spin reversal — v_yaw zeroed");
  }
  target_state(6) = yaw;
  updateArmorsNum(current_armor);
  // Only 4 armors has 2 radius and height
  if (tracked_armors_num == ArmorsNum::NORMAL_4) {
    dz = target_state(4) - current_armor.pose.position.z;
    target_state(4) = current_armor.pose.position.z;
    target_state(5) = 0.0;  // v_za corrupted by za snap — zero it
    std::swap(target_state(8), another_r);
    another_r = std::max(0.12, std::min(another_r, 0.4));
  }
  info_yaw_innov_signed = 0.0;
  RCLCPP_WARN(rclcpp::get_logger("armor_tracker"), "Armor jump!");

  // If position difference is larger than max_match_distance_,
  // take this case as the ekf diverged, reset the state
  auto p = current_armor.pose.position;
  Eigen::Vector3d current_p(p.x, p.y, p.z);
  Eigen::Vector3d infer_p = getArmorPositionFromState(target_state);
  if ((current_p - infer_p).norm() > max_match_distance_) {
    double r = target_state(8);
    target_state(0) = p.x + r * cos(yaw);  // xc
    target_state(1) = 0;                   // vxc
    target_state(2) = p.y + r * sin(yaw);  // yc
    target_state(3) = 0;                   // vyc
    target_state(4) = p.z;                 // za
    target_state(5) = 0;                   // vza
    target_state(7) = 0;                   // v_yaw
    ekf.resetCovariance();
    RCLCPP_ERROR(rclcpp::get_logger("armor_tracker"), "Reset State!");
  }

  ekf.setState(target_state);

  // The yaw was snapped directly, bypassing the EKF update, so the filter
  // underestimates its own uncertainty after a jump. Inflate yaw and v_yaw
  // covariance to reflect this. Skip for BALANCE_2: the ~180° jump always
  // triggers the divergence reset above which calls resetCovariance() already.
  if (tracked_armors_num != ArmorsNum::BALANCE_2) {
    ekf.inflateCovariance(6, 4.0);  // yaw:   variance x4 (~2x std-dev)
    ekf.inflateCovariance(7, 4.0);  // v_yaw: variance x4
  }
  if (tracked_armors_num == ArmorsNum::NORMAL_4) {
    ekf.inflateCovariance(4, 4.0);  // za:   variance x4 (snapped, uncertain)
    ekf.inflateCovariance(5, 4.0);  // v_za: variance x4 (zeroed above)
  }
}

double Tracker::orientationToYaw(const geometry_msgs::msg::Quaternion & q)
{
  // Get armor yaw
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  // Make yaw change continuous (-pi~pi to -inf~inf)
  yaw = last_yaw_ + angles::shortest_angular_distance(last_yaw_, yaw);
  last_yaw_ = yaw;
  return yaw;
}

Eigen::Vector3d Tracker::getArmorPositionFromState(const Eigen::VectorXd & x)
{
  // Calculate predicted position of the current armor
  double xc = x(0), yc = x(2), za = x(4);
  double yaw = x(6), r = x(8);
  double xa = xc - r * cos(yaw);
  double ya = yc - r * sin(yaw);
  return Eigen::Vector3d(xa, ya, za);
}

}  // namespace rm_auto_aim
