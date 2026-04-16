// ============================================================================
// tracker_manager.cpp — Coordinates multiple Tracker instances in one frame.
//
// All functions are static — they operate on a TrackerStore passed in.
//
// ENTRYPOINTS (called by AutoAimNode::processFrame in pipeline order):
//   predictAll         → EKF predict all active trackers
//   assignDetections   → Mahalanobis-based detection assignment
//   updateMatched      → EKF update each tracker with its detections
//   spawnFromUnmatched → create new trackers for novel targets
//   pruneLost          → prune LOST-state trackers from the store
//   collectSnapshots   → extract state snapshots for planning
// ============================================================================
#include "auto_aim_targeting/tracking/tracker_manager.hpp"

#include <algorithm>
#include <cmath>

#include <rclcpp/logging.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace rm_auto_aim
{

namespace
{

geometry_msgs::msg::Point estimateCenterFromArmor(
  const auto_aim_interfaces::msg::Armor & armor,
  double assumed_radius)
{
  tf2::Quaternion q;
  tf2::fromMsg(armor.pose.orientation, q);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  geometry_msgs::msg::Point center = armor.pose.position;
  center.x += assumed_radius * std::cos(yaw);
  center.y += assumed_radius * std::sin(yaw);
  return center;
}

}  // namespace

void Trackers::predictAll(
  TrackerStore & trackers,
  double dt,
  double lost_time_thres)
{
  for (auto & kv : trackers) {
    auto & tracker = *kv.second;
    if (tracker.tracker_state == Tracker::LOST) {
      continue;
    }
    tracker.lost_thres = static_cast<int>(lost_time_thres / dt);
    tracker.computeAdaptiveDamping(dt);
    tracker.predictStep();
  }
}

std::vector<int> Trackers::assignDetections(
  TrackerStore & trackers,
  const auto_aim_interfaces::msg::Armors::SharedPtr & armors,
  double max_assignment_cost)
{
  std::vector<int> detection_owner(armors->armors.size(), -1);

  for (size_t i = 0; i < armors->armors.size(); ++i) {
    const auto & armor = armors->armors[i];
    double best_maha = max_assignment_cost;
    int best_tid = -1;

    for (auto & kv : trackers) {
      auto & tracker = *kv.second;
      if (tracker.tracker_state == Tracker::LOST || armor.number != tracker.tracked_id) {
        continue;
      }

      double maha = tracker.computeAssignmentCost(armor);
      if (tracker.tracker_state == Tracker::DETECTING) {
        maha *= 4.0;
      }
      if (maha < best_maha) {
        best_maha = maha;
        best_tid = kv.first;
      }
    }

    detection_owner[i] = best_tid;
  }

  return detection_owner;
}

void Trackers::updateMatched(
  TrackerStore & trackers,
  const auto_aim_interfaces::msg::Armors::SharedPtr & armors,
  const std::vector<int> & detection_owners)
{
  for (auto & kv : trackers) {
    auto & tracker = *kv.second;
    if (tracker.tracker_state == Tracker::LOST) {
      continue;
    }

    auto tracker_armors = std::make_shared<auto_aim_interfaces::msg::Armors>();
    tracker_armors->header = armors->header;
    for (size_t i = 0; i < armors->armors.size(); ++i) {
      if (detection_owners[i] == kv.first) {
        tracker_armors->armors.push_back(armors->armors[i]);
      }
    }
    tracker.update(tracker_armors);
  }
}

void Trackers::spawnFromUnmatched(
  TrackerStore & trackers,
  int & next_tracker_id,
  const auto_aim_interfaces::msg::Armors::SharedPtr & armors,
  const std::vector<int> & detection_owners,
  int max_trackers,
  double new_tracker_min_dist,
  double new_tracker_assumed_radius,
  const std::function<std::unique_ptr<Tracker>()> & create_tracker,
  const rclcpp::Time & stamp,
  const rclcpp::Logger & logger)
{
  for (size_t i = 0; i < armors->armors.size(); ++i) {
    if (detection_owners[i] >= 0) {
      continue;
    }

    const auto & armor = armors->armors[i];
    const auto provisional_center =
      estimateCenterFromArmor(armor, new_tracker_assumed_radius);
    bool too_close = false;
    for (const auto & kv : trackers) {
      const auto & tracker = *kv.second;
      if (tracker.tracker_state == Tracker::LOST) {
        continue;
      }
      const double dx = provisional_center.x - tracker.target_state(0);
      const double dy = provisional_center.y - tracker.target_state(2);
      const double dz = provisional_center.z - tracker.target_state(4);
      if (std::sqrt(dx * dx + dy * dy + dz * dz) < new_tracker_min_dist) {
        too_close = true;
        break;
      }
    }
    if (too_close || static_cast<int>(trackers.size()) >= max_trackers) {
      continue;
    }

    auto new_tracker = create_tracker();
    new_tracker->initFromArmor(armor);
    const int tracker_id = next_tracker_id++;
    new_tracker->tracker_id = tracker_id;
    new_tracker->setLastMeasurementTime(stamp);
    trackers[tracker_id] = std::move(new_tracker);
    RCLCPP_INFO(logger, "New tracker %d for id=%s", tracker_id, armor.number.c_str());
  }
}

void Trackers::pruneLost(TrackerStore & trackers, const rclcpp::Logger & logger)
{
  for (auto it = trackers.begin(); it != trackers.end();) {
    if (it->second->tracker_state == Tracker::LOST) {
      RCLCPP_INFO(logger, "Removing LOST tracker %d", it->first);
      it = trackers.erase(it);
    } else {
      ++it;
    }
  }
}

Tracker * Trackers::findTracker(TrackerStore & trackers, int tracker_id)
{
  const auto it = trackers.find(tracker_id);
  return it == trackers.end() ? nullptr : it->second.get();
}

const Tracker * Trackers::findTracker(const TrackerStore & trackers, int tracker_id)
{
  const auto it = trackers.find(tracker_id);
  return it == trackers.end() ? nullptr : it->second.get();
}

std::vector<TrackSnapshot> Trackers::collectSnapshots(
  TrackerStore & trackers,
  double dt)
{
  std::vector<TrackSnapshot> snapshots;
  snapshots.reserve(trackers.size());

  for (auto & kv : trackers) {
    auto & tracker = *kv.second;
    if (tracker.tracker_state == Tracker::LOST) {
      continue;
    }
    tracker.updateSmoothedAcceleration(dt);
    snapshots.push_back(tracker.snapshot());
  }
  return snapshots;
}

}  // namespace rm_auto_aim
