#ifndef AUTO_AIM_TARGETING__TRACKING__TRACKER_MANAGER_HPP_
#define AUTO_AIM_TARGETING__TRACKING__TRACKER_MANAGER_HPP_

#include <map>
#include <memory>
#include <functional>
#include <vector>

#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>

#include "auto_aim_targeting/tracking/tracker.hpp"

namespace rm_auto_aim
{

class Trackers
{
public:
  using TrackerStore = std::map<int, std::unique_ptr<Tracker>>;

  static void predictAll(TrackerStore & trackers, double dt, double lost_time_thres);
  static std::vector<int> assignDetections(
    TrackerStore & trackers,
    const auto_aim_interfaces::msg::Armors::SharedPtr & armors,
    double max_assignment_cost);
  static void updateMatched(
    TrackerStore & trackers,
    const auto_aim_interfaces::msg::Armors::SharedPtr & armors,
    const std::vector<int> & detection_owners);
  static void spawnFromUnmatched(
    TrackerStore & trackers,
    int & next_tracker_id,
    const auto_aim_interfaces::msg::Armors::SharedPtr & armors,
    const std::vector<int> & detection_owners,
    int max_trackers,
    double new_tracker_min_dist,
    double new_tracker_assumed_radius,
    const std::function<std::unique_ptr<Tracker>()> & create_tracker,
    const rclcpp::Time & stamp,
    const rclcpp::Logger & logger);
  static void pruneLost(TrackerStore & trackers, const rclcpp::Logger & logger);
  static Tracker * findTracker(TrackerStore & trackers, int tracker_id);
  static const Tracker * findTracker(const TrackerStore & trackers, int tracker_id);
  static std::vector<TrackSnapshot> collectSnapshots(
    TrackerStore & trackers,
    double dt);
};

}  // namespace rm_auto_aim

#endif  // AUTO_AIM_TARGETING__TRACKING__TRACKER_MANAGER_HPP_
