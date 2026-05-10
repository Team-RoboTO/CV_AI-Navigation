#ifndef AUTO_AIM__CONFIG_VALIDATOR_HPP_
#define AUTO_AIM__CONFIG_VALIDATOR_HPP_

#include <rclcpp/logger.hpp>
#include <set>
#include <string>
#include <vector>

#include "auto_aim/tracker.hpp"

namespace auto_aim
{

// Aggregate of every parameter that has a sane physical range.
// Values outside the range produce a warning. Values that would crash the
// pipeline or silently mis-aim produce an error and the node should abort.
struct ValidationResult
{
  std::vector<std::string> errors;
  std::vector<std::string> warnings;
  bool ok() const { return errors.empty(); }
};

// Static helper. Avoids holding any state.
struct ConfigValidator
{
  // Inputs:
  //   cfg                  - tracker/ballistics config struct
  //   target_classes       - set of YOLO class IDs the tracker accepts
  //   yaw_sign, pitch_sign - per-axis sign flips for the gimbal command
  //   smooth_alpha         - EMA alpha for the published yaw/pitch
  //   pitch_offset_deg     - bore-sight pitch correction [deg]
  //   yaw_offset_deg       - bore-sight yaw correction [deg]
  static ValidationResult validate(
    const TrackerConfig & cfg,
    const std::set<std::string> & target_classes,
    double yaw_sign,
    double pitch_sign,
    double smooth_alpha,
    double pitch_offset_deg,
    double yaw_offset_deg,
    double min_keypoint_score,
    double keypoint_max_reproj_error);

  // Log every error at FATAL level and every warning at WARN level. Returns
  // ok() so the caller can chain `if (!log(validate(...))) abort();`.
  static bool logResult(const ValidationResult & r, rclcpp::Logger logger);
};

}  // namespace auto_aim
#endif
