#ifndef AUTO_AIM_TARGETING__TRACKING__TRACKER_RUNTIME_FACTORY_HPP_
#define AUTO_AIM_TARGETING__TRACKING__TRACKER_RUNTIME_FACTORY_HPP_

#include <functional>
#include <memory>
#include <utility>

#include "auto_aim_targeting/config.hpp"
#include "auto_aim_targeting/tracking/tracker.hpp"

namespace rm_auto_aim
{

class TrackerFactory
{
public:
  using DtProvider = std::function<double()>;

  TrackerFactory(TrackingRuntimeConfig config, DtProvider dt_provider)
  : config_(std::move(config)), dt_provider_(std::move(dt_provider)) {}

  std::unique_ptr<Tracker> create() const;

private:
  TrackingRuntimeConfig config_;
  DtProvider dt_provider_;
};

}  // namespace rm_auto_aim

#endif  // AUTO_AIM_TARGETING__TRACKING__TRACKER_RUNTIME_FACTORY_HPP_
