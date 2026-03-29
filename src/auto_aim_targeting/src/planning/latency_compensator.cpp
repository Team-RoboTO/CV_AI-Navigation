// ============================================================================
// latency_compensator.cpp — Adaptive estimation of camera-to-processing delay.
//
// ENTRYPOINTS:
//   updateFromFrameStamp → EMA update of time_bias from frame timestamps
//   transportDelay       → raw frame-to-now delay (no filtering)
//   currentBias          → current smoothed latency estimate (inline in .hpp)
// ============================================================================
#include "auto_aim_targeting/planning/latency_compensator.hpp"

#include <algorithm>
#include <cmath>

namespace rm_auto_aim
{

void LatencyCompensator::updateFromFrameStamp(
  const rclcpp::Time & now,
  const rclcpp::Time & frame_stamp)
{
  const double measured_latency = (now - frame_stamp).seconds();
  if (measured_latency <= 0.001 || measured_latency >= 0.5) {
    return;
  }

  const double residual = measured_latency - this->time_bias_;
  const double sigma = std::sqrt(std::max(this->variance_, 1e-8));
  const bool accept =
    (this->warmup_count_ < this->warmup_samples_) ||
    (std::abs(residual) < this->gate_sigma_ * sigma);

  if (!accept) {
    return;
  }

  this->time_bias_ = this->alpha_ * measured_latency + (1.0 - this->alpha_) * this->time_bias_;
  const double new_residual = measured_latency - this->time_bias_;
  this->variance_ =
    this->alpha_ * new_residual * new_residual + (1.0 - this->alpha_) * this->variance_;
  ++this->warmup_count_;
}

double LatencyCompensator::transportDelay(
  const rclcpp::Time & now,
  const rclcpp::Time & frame_stamp) const
{
  return std::max(0.0, (now - frame_stamp).seconds());
}

}  // namespace rm_auto_aim
