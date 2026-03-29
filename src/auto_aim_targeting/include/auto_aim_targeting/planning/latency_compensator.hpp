#ifndef AUTO_AIM_TARGETING__PLANNING__LATENCY_COMPENSATOR_HPP_
#define AUTO_AIM_TARGETING__PLANNING__LATENCY_COMPENSATOR_HPP_

#include <rclcpp/time.hpp>

namespace rm_auto_aim
{

class LatencyCompensator
{
public:
  LatencyCompensator(
    double initial_bias,
    double alpha,
    double gate_sigma,
    int warmup_samples,
    double initial_variance = 0.0009)
  : time_bias_(initial_bias),
    alpha_(alpha),
    gate_sigma_(gate_sigma),
    warmup_samples_(warmup_samples),
    variance_(initial_variance) {}

  void updateFromFrameStamp(const rclcpp::Time & now, const rclcpp::Time & frame_stamp);
  double transportDelay(const rclcpp::Time & now, const rclcpp::Time & frame_stamp) const;
  double currentBias() const { return this->time_bias_; }

private:
  double time_bias_;
  double alpha_;
  double gate_sigma_;
  int warmup_samples_;
  int warmup_count_ = 0;
  double variance_;
};

}  // namespace rm_auto_aim

#endif  // AUTO_AIM_TARGETING__PLANNING__LATENCY_COMPENSATOR_HPP_
