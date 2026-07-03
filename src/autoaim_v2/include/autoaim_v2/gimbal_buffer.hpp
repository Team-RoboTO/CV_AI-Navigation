#ifndef AUTOAIM_V2__GIMBAL_BUFFER_HPP_
#define AUTOAIM_V2__GIMBAL_BUFFER_HPP_

#include <algorithm>
#include <cmath>
#include <mutex>
#include <vector>

#include "autoaim_v2/types.hpp"

namespace aim
{

inline double norm_angle(double a)
{
  while (a > M_PI) a -= 2 * M_PI;
  while (a < -M_PI) a += 2 * M_PI;
  return a;
}

inline double ang_diff(double to, double from)  // shortest to - from
{
  return norm_angle(to - from);
}

// Ring buffer of gimbal angle samples at the micro's native 1 kHz rate.
// The camera->world transform for a frame must use the angles AT THE CAPTURE
// TIME: at 3 rad/s slew, 10 ms of staleness is 30 mrad = 9 cm at 3 m. With
// 1 kHz samples the interpolation error is < 1 ms worth of motion.
//
// Written by the serial RX thread, read by the aim thread. A plain mutex is
// fine: critical sections are tens of nanoseconds at 1 kHz + 120 Hz.
class GimbalBuffer
{
public:
  explicit GimbalBuffer(size_t capacity = 4096) : buf_(capacity) {}

  void push(const GimbalSample & s)
  {
    std::lock_guard<std::mutex> lk(m_);
    buf_[head_] = s;
    head_ = (head_ + 1) % buf_.size();
    if (count_ < buf_.size()) count_++;
    latest_ = s;
    has_any_ = true;
  }

  bool hasData() const
  {
    std::lock_guard<std::mutex> lk(m_);
    return has_any_;
  }

  GimbalSample latest() const
  {
    std::lock_guard<std::mutex> lk(m_);
    return latest_;
  }

  // Interpolated yaw/pitch at time t. Clamps to the oldest/newest sample when
  // t is outside the buffered range. Returns false only if the buffer is empty.
  bool query(TimePoint t, GimbalSample & out) const
  {
    std::lock_guard<std::mutex> lk(m_);
    if (count_ == 0) return false;

    // Newest sample index = head_-1; walk backwards (t is normally recent).
    auto at = [&](size_t i_back) -> const GimbalSample & {
      size_t idx = (head_ + buf_.size() - 1 - i_back) % buf_.size();
      return buf_[idx];
    };

    const GimbalSample & newest = at(0);
    if (t >= newest.t || count_ == 1) {
      out = newest;
      return true;
    }

    for (size_t i = 1; i < count_; i++) {
      const GimbalSample & older = at(i);
      const GimbalSample & newer = at(i - 1);
      if (t >= older.t) {
        double span = seconds(newer.t, older.t);
        double u = span > 1e-9 ? seconds(t, older.t) / span : 1.0;
        u = std::clamp(u, 0.0, 1.0);
        out.t = t;
        out.yaw = older.yaw + u * ang_diff(newer.yaw, older.yaw);
        out.pitch = older.pitch + u * (newer.pitch - older.pitch);
        out.yaw_raw = older.yaw_raw + u * (newer.yaw_raw - older.yaw_raw);
        out.pitch_raw = older.pitch_raw + u * (newer.pitch_raw - older.pitch_raw);
        return true;
      }
    }
    out = at(count_ - 1);  // older than everything we have
    return true;
  }

private:
  mutable std::mutex m_;
  std::vector<GimbalSample> buf_;
  size_t head_ = 0;
  size_t count_ = 0;
  GimbalSample latest_{};
  bool has_any_ = false;
};

}  // namespace aim

#endif  // AUTOAIM_V2__GIMBAL_BUFFER_HPP_
