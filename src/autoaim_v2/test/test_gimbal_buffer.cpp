#include <gtest/gtest.h>

#include "autoaim_v2/gimbal_buffer.hpp"

using aim::Clock;
using aim::GimbalBuffer;
using aim::GimbalSample;

TEST(GimbalBuffer, InterpolatesBetweenSamples)
{
  GimbalBuffer buf;
  const auto t0 = Clock::now();

  for (int i = 0; i <= 10; i++) {
    GimbalSample s;
    s.t = t0 + std::chrono::milliseconds(i);
    s.yaw = 0.1 * i;
    s.pitch = -0.01 * i;
    s.yaw_raw = 100.0 + 0.1 * i;
    s.pitch_raw = 0.01 * i;
    buf.push(s);
  }

  GimbalSample q;
  ASSERT_TRUE(buf.query(t0 + std::chrono::microseconds(4500), q));
  EXPECT_NEAR(q.yaw, 0.45, 1e-9);
  EXPECT_NEAR(q.pitch, -0.045, 1e-9);
  EXPECT_NEAR(q.yaw_raw, 100.45, 1e-9);

  // Clamp beyond newest / oldest.
  ASSERT_TRUE(buf.query(t0 + std::chrono::seconds(1), q));
  EXPECT_NEAR(q.yaw, 1.0, 1e-9);
  ASSERT_TRUE(buf.query(t0 - std::chrono::seconds(1), q));
  EXPECT_NEAR(q.yaw, 0.0, 1e-9);
}

TEST(GimbalBuffer, WrapsAngleInterpolation)
{
  GimbalBuffer buf;
  const auto t0 = Clock::now();
  GimbalSample a, b;
  a.t = t0;
  a.yaw = M_PI - 0.05;
  b.t = t0 + std::chrono::milliseconds(1);
  b.yaw = -M_PI + 0.05;  // 0.1 rad ahead through the wrap
  buf.push(a);
  buf.push(b);

  GimbalSample q;
  ASSERT_TRUE(buf.query(t0 + std::chrono::microseconds(500), q));
  EXPECT_NEAR(std::fabs(q.yaw), M_PI, 0.011);
}
