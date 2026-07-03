#include <gtest/gtest.h>

#include "autoaim_v2/ballistics.hpp"

using aim::BallisticParams;
using aim::fly;
using aim::solve_ballistics;

TEST(Ballistics, ConvergesOverGrid)
{
  BallisticParams p;
  p.bullet_speed = 25.0;
  for (double gd = 0.5; gd <= 6.5; gd += 0.5) {
    for (double dz = -0.5; dz <= 0.5; dz += 0.1) {
      auto s = solve_ballistics(p, gd, dz);
      ASSERT_TRUE(s.valid) << "gd=" << gd << " dz=" << dz;
      double t, y;
      ASSERT_TRUE(fly(p, s.pitch, gd, t, y));
      EXPECT_NEAR(y, dz, 1e-3) << "gd=" << gd << " dz=" << dz;
      EXPECT_NEAR(t, s.flight_time, 1e-6);
      EXPECT_GT(s.flight_time, 0.0);
    }
  }
}

TEST(Ballistics, DragIncreasesFlightTimeAndPitch)
{
  BallisticParams drag, vac;
  drag.bullet_speed = vac.bullet_speed = 25.0;
  vac.drag_k = 0.0;

  auto sd = solve_ballistics(drag, 4.0, 0.0);
  auto sv = solve_ballistics(vac, 4.0, 0.0);
  ASSERT_TRUE(sd.valid && sv.valid);
  // Verified numerically: ~166.5 ms vs ~160.1 ms at 4 m, 25 m/s.
  EXPECT_GT(sd.flight_time, sv.flight_time + 0.004);
  EXPECT_LT(sd.flight_time, sv.flight_time + 0.012);
  EXPECT_GT(sd.pitch, sv.pitch);
}

TEST(Ballistics, MatchesReferenceNumbers)
{
  // Reference values from the numeric design study (math_verify.py).
  BallisticParams p;
  p.bullet_speed = 25.0;
  auto s2 = solve_ballistics(p, 2.0, 0.0);
  auto s4 = solve_ballistics(p, 4.0, 0.0);
  ASSERT_TRUE(s2.valid && s4.valid);
  EXPECT_NEAR(s2.flight_time, 0.0816, 0.002);
  EXPECT_NEAR(s4.flight_time, 0.1665, 0.003);
}

TEST(Ballistics, RejectsUnreachable)
{
  BallisticParams p;
  p.bullet_speed = 15.0;
  auto s = solve_ballistics(p, 30.0, 0.0);  // way beyond range
  EXPECT_FALSE(s.valid);
  EXPECT_FALSE(solve_ballistics(p, 0.001, 0.0).valid);
}
