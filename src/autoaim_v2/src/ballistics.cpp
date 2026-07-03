#include "autoaim_v2/ballistics.hpp"

#include <cmath>

namespace aim
{

bool fly(const BallisticParams & p, double pitch, double x_target,
         double & t_out, double & y_out)
{
  if (x_target <= 0.0) {
    t_out = 0.0;
    y_out = 0.0;
    return true;
  }

  double vx = p.bullet_speed * std::cos(pitch);
  double vy = p.bullet_speed * std::sin(pitch);
  if (vx < 1.0) return false;

  double x = 0, y = 0, t = 0;
  const double dt = 1e-3;
  const double k = p.drag_k;

  // RK2 (midpoint). At 25 m/s and <= 7 m this is < 300 steps worst case.
  for (int i = 0; i < 2000; i++) {
    double v = std::hypot(vx, vy);
    double ax = -k * v * vx;
    double ay = -p.gravity - k * v * vy;
    double vxm = vx + 0.5 * dt * ax;
    double vym = vy + 0.5 * dt * ay;
    double vm = std::hypot(vxm, vym);
    double axm = -k * vm * vxm;
    double aym = -p.gravity - k * vm * vym;

    double x_new = x + vxm * dt;
    double y_new = y + vym * dt;
    vx += axm * dt;
    vy += aym * dt;

    if (x_new >= x_target) {
      double f = (x_new > x) ? (x_target - x) / (x_new - x) : 1.0;
      t_out = t + f * dt;
      y_out = y + f * (y_new - y);
      return true;
    }
    x = x_new;
    y = y_new;
    t += dt;
    if (vx < 1.0) return false;  // stalled before reaching the target
  }
  return false;
}

BallisticSolution solve_ballistics(const BallisticParams & p,
                                   double ground_dist, double dz)
{
  BallisticSolution s;
  if (ground_dist < 0.05) return s;

  double pitch = std::atan2(dz, ground_dist);
  double t = 0, y = 0;

  for (int iter = 0; iter < 8; iter++) {
    if (!fly(p, pitch, ground_dist, t, y)) return s;
    double err = y - dz;
    if (std::fabs(err) < 1e-5) break;

    double t2 = 0, y2 = 0;
    const double eps = 1e-4;
    if (!fly(p, pitch + eps, ground_dist, t2, y2)) return s;
    double dydp = (y2 - y) / eps;
    if (std::fabs(dydp) < 1e-9) return s;
    pitch -= err / dydp;
    if (!(pitch > -1.4 && pitch < 1.4)) return s;  // physically absurd
  }

  if (!fly(p, pitch, ground_dist, t, y)) return s;
  if (std::fabs(y - dz) > 5e-3) return s;  // did not converge

  s.pitch = pitch;
  s.flight_time = t;
  s.valid = std::fabs(pitch) < 1.2;
  return s;
}

}  // namespace aim
