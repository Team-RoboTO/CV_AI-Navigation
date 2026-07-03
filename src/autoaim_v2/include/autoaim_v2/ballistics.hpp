#ifndef AUTOAIM_V2__BALLISTICS_HPP_
#define AUTOAIM_V2__BALLISTICS_HPP_

namespace aim
{

// Quadratic-drag point-mass ballistics for the 17 mm / 3.2 g pellet.
//
// Why drag: versus the drag-free parabola the pellet drops an extra
// 7 mm @ 4 m, 14 mm @ 5 m, 24 mm @ 6 m and arrives 7-15 ms later — both
// significant against a 60 mm half-height plate and a spinning target
// (see DESIGN.md §2). The integrator is an RK2 with 1 ms steps inside a
// Newton iteration on pitch; residual < 0.01 mm, cost < 10 us.
struct BallisticParams
{
  double bullet_speed = 24.0;   // muzzle speed [m/s] — MEASURE on the robot
  double gravity = 9.81;
  // k/m = 0.5 * rho * Cd * A / m. 17 mm, 3.2 g, Cd 0.47 -> 0.0196 1/m.
  // Set 0.0 to recover the drag-free model.
  double drag_k = 0.0196;
};

struct BallisticSolution
{
  double pitch = 0;        // barrel elevation [rad]
  double flight_time = 0;  // [s]
  bool valid = false;
};

// Solve barrel pitch so the pellet passes through (ground_dist, dz) relative
// to the muzzle. ground_dist [m] horizontal, dz [m] vertical (up positive).
BallisticSolution solve_ballistics(const BallisticParams & p,
                                   double ground_dist, double dz);

// Trajectory height and time at horizontal distance x for a given pitch
// (exposed for tests).
bool fly(const BallisticParams & p, double pitch, double x_target,
         double & t_out, double & y_out);

}  // namespace aim

#endif  // AUTOAIM_V2__BALLISTICS_HPP_
