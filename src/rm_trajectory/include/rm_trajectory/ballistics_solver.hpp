#ifndef RM_TRAJECTORY__BALLISTICS_SOLVER_HPP_
#define RM_TRAJECTORY__BALLISTICS_SOLVER_HPP_

#include "rm_trajectory/trajectory_types.hpp"

namespace rm_auto_aim
{

struct BallisticsParams
{
  double gravity = 9.8;

  // Effective coefficient for the linear-drag-inspired horizontal model.
  // In this implementation it behaves dimensionally like a first-order decay
  // rate, so its effective unit is approximately [1/s].
  double linear_drag_coeff = 0.01;

  // Effective coefficient for the quadratic-drag-inspired horizontal model.
  // In this implementation it behaves dimensionally like an attenuation
  // coefficient over distance, so its effective unit is approximately [1/m].
  double quadratic_drag_coeff = 0.01;

  bool use_quadratic_drag = false;
  double pitch_min = -0.524;
  double pitch_max = 0.524;
};

class BallisticsSolver
{
public:
  explicit BallisticsSolver(BallisticsParams params) : params_(params) {}

  const BallisticsParams & params() const { return params_; }

  BallisticSolution solve(
    double ground_dist,
    double target_z,
    double muzzle_speed) const;

private:
  BallisticsParams params_;
};

}  // namespace rm_auto_aim

#endif  // RM_TRAJECTORY__BALLISTICS_SOLVER_HPP_
