#ifndef RM_TRAJECTORY__ARMOR_PREDICTOR_HPP_
#define RM_TRAJECTORY__ARMOR_PREDICTOR_HPP_

#include <vector>
#include <optional>

#include "rm_trajectory/ballistics_solver.hpp"
#include "rm_trajectory/trajectory_types.hpp"

namespace rm_auto_aim
{

class ArmorPredictor
{
public:
  explicit ArmorPredictor(const BallisticsSolver & ballistics)
  : ballistics_(ballistics) {}

  std::optional<ShotPlan> solveDirect(
    const auto_aim_interfaces::msg::Target & target,
    const SolverContext & ctx) const;

  std::optional<ShotPlan> solveIndirect(
    const auto_aim_interfaces::msg::Target & target,
    const SolverContext & ctx) const;

private:
  FaceGeometry buildFaceGeometry(
    const auto_aim_interfaces::msg::Target & target,
    const PredictedCenter & center,
    double body_yaw,
    int face_index) const;

  double nextAlignmentTime(
    double body_yaw,
    double body_vyaw,
    double face_index,
    double face_spacing,
    double bearing) const;

  ShotPlan buildPlan(
    const auto_aim_interfaces::msg::Target & target,
    const SolverContext & ctx,
    const FaceGeometry & face,
    double predict_time,
    bool indirect,
    double timing_residual) const;

  const BallisticsSolver & ballistics_;
};

}  // namespace rm_auto_aim

#endif  // RM_TRAJECTORY__ARMOR_PREDICTOR_HPP_
