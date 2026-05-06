#ifndef AUTO_AIM_TARGETING__PLANNING__SHOT_PLANNER_HPP_
#define AUTO_AIM_TARGETING__PLANNING__SHOT_PLANNER_HPP_

#include <vector>
#include <optional>

#include "auto_aim_targeting/planning/ballistics_solver.hpp"
#include "auto_aim_targeting/planning_types.hpp"

namespace rm_auto_aim
{

class ShotPlanner
{
public:
  explicit ShotPlanner(const BallisticsSolver & ballistics)
  : ballistics_(ballistics) {}

  std::optional<ShotPlan> solveVisibleDirect(
    const TrackSnapshot & target,
    const PlanningContext & ctx) const;

  std::optional<ShotPlan> solvePredictedDirect(
    const TrackSnapshot & target,
    const PlanningContext & ctx) const;

  std::optional<ShotPlan> solveIndirect(
    const TrackSnapshot & target,
    const PlanningContext & ctx) const;

private:
  FaceGeometry buildMeasuredFaceGeometry(
    const TrackSnapshot & target,
    double predict_time) const;

  FaceGeometry buildFaceGeometry(
    const TrackSnapshot & target,
    const PredictedCenter & center,
    double body_yaw,
    int face_index) const;

  /// Multi-criterion "is candidate better than current best?" comparison.
  ///
  /// Primary criterion differs by mode:
  ///   - Direct modes rank by fire_window_margin (higher is better — we want
  ///     the widest angular margin for a reliable hit).
  ///   - Indirect mode ranks by timing_residual (lower is better — we want
  ///     the bullet arrival to coincide with face alignment).
  ///
  /// After the primary criterion, tie-breakers are identical:
  ///   reachable > secondary_value > range (lower).
  ///
  /// secondary_value carries visibility (direct) or fire_window_margin
  /// (indirect) depending on the mode.
  static bool isBetterCandidate(
    const ShotPlan & candidate,
    const ShotPlan & best,
    bool found);

  double nextAlignmentTime(
    double body_yaw,
    double body_vyaw,
    double face_index,
    double face_spacing,
    double bearing) const;

  ShotPlan buildPlan(
    const TrackSnapshot & target,
    const PlanningContext & ctx,
    const FaceGeometry & face,
    double predict_time,
    bool indirect,
    double timing_residual) const;

  const BallisticsSolver & ballistics_;
};

}  // namespace rm_auto_aim

#endif  // AUTO_AIM_TARGETING__PLANNING__SHOT_PLANNER_HPP_
