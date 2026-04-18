// ============================================================================
// shot_planner.cpp — Solves "where to aim" for each target.
//
// Predicts future armor positions and computes ballistic solutions.
//
// ENTRYPOINTS (called by EngagementPlanner::selectBestPlan):
//   solveVisibleDirect   → aim at the currently matched (visible) face
//   solvePredictedDirect → aim at the best face by state prediction
//   solveIndirect        → fire ahead to hit a spinning plate as it rotates in
//
// HELPERS (stepdown — called by the entrypoints above):
//   buildFaceGeometry    → predict a face's position/orientation at future time
//   nextAlignmentTime    → when a face will point at the camera (for indirect)
//   buildPlan            → assemble a ShotPlan from face geometry + ballistics
// ============================================================================
#include "auto_aim_targeting/planning/shot_planner.hpp"

#include <angles/angles.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

namespace rm_auto_aim
{

std::optional<ShotPlan> ShotPlanner::solveVisibleDirect(
  const TrackSnapshot & target,
  const PlanningContext & ctx) const
{
  if (!target.matched_face.valid || !target.matched_face.fresh || !target.matched_face.observable) {
    return std::nullopt;
  }

  ShotPlan best;
  bool found = false;
  double local_t =
    std::max(0.0, ctx.time_bias + ctx.gimbal_response_delay + 1.5 / std::max(ctx.bullet_speed, 1.0));

  for (int iter = 0; iter < 3; ++iter) {
    const PredictedCenter center = predictCenter(target, local_t);
    const FaceGeometry face = this->buildFaceGeometry(
      target, center, target.yaw + target.v_yaw * local_t, target.matched_face.face_index);

    ShotPlan candidate = this->buildPlan(target, ctx, face, local_t, false, 0.0);
    candidate.mode = AimMode::VISIBLE_DIRECT;
    if (!candidate.ballistic_valid) {
      return std::nullopt;
    }

    best = candidate;
    found = true;

    const double refined_t =
      candidate.flight_time + ctx.time_bias + ctx.gimbal_response_delay;
    if (std::abs(refined_t - local_t) < 1e-3) {
      break;
    }
    local_t = refined_t;
  }

  if (!found) {
    return std::nullopt;
  }
  return best;
}

std::optional<ShotPlan> ShotPlanner::solvePredictedDirect(
  const TrackSnapshot & target,
  const PlanningContext & ctx) const
{
  ShotPlan best;
  bool found = false;

  const int faces = std::max(target.armors_num, 1);
  double predict_t =
    std::max(0.0, ctx.time_bias + ctx.gimbal_response_delay + 1.5 / std::max(ctx.bullet_speed, 1.0));

  for (int i = 0; i < faces; ++i) {
    ShotPlan candidate;
    bool valid = false;
    double local_t = predict_t;

    for (int iter = 0; iter < 3; ++iter) {
      const PredictedCenter center = predictCenter(target, local_t);
      const FaceGeometry face = this->buildFaceGeometry(
        target, center, target.yaw + target.v_yaw * local_t, i);

      candidate = this->buildPlan(target, ctx, face, local_t, false, 0.0);
      candidate.mode = AimMode::PREDICTED_DIRECT;
      if (!candidate.ballistic_valid) {
        valid = false;
        break;
      }
      valid = true;

      const double refined_t =
        candidate.flight_time + ctx.time_bias + ctx.gimbal_response_delay;
      if (std::abs(refined_t - local_t) < 1e-3) {
        break;
      }
      local_t = refined_t;
    }

    if (!valid) {
      continue;
    }

    if (isBetterCandidate(candidate, best, found)) {
      best = candidate;
      found = true;
    }
  }

  if (found) {
    return best;
  }
  return std::nullopt;
}

std::optional<ShotPlan> ShotPlanner::solveIndirect(
  const TrackSnapshot & target,
  const PlanningContext & ctx) const
{
  ShotPlan best;
  bool found = false;

  const int faces = std::max(target.armors_num, 1);
  const double face_spacing = 2.0 * M_PI / faces;
  const double abs_vyaw = std::max(std::abs(target.v_yaw), 0.1);
  const double period = 2.0 * M_PI / abs_vyaw;

  int produced = 0;

  const double seed_dt =
    std::max(0.0, ctx.time_bias + ctx.gimbal_response_delay + 1.0 / std::max(ctx.bullet_speed, 1.0));
  const PredictedCenter seed_center = predictCenter(target, seed_dt);
  const double seed_bearing = std::atan2(seed_center.y, seed_center.x);

  for (int i = 0; i < faces && produced < ctx.indirect_max_candidates; ++i) {
    const double ta0 = this->nextAlignmentTime(target.yaw, target.v_yaw, i, face_spacing, seed_bearing);
    if (!std::isfinite(ta0)) {
      continue;
    }

    for (int occ = 0; occ < 2 && produced < ctx.indirect_max_candidates; ++occ) {
      double align_t = ta0 + occ * period;
      if (align_t < 0.001 || align_t > 2.0) {
        continue;
      }

      ShotPlan candidate;
      bool valid = false;

      for (int iter = 0; iter < 2; ++iter) {
        const PredictedCenter center = predictCenter(target, align_t);
        const FaceGeometry face = this->buildFaceGeometry(
          target, center, target.yaw + target.v_yaw * align_t, i);

        const double seed_residual =
          std::abs(align_t - (ctx.time_bias + ctx.gimbal_response_delay));
        candidate = this->buildPlan(target, ctx, face, align_t, true, seed_residual);
        candidate.mode = AimMode::INDIRECT;
        if (!candidate.ballistic_valid) {
          valid = false;
          break;
        }
        valid = true;

        const double desired_t =
          candidate.flight_time + ctx.time_bias + ctx.gimbal_response_delay;
        candidate.timing_residual = std::abs(align_t - desired_t);

        const double timing_sigma =
          candidate.sigma_angular / std::max(std::abs(target.v_yaw), 0.1);
        const double effective_tol =
          std::max(ctx.indirect_timing_tolerance, timing_sigma);

        candidate.fire_window_margin = std::min(
          candidate.fire_window_margin,
          effective_tol - candidate.timing_residual);

        const double occ_index = std::round((desired_t - ta0) / period);
        const double refined_align_t = ta0 + std::max(0.0, occ_index) * period;

        if (std::abs(refined_align_t - align_t) < 1e-3) {
          break;
        }
        align_t = refined_align_t;
      }

      if (!valid) {
        ++produced;
        continue;
      }

      if (isBetterCandidate(candidate, best, found)) {
        best = candidate;
        found = true;
      }

      ++produced;
    }
  }

  if (found) {
    return best;
  }
  return std::nullopt;
}

bool ShotPlanner::isBetterCandidate(
  const ShotPlan & candidate,
  const ShotPlan & best,
  bool found)
{
  if (!found) {
    return true;
  }

  // Primary criterion depends on mode:
  //   Direct modes:  fire_window_margin  (higher is better)
  //   Indirect mode: timing_residual     (lower  is better)
  //
  // Secondary criterion (first tie-breaker after reachable):
  //   Direct modes:  visibility          (higher is better)
  //   Indirect mode: fire_window_margin  (higher is better)
  double primary_cand, primary_best;
  double secondary_cand, secondary_best;

  if (candidate.indirect) {
    // Indirect: prefer lower timing_residual (negate so "higher is better")
    primary_cand = -candidate.timing_residual;
    primary_best = -best.timing_residual;
    secondary_cand = candidate.fire_window_margin;
    secondary_best = best.fire_window_margin;
  } else {
    // Direct: prefer higher fire_window_margin
    primary_cand = candidate.fire_window_margin;
    primary_best = best.fire_window_margin;
    secondary_cand = candidate.visibility;
    secondary_best = best.visibility;
  }

  if (primary_cand > primary_best + 1e-9) {
    return true;
  }
  if (std::abs(primary_cand - primary_best) < 1e-9 &&
      candidate.reachable && !best.reachable) {
    return true;
  }
  if (std::abs(primary_cand - primary_best) < 1e-9 &&
      candidate.reachable == best.reachable &&
      secondary_cand > secondary_best + 1e-9) {
    return true;
  }
  if (std::abs(primary_cand - primary_best) < 1e-9 &&
      std::abs(secondary_cand - secondary_best) < 1e-9 &&
      candidate.range < best.range) {
    return true;
  }
  return false;
}

FaceGeometry ShotPlanner::buildFaceGeometry(
  const TrackSnapshot & target,
  const PredictedCenter & center,
  double matched_face_yaw,
  int face_index) const
{
  FaceGeometry out;
  out.face_index = face_index;

  const int faces = std::max(target.armors_num, 1);
  const double face_spacing = 2.0 * M_PI / faces;
  out.alternate_pair = (faces == 4) && (std::abs(face_index) % 2 == 1);
  out.radius = out.alternate_pair ? target.radius_2 : target.radius_1;
  out.dz_offset = out.alternate_pair ? target.dz : 0.0;
  out.face_yaw = matched_face_yaw + face_index * face_spacing;

  out.x = center.x - out.radius * std::cos(out.face_yaw);
  out.y = center.y - out.radius * std::sin(out.face_yaw);
  out.z = center.z + out.dz_offset;
  out.ground_dist = std::hypot(out.x, out.y);
  out.range = std::sqrt(out.x * out.x + out.y * out.y + out.z * out.z);
  out.bearing = std::atan2(out.y, out.x);

  const double face_to_camera = std::atan2(-out.y, -out.x);
  const double face_normal = out.face_yaw + M_PI;
  const double oblique =
    std::abs(angles::shortest_angular_distance(face_normal, face_to_camera));
  out.visibility = std::clamp(std::cos(oblique), 0.0, 1.0);
  return out;
}

double ShotPlanner::nextAlignmentTime(
  double body_yaw,
  double body_vyaw,
  double face_index,
  double face_spacing,
  double bearing) const
{
  if (std::abs(body_vyaw) < 0.1) {
    return std::numeric_limits<double>::infinity();
  }

  const double face_angle = body_yaw + face_index * face_spacing;
  double dt = angles::shortest_angular_distance(face_angle, bearing) / body_vyaw;
  const double period = 2.0 * M_PI / std::abs(body_vyaw);
  while (dt < 0.0) {
    dt += period;
  }
  return dt;
}

ShotPlan ShotPlanner::buildPlan(
  const TrackSnapshot & target,
  const PlanningContext & ctx,
  const FaceGeometry & face,
  double predict_time,
  bool indirect,
  double timing_residual) const
{
  ShotPlan out;
  out.tracker_id = target.tracker_id;
  out.face_index = face.face_index;
  out.mode = indirect ? AimMode::INDIRECT : AimMode::PREDICTED_DIRECT;
  out.indirect = indirect;
  out.range = face.range;
  out.predict_time = predict_time;
  out.absolute_yaw = face.bearing;
  out.visibility = face.visibility;
  out.measurement_age = 0.0;
  out.timing_residual = timing_residual;
  out.temp_lost = target.temp_lost;
  out.tracking_valid = target.tracking;
  out.x = face.x;
  out.y = face.y;
  out.z = face.z;

  const BallisticSolution ballistic = this->ballistics_.solve(
    face.ground_dist, face.z - ctx.gimbal_height, ctx.bullet_speed);
  out.absolute_pitch = ballistic.pitch;
  out.flight_time = ballistic.flight_time;
  out.ballistic_valid = ballistic.valid;
  out.reachable = ballistic.reachable;
  out.relative_yaw =
    angles::shortest_angular_distance(ctx.current_yaw, out.absolute_yaw);
  out.relative_pitch = out.absolute_pitch - ctx.current_pitch;

  const double yaw_variance = std::max(target.yaw_variance, 0.0);
  const double vyaw_variance = std::max(target.v_yaw_variance, 0.0);
  const double sigma_yaw =
    std::sqrt(std::max(yaw_variance + vyaw_variance * predict_time * predict_time, 1e-8));
  const double sigma_spin =
    face.radius * sigma_yaw / std::max(face.range, 0.5);
  const double sigma_cross = crossRangeSigma(target, face.bearing);
  const double sigma_pos = sigma_cross / std::max(face.range, 0.5);
  out.sigma_angular = std::sqrt(sigma_spin * sigma_spin + sigma_pos * sigma_pos);

  const double dist_scale =
    std::min(ctx.angular_window_ref_dist / std::max(face.range, 0.5), 2.0);
  const double effective_window =
    std::max(
      0.0,
      std::min(
        ctx.angular_window * dist_scale,
        M_PI / std::max(1, target.armors_num)) - out.sigma_angular);
  const double face_error = std::abs(
    angles::shortest_angular_distance(out.absolute_yaw, face.face_yaw));
  out.fire_window_margin = effective_window - face_error;

  return out;
}

}  // namespace rm_auto_aim
