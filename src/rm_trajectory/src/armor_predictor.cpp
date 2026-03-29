#include "rm_trajectory/armor_predictor.hpp"

#include <angles/angles.h>

#include <algorithm>
#include <optional>
#include <cmath>
#include <limits>

namespace rm_auto_aim
{

FaceGeometry ArmorPredictor::buildFaceGeometry(
  const auto_aim_interfaces::msg::Target & target,
  const PredictedCenter & center,
  double body_yaw,
  int face_index) const
{
  FaceGeometry out;
  out.face_index = face_index;

  const int faces = std::max<int>(target.armors_num, 1);
  const double face_spacing = 2.0 * M_PI / faces;
  out.alternate_pair = (faces == 4) && (face_index % 2 == 1);
  out.radius = out.alternate_pair ? target.radius_2 : target.radius_1;
  out.dz_offset = out.alternate_pair ? target.dz : 0.0;
  out.face_yaw = body_yaw + face_index * face_spacing;

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

double ArmorPredictor::nextAlignmentTime(
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




ShotPlan ArmorPredictor::buildPlan(
  const auto_aim_interfaces::msg::Target & target,
  const SolverContext & ctx,
  const FaceGeometry & face,
  double predict_time,
  bool indirect,
  double timing_residual) const
{
  ShotPlan out;
  out.tracker_id = target.tracker_id;
  out.face_index = face.face_index;
  out.indirect = indirect;
  out.range = face.range;
  out.predict_time = predict_time;
  out.absolute_yaw = face.bearing;
  out.visibility = face.visibility;
  out.measurement_age = 0.0;
  out.timing_residual = timing_residual;
  out.temp_lost =
    target.tracker_state == auto_aim_interfaces::msg::Target::TEMP_LOST;
  out.tracking_valid = target.tracking;
  out.x = face.x;
  out.y = face.y;
  out.z = face.z;

  const BallisticSolution ballistic = ballistics_.solve(
    face.ground_dist, face.z - ctx.gimbal_height, ctx.bullet_speed);
  out.absolute_pitch = ballistic.pitch;
  out.flight_time = ballistic.flight_time;
  out.ballistic_valid = ballistic.valid;
  out.reachable = ballistic.reachable;
  out.relative_yaw =
    angles::shortest_angular_distance(ctx.current_yaw, out.absolute_yaw);
  out.relative_pitch = out.absolute_pitch - ctx.current_pitch;

  const double sigma_vyaw = std::sqrt(std::max(target.v_yaw_variance, 1e-8));
  const double sigma_spin = sigma_vyaw * std::max(predict_time, 0.0);
  const double sigma_cross = crossRangeSigma(target, face.bearing);
  const double sigma_pos = sigma_cross / std::max(face.range, 0.5);
  out.sigma_angular = std::sqrt(sigma_spin * sigma_spin + sigma_pos * sigma_pos);

  const double dist_scale =
    std::min(ctx.angular_window_ref_dist / std::max(face.range, 0.5), 2.0);
  const double effective_window =
    std::min(ctx.angular_window * dist_scale + out.sigma_angular, M_PI / std::max(1, target.armors_num));
  const double face_error = std::abs(
    angles::shortest_angular_distance(out.absolute_yaw, face.face_yaw));
  out.fire_window_margin = effective_window - face_error;

  return out;
}




std::optional<ShotPlan> ArmorPredictor::solveDirect(
  const auto_aim_interfaces::msg::Target & target,
  const SolverContext & ctx) const
{
  // DIRECT MODE: Shooting at the armor plate that is currently facing us.
  // The goal is not to predict the next rotation, but to hit it before it rotates away.

  ShotPlan best;
  bool found = false;

  const int faces = std::max<int>(target.armors_num, 1);
  double predict_t =
    std::max(0.0, ctx.time_bias + ctx.gimbal_response_delay + 1.5 / std::max(ctx.bullet_speed, 1.0));

  for (int i = 0; i < faces; ++i) {
    ShotPlan candidate;
    bool valid = false;
    double local_t = predict_t;

    for (int iter = 0; iter < 2; ++iter) {
      const PredictedCenter center = predictCenter(target, local_t);
      const FaceGeometry face = buildFaceGeometry(
        target, center, target.yaw + target.v_yaw * local_t, i);

      candidate = buildPlan(target, ctx, face, local_t, false, 0.0);
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

    const bool better =
      !found ||
      (candidate.fire_window_margin > best.fire_window_margin + 1e-9) ||
      (std::abs(candidate.fire_window_margin - best.fire_window_margin) < 1e-9 &&
       candidate.reachable && !best.reachable) ||
      (std::abs(candidate.fire_window_margin - best.fire_window_margin) < 1e-9 &&
       candidate.reachable == best.reachable &&
       candidate.visibility > best.visibility + 1e-9) ||
      (std::abs(candidate.fire_window_margin - best.fire_window_margin) < 1e-9 &&
       std::abs(candidate.visibility - best.visibility) < 1e-9 &&
       candidate.range < best.range);

    if (better) {
      best = candidate;
      found = true;
    }
  }

  if (found) return best;
  return std::nullopt;
}




std::optional<ShotPlan> ArmorPredictor::solveIndirect(
  const auto_aim_interfaces::msg::Target & target,
  const SolverContext & ctx) const
{
  // INDIRECT MODE: The target is spinning fast!
  // Instead of chasing the plate circularly, we aim at a fixed spot and fire ahead of time
  // so the plate rotates exactly into our bullet when it arrives.

  ShotPlan best;
  bool found = false;

  const int faces = std::max<int>(target.armors_num, 1);
  const double face_spacing = 2.0 * M_PI / faces;
  const double abs_vyaw = std::max(std::abs(target.v_yaw), 0.1);
  const double period = 2.0 * M_PI / abs_vyaw;

  int produced = 0;

  const double seed_dt =
    std::max(0.0, ctx.time_bias + ctx.gimbal_response_delay + 1.0 / std::max(ctx.bullet_speed, 1.0));
  const PredictedCenter seed_center = predictCenter(target, seed_dt);
  const double seed_bearing = std::atan2(seed_center.y, seed_center.x);

  for (int i = 0; i < faces && produced < ctx.indirect_max_candidates; ++i) {
    const double ta0 = nextAlignmentTime(target.yaw, target.v_yaw, i, face_spacing, seed_bearing);
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
        const FaceGeometry face = buildFaceGeometry(
          target, center, target.yaw + target.v_yaw * align_t, i);

        const double seed_residual =
          std::abs(align_t - (ctx.time_bias + ctx.gimbal_response_delay));
        candidate = buildPlan(target, ctx, face, align_t, true, seed_residual);
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

      const bool better =
        !found ||
        (candidate.timing_residual < best.timing_residual - 1e-9) ||
        (std::abs(candidate.timing_residual - best.timing_residual) < 1e-9 &&
         candidate.reachable && !best.reachable) ||
        (std::abs(candidate.timing_residual - best.timing_residual) < 1e-9 &&
         candidate.reachable == best.reachable &&
         candidate.fire_window_margin > best.fire_window_margin + 1e-9) ||
        (std::abs(candidate.timing_residual - best.timing_residual) < 1e-9 &&
         std::abs(candidate.fire_window_margin - best.fire_window_margin) < 1e-9 &&
         candidate.range < best.range);

      if (better) {
        best = candidate;
        found = true;
      }

      ++produced;
    }
  }

  if (found) return best;
  return std::nullopt;
}

}  // namespace rm_auto_aim
