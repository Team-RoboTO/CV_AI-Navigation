// ============================================================================
// ballistics_solver.cpp — Iterative bullet flight solver (gravity + drag).
//
// ENTRYPOINT: BallisticsSolver::solve()
//   Given ground distance, target height, and muzzle speed, iteratively
//   converges on the required pitch angle and flight time.
//   Supports linear drag (slow projectiles) and quadratic drag (fast).
// ============================================================================
#include "auto_aim_targeting/planning/ballistics_solver.hpp"

#include <algorithm>
#include <cmath>

namespace rm_auto_aim
{

BallisticSolution BallisticsSolver::solve(
  double ground_dist,
  double target_z,
  double muzzle_speed) const
{
  // The goal here is to iteratively find the flight time and required pitch.
  // The drag slows down the bullet, meaning it takes longer to arrive,
  // which means gravity pulls it down more!

  BallisticSolution out;
  if (ground_dist < 1e-4 || muzzle_speed < 1e-4) {
    return out;
  }

  double t = ground_dist / muzzle_speed;
  double pitch = 0.0;
  constexpr int kMaxIter = 12;

  for (int i = 0; i < kMaxIter; ++i) {
    const double dz = 0.5 * this->params_.gravity * t * t;
    pitch = std::atan2(target_z + dz, ground_dist);
    pitch = std::clamp(pitch, this->params_.pitch_min, this->params_.pitch_max);

    const double v_h = muzzle_speed * std::cos(pitch);
    if (v_h < 1e-4) {
      return out;
    }

    double new_t = 0.0;

    if (this->params_.use_quadratic_drag) {
      // ------ QUADRATIC DRAG (Strada B) ------
      // Used for higher speeds where air resistance scales with velocity squared.
      // Faster, but causes dramatic bullet drop at long distances.
      // Quadratic-drag-inspired horizontal time-of-flight approximation.
      // The effective coefficient acts over distance, so the implementation
      // expects a parameter with behavior close to [1/m].
      const double k_q = this->params_.quadratic_drag_coeff;
      if (k_q < 1e-6) {
        new_t = ground_dist / v_h;
      } else {
        new_t = (std::exp(k_q * ground_dist) - 1.0) / (k_q * v_h);
      }
    } else {
      // ------ LINEAR DRAG (Strada A) ------
      // Standard linear approximation where drag scales directly with velocity.
      // Works well for relatively slow projectiles or short distances.
      // Linear-drag-inspired horizontal time-of-flight approximation.
      // The effective coefficient acts like a first-order decay rate on the
      // horizontal speed, so the implementation expects a parameter with
      // behavior close to [1/s].
      const double k_l = this->params_.linear_drag_coeff;
      if (k_l < 1e-6) {
        new_t = ground_dist / v_h;
      } else {
        const double alpha = k_l * ground_dist / v_h;
        if (alpha >= 1.0) {
          return out;
        }
        new_t = -std::log(1.0 - alpha) / k_l;
      }
    }

    if (!std::isfinite(new_t)) {
      return out;
    }
    if (std::abs(new_t - t) < 1e-4) {
      t = new_t;
      break;
    }
    t = new_t;
  }

  const double unclamped_pitch =
    std::atan2(target_z + 0.5 * this->params_.gravity * t * t, ground_dist);

  out.pitch = pitch;
  out.flight_time = t;
  
  // REACHABLE: Physical constraint - can the robot actually tilt its head this high/low?
  out.reachable =
    unclamped_pitch >= this->params_.pitch_min &&
    unclamped_pitch <= this->params_.pitch_max;
    
  // VALID: Numerical constraint - did the math produce a real number without hitting an asymptote?
  out.valid = std::isfinite(out.pitch) && std::isfinite(out.flight_time);
  return out;
}

}  // namespace rm_auto_aim
