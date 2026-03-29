// ============================================================================
// tracker_runtime_factory.cpp — Creates fully-configured Tracker instances.
//
// ENTRYPOINT: TrackerFactory::create()
//
// Builds the EKF with injected lambdas for process model (f), observation
// model (h), their Jacobians (j_f, j_h), process noise (Q), and measurement
// noise (R).  Each lambda captures a raw pointer to the Tracker it belongs to,
// so the process model reads per-tracker damping alphas at runtime.
// ============================================================================
#include "auto_aim_targeting/tracking/tracker_runtime_factory.hpp"

#include <algorithm>
#include <cmath>

namespace rm_auto_aim
{

std::unique_ptr<Tracker> TrackerFactory::create() const
{
  auto tracker = std::make_unique<Tracker>(this->config_.tracker);
  tracker->tracking_thres = this->config_.tracker.tracking_threshold;

  Tracker * tracker_ptr = tracker.get();
  const auto refresh_frequency = this->config_.refresh_frequency;

  auto f = [tracker_ptr, dt_provider = this->dt_provider_, refresh_frequency](const Eigen::VectorXd & x) {
    Eigen::VectorXd x_new = x;
    const double dt = dt_provider();
    const double position_decay =
      std::pow(tracker_ptr->position_velocity_decay, dt * refresh_frequency);
    const double yaw_decay =
      std::pow(tracker_ptr->yaw_velocity_decay, dt * refresh_frequency);
    const double damped_vx = x(1) * position_decay;
    const double damped_vy = x(3) * position_decay;
    const double damped_vz = x(5) * position_decay;
    const double damped_vyaw = x(7) * yaw_decay;
    x_new(0) += damped_vx * dt;
    x_new(1) = damped_vx;
    x_new(2) += damped_vy * dt;
    x_new(3) = damped_vy;
    x_new(4) += damped_vz * dt;
    x_new(5) = damped_vz;
    x_new(6) += damped_vyaw * dt;
    x_new(7) = damped_vyaw;
    return x_new;
  };

  auto j_f = [tracker_ptr, dt_provider = this->dt_provider_, refresh_frequency](const Eigen::VectorXd &) {
    const double dt = dt_provider();
    const double position_decay =
      std::pow(tracker_ptr->position_velocity_decay, dt * refresh_frequency);
    const double yaw_decay =
      std::pow(tracker_ptr->yaw_velocity_decay, dt * refresh_frequency);
    Eigen::MatrixXd jacobian(9, 9);
    jacobian <<
      1, position_decay * dt, 0, 0, 0, 0, 0, 0, 0,
      0, position_decay, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 1, position_decay * dt, 0, 0, 0, 0, 0,
      0, 0, 0, position_decay, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 1, position_decay * dt, 0, 0, 0,
      0, 0, 0, 0, 0, position_decay, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 1, yaw_decay * dt, 0,
      0, 0, 0, 0, 0, 0, 0, yaw_decay, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 1;
    return jacobian;
  };

  auto h = [](const Eigen::VectorXd & x) {
    Eigen::VectorXd z(4);
    const double yaw = x(6);
    const double radius = x(8);
    z(0) = x(0) - radius * std::cos(yaw);
    z(1) = x(2) - radius * std::sin(yaw);
    z(2) = x(4);
    z(3) = x(6);
    return z;
  };

  auto j_h = [](const Eigen::VectorXd & x) {
    Eigen::MatrixXd h_jacobian(4, 9);
    const double yaw = x(6);
    const double radius = x(8);
    h_jacobian <<
      1, 0, 0, 0, 0, 0, radius * std::sin(yaw), 0, -std::cos(yaw),
      0, 0, 1, 0, 0, 0, -radius * std::cos(yaw), 0, -std::sin(yaw),
      0, 0, 0, 0, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 1, 0, 0;
    return h_jacobian;
  };

  auto u_q = [config = this->config_, dt_provider = this->dt_provider_]() {
    const double dt = dt_provider();
    const double x = config.process_noise_position;
    const double y = config.process_noise_yaw;
    const double r = config.process_noise_radius;
    const double q_x_x = std::pow(dt, 4) / 4 * x;
    const double q_x_vx = std::pow(dt, 3) / 2 * x;
    const double q_vx_vx = std::pow(dt, 2) * x;
    const double q_y_y = std::pow(dt, 4) / 4 * y;
    const double q_y_vy = std::pow(dt, 3) / 2 * y;
    const double q_vy_vy = std::pow(dt, 2) * y;
    const double q_r = std::pow(dt, 4) / 4 * r;
    Eigen::MatrixXd q(9, 9);
    q <<
      q_x_x, q_x_vx, 0, 0, 0, 0, 0, 0, 0,
      q_x_vx, q_vx_vx, 0, 0, 0, 0, 0, 0, 0,
      0, 0, q_x_x, q_x_vx, 0, 0, 0, 0, 0,
      0, 0, q_x_vx, q_vx_vx, 0, 0, 0, 0, 0,
      0, 0, 0, 0, q_x_x, q_x_vx, 0, 0, 0,
      0, 0, 0, 0, q_x_vx, q_vx_vx, 0, 0, 0,
      0, 0, 0, 0, 0, 0, q_y_y, q_y_vy, 0,
      0, 0, 0, 0, 0, 0, q_y_vy, q_vy_vy, 0,
      0, 0, 0, 0, 0, 0, 0, 0, q_r;
    return q;
  };

  auto u_r = [config = this->config_](const Eigen::VectorXd & z) {
    Eigen::DiagonalMatrix<double, 4> r;
    const double dist = std::sqrt(z[0] * z[0] + z[1] * z[1] + z[2] * z[2]);
    const double ps =
      config.tracker.position_noise_base + config.tracker.position_noise_slope * dist;
    const double ys = config.tracker.yaw_noise_base + config.tracker.yaw_noise_slope * dist;
    double xyz_angle_factor = 1.0;
    double yaw_angle_factor = 1.0;

    if (z[0] * z[0] + z[1] * z[1] >= 1e-12) {
      const double bearing_opp = std::atan2(-z[1], -z[0]);
      const double face_angle = std::abs(std::remainder(z[3] - bearing_opp, 2.0 * M_PI));
      const double cos_fa = std::cos(face_angle);
      xyz_angle_factor = 1.0 / std::max(cos_fa * cos_fa, 0.04);
      const double cos_pow =
        std::pow(std::abs(cos_fa), config.tracker.yaw_obliquity_exponent);
      yaw_angle_factor = 1.0 / std::max(cos_pow, 1e-4);
      const double max_oblique_rad = config.tracker.max_yaw_oblique_deg * M_PI / 180.0;
      if (face_angle > max_oblique_rad) {
        yaw_angle_factor = 1e6;
      }
    }

    r.diagonal() <<
      ps * ps * xyz_angle_factor,
      ps * ps * xyz_angle_factor,
      ps * ps * xyz_angle_factor,
      ys * ys * yaw_angle_factor;
    return r;
  };

  Eigen::DiagonalMatrix<double, 9> p0;
  p0.diagonal() << 0.1, 1.0, 0.1, 1.0, 0.1, 0.2, 0.1, 3.0, 0.001;
  tracker->ekf = ExtendedKalmanFilter{f, h, j_f, j_h, u_q, u_r, p0};

  Eigen::VectorXd max_cov(9);
  max_cov << 1.0, 10.0, 1.0, 10.0, 1.0, 2.0, 1.0, 30.0, 0.01;
  tracker->ekf.max_covariance = max_cov;
  tracker->ekf.max_condition_number = 1e6;
  return tracker;
}

}  // namespace rm_auto_aim
