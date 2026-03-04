#include "rm_trajectory/trajectory_solver.hpp"

#include <angles/angles.h>
#include <cmath>
#include <memory>
#include <vector>

#include "rclcpp_components/register_node_macro.hpp"

namespace rm_auto_aim {

// ---------------------------------------------------------------------------
// Constructor — declare ROS 2 parameters and create pub/sub handles.
//
// Parameters (tunable at runtime via ROS 2 parameter server):
//   bullet_speed      [m/s]  — muzzle velocity of the projectile
//   gravity           [m/s²] — gravitational acceleration (positive = down)
//   k                 [1/m]  — linear air-drag coefficient  (a = -k·v)
//   time_bias         [s]    — EMA estimate of pipeline latency (camera→cmd)
//   time_bias_alpha   [–]    — EMA learning rate for the latency estimate
//   gimbal_height     [m]    — vertical offset of gimbal barrel above camera
//   min/max_fire_dist [m]    — range gate: fire only within this distance band
//   angular_window    [rad]  — half-width of the "face aligned" fire gate
//   accel_ema_alpha   [–]    — EMA weight for acceleration estimation
//   max_accel         [m/s²] — clamp on raw acceleration estimates (outlier rejection)
//   latency_gate_sigma[σ]    — reject latency samples further than N·σ from mean
//   indirect_vyaw_threshold  [rad/s] — spin rate above which indirect mode activates
//   indirect_timing_tolerance[s]     — minimum timing residual tolerance in indirect mode
//   indirect_max_candidates  [–]     — search breadth for alignment candidates
// ---------------------------------------------------------------------------
TrajectorySolverNode::TrajectorySolverNode(const rclcpp::NodeOptions &options)
    : Node("trajectory_solver", options) {
  RCLCPP_INFO(this->get_logger(), "Starting TrajectorySolverNode!");

  bullet_speed_    = this->declare_parameter("bullet_speed",    25.0);
  gravity_         = this->declare_parameter("gravity",          9.8);
  k_               = this->declare_parameter("k",                0.01);
  time_bias_       = this->declare_parameter("time_bias",        0.08);
  time_bias_alpha_ = this->declare_parameter("time_bias_alpha",  0.35);
  gimbal_height_   = this->declare_parameter("gimbal_height",    0.5);
  min_fire_dist_   = this->declare_parameter("min_fire_dist",    0.5);
  max_fire_dist_   = this->declare_parameter("max_fire_dist",   10.0);
  angular_window_              = this->declare_parameter("angular_window",               0.09);
  max_measurement_age_         = this->declare_parameter("max_measurement_age",          0.10);
  accel_ema_alpha_ = this->declare_parameter("accel_ema_alpha", 0.3);
  max_accel_       = this->declare_parameter("max_accel",       6.0);
  latency_gate_sigma_ = this->declare_parameter("latency_gate_sigma", 2.5);
  indirect_vyaw_threshold_    = this->declare_parameter("indirect_vyaw_threshold",    3.0);
  indirect_timing_tolerance_  = this->declare_parameter("indirect_timing_tolerance",  0.02);
  indirect_max_candidates_    = this->declare_parameter("indirect_max_candidates",    8);
  oblique_exponent_           = this->declare_parameter("oblique_exponent",            2.0);
  gimbal_pitch_max_ = this->declare_parameter("gimbal_pitch_max",  0.524);  // +30°
  gimbal_pitch_min_ = this->declare_parameter("gimbal_pitch_min", -0.524);  // -30°

  // Match RELIABLE QoS used by armor_tracker's target publisher
  auto target_qos = rclcpp::SensorDataQoS()
      .reliability(rclcpp::ReliabilityPolicy::Reliable)
      .durability(rclcpp::DurabilityPolicy::Volatile)
      .keep_last(30);
  target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
      "/tracker/target", target_qos,
      std::bind(&TrajectorySolverNode::targetCallback, this,
                std::placeholders::_1));

  // Publish gimbal pitch/yaw commands and UART-ready Twist messages
  cmd_pub_ = this->create_publisher<auto_aim_interfaces::msg::GimbalCmd>(
      "/tracker/cmd_gimbal", rclcpp::SensorDataQoS());

  // /cmd_vel carries fire trigger (angular.x) + angles in degrees (y=pitch, z=yaw)
  twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(10).durability(rclcpp::DurabilityPolicy::Volatile));

  // Debug sphere showing predicted bullet impact point in RViz
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/trajectory/marker", rclcpp::QoS(10).durability(rclcpp::DurabilityPolicy::Volatile));

  // Subscribe to current gimbal pose to calculate relative commands
  yaw_sign_      = this->declare_parameter("gimbal_yaw_sign", 1.0);
  pitch_sign_    = this->declare_parameter("gimbal_pitch_sign", 1.0);
  micro_pose_timeout_ = this->declare_parameter("micro_pose_timeout", 0.15);

  micro_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/micro_pose", rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        double p = pitch_sign_ * msg->pose.position.x;
        double y = yaw_sign_  * msg->pose.position.y;
        // Reject garbage serial data — gimbal angles physically can't exceed ~90°
        if (std::abs(p) < 1.6 && std::abs(y) < 1.6) {
          current_pitch_ = p;
          current_yaw_   = y;
        }
        last_micro_pose_time_ = this->now();
      });
}

// ---------------------------------------------------------------------------
// solveTrajectory — iterative ballistic solver with linear air drag.
//
// Given the horizontal ground distance to the target and its height relative
// to the camera, compute the required barrel pitch angle and bullet flight time.
//
// Physics model (in the barrel-plane):
//   Horizontal: x(t) = v·cos(pitch)·(1−e^(−kt))/k   [drag decays speed]
//   Vertical:   z(t) = v·sin(pitch)·t − ½·g·t²       [gravity pulls down]
//
// The solver iterates because pitch depends on flight time, which depends on
// the effective speed, which depends on pitch (via path length).  Each pass:
//   1. Estimate gravity drop for current t → refine pitch via atan2
//   2. Compute path length and mean effective speed under drag → refine t
// Convergence is typically reached in 3–5 iterations.
//
// Returns {pitch [rad], flight_time [s]}.  Returns {0,0} on invalid input or
// non-finite result.
// ---------------------------------------------------------------------------
std::tuple<double, double, bool> TrajectorySolverNode::solveTrajectory(
    const double ground_dist, const double target_z, const double v)
{
  // Input validation — avoid division-by-zero and degenerate geometry
  if (ground_dist < 1e-3 || v < 1e-3) {
    return {0.0, 0.0, false};
  }

  // Seed: assume bullet travels horizontally at full speed (no drag, no gravity)
  double t = ground_dist / v;
  double pitch = 0.0;
  const int max_iter = 10;

  for (int i = 0; i < max_iter; i++) {
    // Step 1: how much does gravity pull the bullet down over t seconds?
    // The barrel must point high enough to compensate for this drop.
    double dz = 0.5 * gravity_ * t * t;
    pitch = std::atan2(target_z + dz, ground_dist);

    // Clamp pitch to physical gimbal limits (avoids cos(pitch)→0 and unreachable angles)
    if (pitch > gimbal_pitch_max_) pitch = gimbal_pitch_max_;
    else if (pitch < gimbal_pitch_min_) pitch = gimbal_pitch_min_;

    // Step 2: actual path length is longer than ground_dist because of pitch
    double cos_pitch = std::cos(pitch);
    double path_len = ground_dist / cos_pitch;

    // Effective bullet speed accounting for linear drag (a = -k*v)
    // Integrating v(t)=v0·e^(-kt) gives mean speed = v0·(1−e^(−kt))/(kt)
    double v_eff;
    double kt = k_ * t;
    if (kt > 1e-6) {
      v_eff = v * (1.0 - std::exp(-kt)) / kt;
    } else {
      v_eff = v;  // drag negligible at very short times
    }

    // Refine flight-time estimate
    double new_t = path_len / v_eff;
    if (std::abs(new_t - t) < 1e-4) {  // converged (< 0.1 ms change)
      t = new_t;
      break;
    }
    t = new_t;
  }

  // Final validity check — catches NaN/Inf from edge cases
  if (!std::isfinite(pitch) || !std::isfinite(t)) {
    RCLCPP_WARN(this->get_logger(), "Trajectory solver produced non-finite result");
    return {0.0, 0.0, false};
  }

  // Check whether the unclamped pitch (what physics requires) is within gimbal
  // limits.  The solver loop clamps pitch for numerical stability, so we
  // recompute the true required angle here.  If it exceeds the limits the
  // target is physically unreachable even though we still return a clamped
  // pitch (so the gimbal tracks as closely as possible).
  double dz_final = 0.5 * gravity_ * t * t;
  double unclamped_pitch = std::atan2(target_z + dz_final, ground_dist);
  bool reachable = (unclamped_pitch >= gimbal_pitch_min_) &&
                   (unclamped_pitch <= gimbal_pitch_max_);

  return {pitch, t, reachable};
}

// ---------------------------------------------------------------------------
// nextAlignmentTime — when does a spinning armor face next point at us?
//
// A 4-armor robot has faces evenly separated by face_spacing = 2π/n_faces.
// Face i currently points at angle:  face_angle = yaw + i·face_spacing
// The camera bearing (direction from robot to camera) is `bearing`.
//
// We want the face to rotate until face_angle == bearing, i.e. the face is
// squarely facing the camera so a bullet aimed at it will not hit an edge.
//
//   delta = shortest angular distance from face_angle to bearing
//   t     = delta / v_yaw   (signed, because v_yaw can be negative)
//
// If t comes out negative the face just passed its alignment window; we add
// one full rotation period to get the next upcoming opportunity.
//
// Returns time [s] until the next alignment (always >= 0).
// ---------------------------------------------------------------------------
double TrajectorySolverNode::nextAlignmentTime(
    double yaw, double v_yaw, double face_idx,
    double face_spacing, double bearing) const
{
  if (std::abs(v_yaw) < 0.1) return 1e9;  // EKF divergence guard — avoid division by zero
  // Current angular position of this face
  double face_angle = yaw + face_idx * face_spacing;
  // How far the face must rotate to reach the bearing
  double delta = angles::shortest_angular_distance(face_angle, bearing);
  // Time for that rotation (may be negative if the face recently passed)
  double t = delta / v_yaw;
  // If negative, the alignment already passed — advance by one full revolution
  double face_period = (2.0 * M_PI) / std::abs(v_yaw);
  if (t < 0.0) t += face_period;
  return t;
}

// ---------------------------------------------------------------------------
// targetCallback — main control loop, called once per EKF output frame.
//
// High-level flow:
//   1. If not tracking → publish zeroed safe command and return.
//   2. Adaptive latency compensation — keep a running EMA of pipeline delay
//      (camera timestamp → now) so all predictions use an accurate dt.
//   3. Acceleration estimation — differentiate EKF velocities and low-pass
//      filter to get a 2nd-order motion model for the target center.
//   4. Mode selection — if the robot spins faster than indirect_vyaw_threshold_
//      switch to INDIRECT mode (pre-aim at a future face alignment moment);
//      otherwise use DIRECT mode (aim at the face currently facing us).
//
// DIRECT mode (slow/stationary spinners):
//   Pass 1 — quick estimate: assume bullet flies to current center position,
//             find which face will be closest to the camera bearing at impact.
//   Pass 2 — refine: recompute armor plate world position at the flight time
//             found in pass 1; re-run trajectory solver.
//   Fire gate — measure angular gap between bullet bearing and nearest face
//               normal at impact time; fire only if gap < effective_window.
//
// INDIRECT mode (fast spinners, |v_yaw| > threshold):
//   For each armor face, compute every upcoming alignment window within 2 s.
//   Solve trajectory to each candidate impact point.
//   Score candidates by |t_align − t_flight| (timing residual).
//   Iteratively refine the best candidate with 2 Newton-like passes.
//   Fire gate — residual must be below a variance-scaled timing tolerance.
// ---------------------------------------------------------------------------
void TrajectorySolverNode::targetCallback(
    auto_aim_interfaces::msg::Target::UniquePtr msg)
{
  if (!msg->tracking) {
    // Publish safe command: fire off, zero angles
    auto_aim_interfaces::msg::GimbalCmd cmd;
    cmd.header = msg->header;
    cmd.pitch    = 0.0;
    cmd.yaw      = 0.0;
    cmd.distance = 0.0;
    cmd.fire_cmd = false;
    cmd_pub_->publish(cmd);

    has_prev_target_ = false;
    ax_ema_ = 0.0; ay_ema_ = 0.0; az_ema_ = 0.0;

    geometry_msgs::msg::Twist twist;
    twist.angular.x = 0.0;  // fire off
    twist_pub_->publish(twist);

    // Delete stale impact marker
    visualization_msgs::msg::Marker del;
    del.header = msg->header;
    del.ns = "impact_point";
    del.action = visualization_msgs::msg::Marker::DELETE;
    marker_pub_->publish(del);
    return;
  }

  // === FIRE SUPPRESSION CONDITIONS ===
  // We compute several conditions that will PREVENT firing even if the aim
  // looks good.  Each addresses a specific failure mode:
  //
  // temp_lost: The EKF has no detection this frame and is coasting on its
  //   motion model.  The predicted position drifts further from reality each
  //   frame.  Firing at a stale prediction wastes ammo and risks hitting
  //   the wrong spot (or a teammate if the robot moved).
  //
  // measurement_stale: Even if the tracker says TRACKING, if the last actual
  //   measurement was > 100ms ago (e.g. YOLO dropped frames but tracker hasn't
  //   timed out yet), the position is unreliable.
  //
  // micro_pose_stale: We publish RELATIVE gimbal angles (target − current).
  //   If the gimbal feedback (/micro_pose) is stale or never received, we
  //   don't know the current gimbal position, so the relative angle is wrong.
  //   Firing with wrong angles sends the bullet in the wrong direction.
  bool temp_lost = (msg->tracker_state == auto_aim_interfaces::msg::Target::TEMP_LOST);
  double measurement_age =
    (rclcpp::Time(msg->header.stamp) - rclcpp::Time(msg->last_measurement_stamp)).seconds();
  bool measurement_stale = (measurement_age > max_measurement_age_);

  double micro_pose_age = (this->now() - last_micro_pose_time_).seconds();
  bool micro_pose_stale = (last_micro_pose_time_.nanoseconds() == 0) ||
                          (micro_pose_age > micro_pose_timeout_);

  // -------------------------------------------------------------------------
  // Adaptive latency compensation (EMA with outlier rejection)
  //
  // The timestamp in msg->header.stamp is set by the camera driver when the
  // frame was captured.  The difference between that and now() is the total
  // pipeline delay (image capture → YOLO → EKF → here).  We track this
  // latency with an Exponential Moving Average so that all kinematic
  // predictions are shifted forward by the correct amount of time.
  //
  // Outlier rejection: GPU thermal throttle or USB stalls cause occasional
  // latency spikes.  We gate updates to within latency_gate_sigma_ standard
  // deviations of the running mean to prevent those from corrupting time_bias_.
  // -------------------------------------------------------------------------
  double measured_latency = (this->now() - msg->header.stamp).seconds();
  if (measured_latency <= 0.0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Measured latency %.4fs is near-zero — skipping EMA update (clock sync issue?)",
      measured_latency);
  }
  if (measured_latency >= 0.001 && measured_latency < 0.5) {
    double residual = measured_latency - time_bias_;
    double sigma = std::sqrt(std::max(time_bias_var_, 1e-8));
    // Accept first N samples unconditionally to bootstrap the EMA,
    // then apply outlier rejection (GPU thermal throttle, USB stall)
    bool in_warmup = latency_warmup_count_ < kLatencyWarmupSamples;
    if (in_warmup || std::abs(residual) < latency_gate_sigma_ * sigma) {
      if (in_warmup) {
        ++latency_warmup_count_;
      }
      time_bias_ = time_bias_alpha_ * measured_latency
                   + (1.0 - time_bias_alpha_) * time_bias_;
      // Update variance only for accepted samples to prevent outlier spikes
      // from inflating sigma and widening the gate permanently
      double new_residual = measured_latency - time_bias_;
      time_bias_var_ = time_bias_alpha_ * (new_residual * new_residual)
                       + (1.0 - time_bias_alpha_) * time_bias_var_;
    }
  }

  // EKF state: robot *center* position and velocity (not the visible armor plate)
  double xc = msg->position.x, yc = msg->position.y, za = msg->position.z;
  double vx = msg->velocity.x, vy = msg->velocity.y, vz = msg->velocity.z;

  // -------------------------------------------------------------------------
  // Acceleration estimation — EMA of finite-difference velocity derivatives
  //
  // The EKF state only carries velocity, not acceleration.  We estimate
  // acceleration by differencing consecutive velocity readings and then
  // smoothing with an EMA to suppress noise.
  //
  // Time-adjusted alpha: α_dt = 1 − (1−α)^(dt / dt_nominal)
  //   This keeps the filter bandwidth consistent regardless of whether frames
  //   arrive at 30 Hz or 60 Hz (normalised to a 30 Hz reference).
  //
  // Raw acceleration estimates are clamped to max_accel_ before filtering to
  // reject sensor glitches or large EKF jumps.
  // -------------------------------------------------------------------------
  double ax = 0.0, ay = 0.0, az = 0.0;
  if (has_prev_target_) {
    double dt_tgt = (rclcpp::Time(msg->header.stamp) - prev_target_time_).seconds();
    if (dt_tgt > 0.005 && dt_tgt < 0.2) {
      auto clamp = [this](double val) {
        return std::max(-max_accel_, std::min(val, max_accel_));
      };
      // Time-adjusted alpha: consistent bandwidth regardless of frame rate
      double alpha_dt = 1.0 - std::pow(1.0 - accel_ema_alpha_, dt_tgt / (1.0 / 30.0));
      ax_ema_ = alpha_dt * clamp((vx - prev_vx_) / dt_tgt)
                + (1.0 - alpha_dt) * ax_ema_;
      ay_ema_ = alpha_dt * clamp((vy - prev_vy_) / dt_tgt)
                + (1.0 - alpha_dt) * ay_ema_;
      az_ema_ = alpha_dt * clamp((vz - prev_vz_) / dt_tgt)
                + (1.0 - alpha_dt) * az_ema_;
    }
  }
  ax = ax_ema_; ay = ay_ema_; az = az_ema_;
  has_prev_target_ = true;
  prev_target_time_ = msg->header.stamp;
  prev_vx_ = vx; prev_vy_ = vy; prev_vz_ = vz;

  // Validate armor geometry fields before any trajectory math
  int n_faces = std::max(msg->armors_num, 1);
  if (msg->radius_1 <= 0.0 || (n_faces == 4 && msg->radius_2 <= 0.0)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Invalid armor radii (r1=%.3f r2=%.3f) — skipping frame", msg->radius_1, msg->radius_2);
    auto_aim_interfaces::msg::GimbalCmd safe;
    safe.header = msg->header;
    safe.fire_cmd = false;
    cmd_pub_->publish(safe);
    return;
  }

  // Evenly-spaced face angles around the robot (2π / number of armor plates)
  double face_spacing = 2.0 * M_PI / n_faces;

  // -------------------------------------------------------------------------
  // Indirect mode selection (hysteresis)
  //
  // When a robot spins very fast (|v_yaw| > indirect_vyaw_threshold_) it is
  // impractical to track a single face directly because the face disappears
  // before the bullet arrives.  Instead we pre-aim at a *future* alignment
  // moment when a face will rotate into our line of fire.
  //
  // Hysteresis prevents rapid toggling near the threshold:
  //   activate when |v_yaw| > threshold
  //   deactivate when |v_yaw| < 0.7 × threshold
  // -------------------------------------------------------------------------
  double abs_vyaw = std::abs(msg->v_yaw);
  if (indirect_mode_active_) {
    indirect_mode_active_ = (abs_vyaw > indirect_vyaw_threshold_ * 0.7) && (n_faces > 1);
  } else {
    indirect_mode_active_ = (abs_vyaw > indirect_vyaw_threshold_) && (n_faces > 1);
  }

  if (indirect_mode_active_) {
    // -----------------------------------------------------------------------
    // INDIRECT AIMING — for fast-spinning robots (|v_yaw| > ~3 rad/s)
    //
    // WHY A SEPARATE MODE?
    //   When a robot spins fast, each face is only visible for ~50ms before
    //   the next face rotates in.  The bullet takes ~100-200ms to fly there.
    //   So by the time the bullet arrives, the face we aimed at is gone and
    //   a different face (or the gap between faces) is in its place.
    //
    //   Direct aiming fails because it aims at WHERE the face IS, not where
    //   it WILL BE.  Even with flight-time compensation, the face rotates
    //   too fast for the correction to keep up.
    //
    // STRATEGY:
    //   Don't chase the current face.  Instead, ask: "when will ANY face next
    //   be pointing at my barrel?"  For each face, compute its next alignment
    //   window.  Solve the ballistic trajectory to that future position.  The
    //   best candidate is the one where the bullet flight time most closely
    //   matches the alignment time (smallest timing residual).
    //
    //   Fire only when the residual is small enough that the bullet arrives
    //   within the angular window of the aligned face.
    // -----------------------------------------------------------------------

    // Coarse flight-time seed: distance to robot center / muzzle velocity
    double dist_center = std::sqrt(xc * xc + yc * yc + za * za);
    if (bullet_speed_ < 1e-3) {
      RCLCPP_WARN(this->get_logger(), "bullet_speed is near zero, skipping");
      visualization_msgs::msg::Marker del;
      del.header = msg->header;
      del.ns = "impact_point";
      del.action = visualization_msgs::msg::Marker::DELETE;
      marker_pub_->publish(del);
      return;
    }
    double t0 = dist_center / bullet_speed_;

    // Approximate bearing to the predicted center position at impact time
    // (will be refined per-candidate below)
    double bearing0 = std::atan2(yc + vy * (t0 + time_bias_),
                                 xc + vx * (t0 + time_bias_));

    // Each candidate represents one future alignment window: which face (i),
    // when it aligns (t_align), and the resulting trajectory solution.
    struct Candidate {
      int face;
      double t_align;   // time until this face faces the camera [s]
      double residual;  // |t_align − (t_flight + latency)| — timing error [s]
      double pitch;     // required gimbal pitch [rad]
      double yaw;       // required gimbal yaw [rad]
      double range;     // 3-D distance to predicted impact point [m]
      double tx, ty, tz; // predicted impact position in camera frame [m]
      bool reachable;   // true if unclamped pitch is within gimbal limits
    };
    std::vector<Candidate> candidates;
    candidates.reserve(indirect_max_candidates_);

    // One full rotation of the robot in seconds
    double face_period = (2.0 * M_PI) / abs_vyaw;

    // Build the candidate list: for each face, consider its next 2 alignment
    // windows (current and one revolution later) to give the solver choices
    // in case the nearest window is too soon for the bullet to arrive.
    for (int i = 0; i < n_faces && static_cast<int>(candidates.size()) < indirect_max_candidates_; i++) {
      double ta = nextAlignmentTime(msg->yaw, msg->v_yaw, static_cast<double>(i),
                                     face_spacing, bearing0);
      for (int occ = 0; occ < 2 && static_cast<int>(candidates.size()) < indirect_max_candidates_; occ++) {
        double t_a = ta + occ * face_period;
        if (t_a < 0.001 || t_a > 2.0) continue;  // Skip implausible times

        // For 4-armor robots, faces alternate between two radii (r1, r2) and
        // two heights (za, za+dz) — even-indexed faces use radius_1/height za.
        bool is_current_pair = (i % 2 == 0);
        double r = (n_faces == 4 && !is_current_pair) ? msg->radius_2 : msg->radius_1;
        double dz_offset = (n_faces == 4 && !is_current_pair) ? msg->dz : 0.0;

        // Kinematic prediction of robot center at alignment time t_a
        // (constant acceleration model: p = p0 + v·t + ½·a·t²)
        double pcx = xc + vx * t_a + 0.5 * ax * t_a * t_a;
        double pcy = yc + vy * t_a + 0.5 * ay * t_a * t_a;
        double pcz = za + vz * t_a + 0.5 * az * t_a * t_a;

        // Face position = center − radius * [cos(face_yaw), sin(face_yaw), 0]
        // (armor plate is offset from center by radius in the horizontal plane)
        double face_yaw_at = msg->yaw + msg->v_yaw * t_a + i * face_spacing;
        double ctx = pcx - r * std::cos(face_yaw_at);
        double cty = pcy - r * std::sin(face_yaw_at);
        double ctz = pcz + dz_offset;

        double gdist = std::sqrt(ctx * ctx + cty * cty);
        double cyaw = std::atan2(cty, ctx);
        auto result = solveTrajectory(gdist, ctz - gimbal_height_, bullet_speed_);
        double cpitch = std::get<0>(result);
        double ct_flight = std::get<1>(result);
        bool creachable = std::get<2>(result);
        if (ct_flight < 1e-6) continue;  // Solver returned degenerate result

        double crange = std::sqrt(ctx * ctx + cty * cty + ctz * ctz);
        // Timing residual: how closely does the bullet's travel time match
        // the moment the face is aligned? Smaller = better shot opportunity.
        double res = std::abs(t_a - (ct_flight + time_bias_));

        candidates.push_back({i, t_a, res, cpitch, cyaw, crange, ctx, cty, ctz, creachable});
      }
    }

    if (candidates.empty()) {
      // No valid candidate — publish safe command
      auto_aim_interfaces::msg::GimbalCmd cmd;
      cmd.header = msg->header;
      cmd.pitch    = 0.0;
      cmd.yaw      = 0.0;
      cmd.distance = 0.0;
      cmd.fire_cmd = false;
      cmd_pub_->publish(cmd);

      geometry_msgs::msg::Twist twist;
      twist.angular.x = 0.0;
      twist_pub_->publish(twist);

      visualization_msgs::msg::Marker del;
      del.header = msg->header;
      del.ns = "impact_point";
      del.action = visualization_msgs::msg::Marker::DELETE;
      marker_pub_->publish(del);
      return;
    }

    // Select the candidate with the smallest timing residual, preferring
    // reachable candidates.  Fall back to unreachable only for tracking aim
    // (fire will be suppressed anyway if !reachable).
    Candidate *best = &candidates[0];
    Candidate *best_reachable = nullptr;
    for (auto &c : candidates) {
      if (c.reachable && (!best_reachable || c.residual < best_reachable->residual))
        best_reachable = &c;
      if (c.residual < best->residual)
        best = &c;
    }
    if (best_reachable) best = best_reachable;

    // -----------------------------------------------------------------------
    // Newton-like refinement (2 passes)
    //
    // The initial candidate used bearing0 (estimated from the center).  Now
    // that we have a better range estimate for this specific face, recompute:
    //   1. The bearing at the refined flight time.
    //   2. The next alignment time of the chosen face near that flight time.
    //   3. The full trajectory to the refined impact point.
    // Two passes are enough because convergence is fast once the face is chosen.
    // -----------------------------------------------------------------------
    for (int iter = 0; iter < 2; iter++) {
      // Expected total time from now until bullet hits: flight time + latency
      double t_target = best->range / bullet_speed_ + time_bias_;
      // Recompute bearing at the refined impact time (includes acceleration)
      double bearing_new = std::atan2(yc + vy * t_target + 0.5 * ay * t_target * t_target,
                                      xc + vx * t_target + 0.5 * ax * t_target * t_target);
      double ta_new = nextAlignmentTime(msg->yaw, msg->v_yaw,
                                         static_cast<double>(best->face),
                                         face_spacing, bearing_new);
      // Among {ta_new, ta_new±period} pick the occurrence closest to t_target
      double t_candidates[3] = {ta_new, ta_new + face_period, ta_new - face_period};
      double best_ta = ta_new;
      double best_diff = std::abs(ta_new - t_target);
      for (int j = 1; j < 3; j++) {
        if (t_candidates[j] > 0.001) {
          double diff = std::abs(t_candidates[j] - t_target);
          if (diff < best_diff) {
            best_diff = diff;
            best_ta = t_candidates[j];
          }
        }
      }
      best->t_align = best_ta;

      // Recompute impact position and trajectory at the refined alignment time
      bool is_current_pair = (best->face % 2 == 0);
      double r = (n_faces == 4 && !is_current_pair) ? msg->radius_2 : msg->radius_1;
      double dz_offset = (n_faces == 4 && !is_current_pair) ? msg->dz : 0.0;

      double pcx = xc + vx * best_ta + 0.5 * ax * best_ta * best_ta;
      double pcy = yc + vy * best_ta + 0.5 * ay * best_ta * best_ta;
      double pcz = za + vz * best_ta + 0.5 * az * best_ta * best_ta;
      double face_yaw_at = msg->yaw + msg->v_yaw * best_ta + best->face * face_spacing;

      best->tx = pcx - r * std::cos(face_yaw_at);
      best->ty = pcy - r * std::sin(face_yaw_at);
      best->tz = pcz + dz_offset;

      double gdist = std::sqrt(best->tx * best->tx + best->ty * best->ty);
      best->yaw = std::atan2(best->ty, best->tx);
      auto result = solveTrajectory(gdist, best->tz - gimbal_height_, bullet_speed_);
      best->pitch = std::get<0>(result);
      double t_fl = std::get<1>(result);
      best->reachable = std::get<2>(result);
      if (t_fl < 1e-6) break;
      best->range = std::sqrt(best->tx * best->tx + best->ty * best->ty + best->tz * best->tz);
      best->residual = std::abs(best->t_align - (t_fl + time_bias_));
    }

    // -----------------------------------------------------------------------
    // Indirect fire gate
    //
    // Fire only when:
    //   • Target is within the safe range band.
    //   • EKF is not coasting (temp_lost).
    //   • Timing residual < tolerance (bullet will arrive as the face aligns).
    //
    // Timing tolerance is scaled by yaw-rate uncertainty so that a poorly
    // estimated spin rate results in a wider (more conservative) tolerance,
    // but never smaller than indirect_timing_tolerance_ (hardware minimum).
    // -----------------------------------------------------------------------
    bool dist_ok = (best->range >= min_fire_dist_) && (best->range <= max_fire_dist_);
    bool fire = false;
    if (dist_ok && !temp_lost && !measurement_stale && !micro_pose_stale && best->reachable) {
      double sigma_vyaw = std::sqrt(std::max(msg->v_yaw_variance, 1e-6));
      // Angular uncertainty at impact time → convert to time via spin rate
      double timing_tol = (abs_vyaw > 0.1)
          ? (angular_window_ + sigma_vyaw * best->t_align) / abs_vyaw
          : indirect_timing_tolerance_;
      timing_tol = std::max(timing_tol, indirect_timing_tolerance_);
      fire = (best->residual < timing_tol);
    }

    // Publish GimbalCmd (relative angles!)
    auto_aim_interfaces::msg::GimbalCmd cmd;
    cmd.header = msg->header;
    double rel_pitch = best->pitch - current_pitch_;
    double rel_yaw = angles::shortest_angular_distance(current_yaw_, best->yaw);
    
    cmd.pitch    = rel_pitch * 180.0 / M_PI;
    cmd.yaw      = rel_yaw * 180.0 / M_PI;
    cmd.distance = best->range;
    cmd.fire_cmd = fire;
    cmd_pub_->publish(cmd);

    // Publish Twist
    geometry_msgs::msg::Twist twist;
    twist.angular.x = fire ? 1.0 : 0.0;
    twist.angular.y = rel_pitch * 180.0 / M_PI;
    twist.angular.z = rel_yaw   * 180.0 / M_PI;
    twist_pub_->publish(twist);

    // Orange marker for indirect impact point
    visualization_msgs::msg::Marker marker;
    marker.header = msg->header;
    marker.ns = "impact_point";
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = best->tx;
    marker.pose.position.y = best->ty;
    marker.pose.position.z = best->tz;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.1;
    marker.color.a = 1.0; marker.color.r = 1.0; marker.color.g = 0.5;
    marker_pub_->publish(marker);

    return;
  }

  // =========================================================================
  // DIRECT AIMING — for slow/stationary spinners (normal case)
  //
  // We aim at the armor face that will be most directly facing the camera
  // at the moment the bullet arrives.  Two-pass approach removes the
  // chicken-and-egg dependency between "which face?" and "how long to fly?".
  // =========================================================================

  // Pass 1 seed: approximate flight time using straight-line distance to center
  double dist_center = std::sqrt(xc * xc + yc * yc + za * za);
  if (bullet_speed_ < 1e-3) {
    RCLCPP_WARN(this->get_logger(), "bullet_speed is near zero, skipping");
    auto_aim_interfaces::msg::GimbalCmd cmd;
    cmd.header = msg->header;
    cmd.fire_cmd = false;
    cmd_pub_->publish(cmd);

    geometry_msgs::msg::Twist twist;
    twist.angular.x = 0.0;
    twist_pub_->publish(twist);

    visualization_msgs::msg::Marker del;
    del.header = msg->header;
    del.ns = "impact_point";
    del.action = visualization_msgs::msg::Marker::DELETE;
    marker_pub_->publish(del);
    return;
  }
  double t0 = dist_center / bullet_speed_;
  // Total prediction horizon = flight time + pipeline latency
  double pt = t0 + time_bias_;

  // findBestFace: given a prediction time, return the index of the armor face
  // that is the best shooting target, considering both distance and oblique angle.
  //
  // Score = distance / cos²(oblique_angle)
  //   - A close, square-on face scores low (good).
  //   - A far, oblique face scores high (bad).
  //   - cos(oblique) < 0.26 means the face is >75° away — not a good target.
  //
  // The cos² penalty ensures face-on armor is preferred even at close range,
  // where the radius offset makes the oblique face significantly closer.
  // With single cos, the distance advantage can dominate at ranges < 1m.
  //
  // If all faces are too oblique, all_faces_oblique is set true and the
  // least-bad face is returned (for aiming), but the caller suppresses fire.
  bool all_faces_oblique = false;
  auto findBestFace = [&](double predict_time) -> int {
    double pred_yaw = msg->yaw + msg->v_yaw * predict_time;
    double pred_cx = xc + vx * predict_time + 0.5 * ax * predict_time * predict_time;
    double pred_cy = yc + vy * predict_time + 0.5 * ay * predict_time * predict_time;
    double pred_cz = za + vz * predict_time + 0.5 * az * predict_time * predict_time;

    double best_score = 1e9;
    int best_f = -1;
    // Fallback: track the least-bad face (lowest raw distance) when all are oblique
    double fallback_dist = 1e9;
    int fallback_f = 0;
    for (int i = 0; i < n_faces; i++) {
      // Each face is at yaw + i × (2π/n) from the robot center
      double face_yaw = pred_yaw + i * face_spacing;

      // Face position = center − r · [cos(face_yaw), sin(face_yaw)]
      // For 4-armor robots: even faces (0,2) use r1/za, odd faces (1,3) use r2/za+dz
      bool is_current_pair = (i % 2 == 0);
      double r = (n_faces == 4 && !is_current_pair) ? msg->radius_2 : msg->radius_1;
      double dz_off = (n_faces == 4 && !is_current_pair) ? msg->dz : 0.0;

      double fx = pred_cx - r * std::cos(face_yaw);
      double fy = pred_cy - r * std::sin(face_yaw);
      double fz = pred_cz + dz_off;

      // Distance from camera (at origin in odom frame) to this face
      double dist = std::sqrt(fx * fx + fy * fy + fz * fz);

      // Oblique angle: how much the face is turned away from the camera.
      //
      //   face_normal: the direction the armor plate faces (outward from center).
      //                face_yaw points from center → plate, so outward = face_yaw + π.
      //   face_to_cam: bearing from the plate back to the camera (at origin).
      //                = atan2(-fy, -fx) because camera is at (0,0).
      //   oblique = angle between these two directions.
      //
      //   oblique = 0°  → face is perfectly facing the camera (ideal shot)
      //   oblique = 90° → face is edge-on to the camera (bullet hits edge, not plate)
      double face_normal = face_yaw + M_PI;  // outward from center
      double face_to_cam = std::atan2(-fy, -fx);
      double oblique = std::abs(angles::shortest_angular_distance(face_normal, face_to_cam));

      // Effective distance: penalises oblique faces (less visible area).
      // cos^n penalty ensures face-on armor wins even at close range where
      // the radius offset makes the oblique face significantly closer.
      double cos_obl = std::cos(oblique);
      RCLCPP_DEBUG(this->get_logger(),
          "  Face[%d] pos=(%.3f,%.3f,%.3f) dist=%.3f oblique=%.1f° cos=%.3f %s",
          i, fx, fy, fz, dist, oblique * 180.0 / M_PI, cos_obl,
          (cos_obl < 0.26) ? "OBLIQUE" : "");

      // Track closest face as fallback regardless of oblique threshold
      if (dist < fallback_dist) { fallback_dist = dist; fallback_f = i; }

      if (cos_obl < 0.26) continue;  // face pointing away (>75°)
      double score = dist / std::pow(cos_obl, oblique_exponent_);

      RCLCPP_DEBUG(this->get_logger(), "    -> score=%.3f (best=%.3f)", score, best_score);
      if (score < best_score) { best_score = score; best_f = i; }
    }
    if (best_f < 0) {
      // All faces too oblique — return closest for aiming, flag for fire suppression
      all_faces_oblique = true;
      return fallback_f;
    }
    all_faces_oblique = false;
    return best_f;
  };

  // Pick the face that gives the best shot: close + face-on at predicted impact time
  int best_face = findBestFace(pt);
  double pitch = 0.0, t_flight = 0.0;
  bool reachable = false;
  double tx = 0.0, ty = 0.0, tz = 0.0;
  double ground_dist = 0.0, yaw = 0.0;

  // === TWO-PASS TRAJECTORY REFINEMENT ===
  // WHY TWO PASSES?  There's a chicken-and-egg problem:
  //   - To pick the best face, we need to know the flight time (so we can predict
  //     where each face will be when the bullet arrives).
  //   - To compute the flight time, we need to know WHICH face we're aiming at
  //     (because its 3D position determines the trajectory).
  //
  // Pass 0: Use a rough flight-time estimate (straight-line distance / speed)
  //         to pick a face, then solve the real trajectory to that face.
  // After pass 0: Check — with the real flight time, is the same face still the
  //         best?  If yes, we're done (break).  If not, re-solve with the new face.
  // Pass 1: Only runs if the face changed.  Re-solve trajectory to the new face.
  //
  // In practice, the face rarely changes between passes (maybe 5% of frames at
  // close range where the radius offset matters more).
  for (int pass = 0; pass < 2; pass++) {
    // Use the refined flight time on pass 1
    double total_t = (pass == 0) ? pt : (t_flight + time_bias_);
    double yaw_at = msg->yaw + msg->v_yaw * total_t;
    double face_yaw = yaw_at + best_face * face_spacing;

    // Even-indexed faces: radius_1 / height za
    // Odd-indexed faces:  radius_2 / height za+dz  (for 4-armor robots)
    bool is_current_pair = (best_face % 2 == 0);
    double r = (n_faces == 4 && !is_current_pair) ? msg->radius_2 : msg->radius_1;
    double dz_offset = (n_faces == 4 && !is_current_pair) ? msg->dz : 0.0;

    // Predict robot center at total_t then offset by armor radius to get plate position
    double pcx = xc + vx * total_t + 0.5 * ax * total_t * total_t;
    double pcy = yc + vy * total_t + 0.5 * ay * total_t * total_t;
    double pcz = za + vz * total_t + 0.5 * az * total_t * total_t;

    tx = pcx - r * std::cos(face_yaw);
    ty = pcy - r * std::sin(face_yaw);
    tz = pcz + dz_offset;

    ground_dist = std::sqrt(tx * tx + ty * ty);
    yaw = std::atan2(ty, tx);

    auto result = solveTrajectory(ground_dist, tz - gimbal_height_, bullet_speed_);
    pitch = std::get<0>(result);
    t_flight = std::get<1>(result);
    reachable = std::get<2>(result);

    if (pass == 0) {
      // Recheck: with refined flight time, is a different face better?
      int new_face = findBestFace(t_flight + time_bias_);
      if (new_face == best_face) break;  // converged — same face
      best_face = new_face;  // face changed — re-solve in pass 1
    }
  }

  // Guard: solveTrajectory returns (0,0) when ground_dist ≈ 0 (target at camera origin)
  if (t_flight < 1e-6) {
    auto_aim_interfaces::msg::GimbalCmd cmd;
    cmd.header = msg->header;
    cmd.pitch    = 0.0;
    cmd.yaw      = 0.0;
    cmd.distance = 0.0;
    cmd.fire_cmd = false;
    cmd_pub_->publish(cmd);

    geometry_msgs::msg::Twist twist;
    twist.angular.x = 0.0;
    twist_pub_->publish(twist);

    visualization_msgs::msg::Marker del;
    del.header = msg->header;
    del.ns = "impact_point";
    del.action = visualization_msgs::msg::Marker::DELETE;
    marker_pub_->publish(del);
    return;
  }

  double range = std::sqrt(tx * tx + ty * ty + tz * tz);
  bool dist_ok = (range >= min_fire_dist_) && (range <= max_fire_dist_);

  // -----------------------------------------------------------------------
  // Direct fire gate
  //
  // At the predicted impact time, compute the angular gap between our barrel
  // bearing (yaw) and the nearest face normal.  Fire only if that gap is
  // smaller than the effective angular window.
  //
  // The window is widened by yaw-rate uncertainty (σ_vyaw × t_flight) to
  // account for EKF spin-rate estimation error, but capped at half the face
  // spacing to avoid accidently allowing shots at non-facing plates.
  // -----------------------------------------------------------------------
  bool in_range = false;
  if (dist_ok && !temp_lost && !measurement_stale && !micro_pose_stale && reachable &&
      !all_faces_oblique) {
    // Predict robot yaw at the moment the bullet hits
    double final_yaw = msg->yaw + msg->v_yaw * (t_flight + time_bias_);

    // Check only the face we aimed at (best_face), not the nearest face.
    // Using the nearest face would allow firing when the barrel is aligned
    // with a neighbour face while best_face is still off-angle.
    double aimed_fy = final_yaw + best_face * face_spacing;
    double face_diff = std::abs(angles::shortest_angular_distance(yaw, aimed_fy));

    // Variance-based window, capped to half face spacing
    double sigma_vyaw = std::sqrt(std::max(msg->v_yaw_variance, 1e-6));
    double yaw_uncertainty = sigma_vyaw * (t_flight + time_bias_);
    double max_window = face_spacing * 0.5;  // Can't exceed half the face spacing
    double effective_window = std::min(angular_window_ + yaw_uncertainty, max_window);

    in_range = (face_diff < effective_window);
  }

  // -----------------------------------------------------------------------
  // Publish GimbalCmd — RELATIVE angles (degrees) for the lower computer.
  //
  // The lower computer's PID controller expects:
  //   pitch = (target_pitch − current_pitch) in degrees
  //   yaw   = shortest_angular_distance(current_yaw, target_yaw) in degrees
  //
  // current_pitch_ and current_yaw_ come from /micro_pose (gimbal feedback).
  // If /micro_pose is stale, the relative angles will be wrong and fire is
  // already suppressed by the micro_pose_stale check above.
  // -----------------------------------------------------------------------
  auto_aim_interfaces::msg::GimbalCmd cmd;
  cmd.header = msg->header;
  double rel_pitch = pitch - current_pitch_;
  double rel_yaw = angles::shortest_angular_distance(current_yaw_, yaw);

  cmd.pitch    = rel_pitch * 180.0 / M_PI;   // degrees
  cmd.yaw      = rel_yaw * 180.0 / M_PI;     // degrees
  cmd.distance = range;                       // 3D distance to target (m)
  cmd.fire_cmd = in_range;                    // true = "pull trigger now"
  cmd_pub_->publish(cmd);

  RCLCPP_DEBUG(this->get_logger(),
      "DIRECT body_yaw=%.3f face=%d pos=(%.3f,%.3f,%.3f) abs_yaw=%.1f° abs_pitch=%.1f° rel_yaw=%.1f° rel_pitch=%.1f° fire=%d",
      msg->yaw, best_face, tx, ty, tz,
      yaw * 180.0 / M_PI, pitch * 180.0 / M_PI,
      rel_yaw * 180.0 / M_PI, rel_pitch * 180.0 / M_PI, in_range);

  // Twist message for /cmd_vel (UART bridge to lower computer):
  //   angular.x = fire trigger (1.0 = fire, 0.0 = hold)
  //   angular.y = relative pitch in degrees
  //   angular.z = relative yaw in degrees
  // The cmd_vel_subscriber Python node reads these and packs them into
  // the UART protocol expected by the STM32 lower computer.
  geometry_msgs::msg::Twist twist;
  twist.angular.x = in_range ? 1.0 : 0.0;
  twist.angular.y = rel_pitch * 180.0 / M_PI;
  twist.angular.z = rel_yaw   * 180.0 / M_PI;
  twist_pub_->publish(twist);

  // Visualise predicted impact point (green sphere in RViz)
  visualization_msgs::msg::Marker marker;
  marker.header = msg->header;
  marker.ns = "impact_point";
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = tx;
  marker.pose.position.y = ty;
  marker.pose.position.z = tz;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = marker.scale.y = marker.scale.z = 0.1;
  marker.color.a = 1.0; marker.color.g = 1.0;
  marker_pub_->publish(marker);
}

} // namespace rm_auto_aim

RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::TrajectorySolverNode);
