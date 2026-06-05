#ifndef AUTOAIM__TRACKER_HPP_
#define AUTOAIM__TRACKER_HPP_

#include <Eigen/Dense>
#include <cmath>
#include <string>
#include <vector>
#include <rclcpp/time.hpp>

namespace autoaim
{

// ── Armor detection in odom frame (after TF transform) ──
struct ArmorDetection
{
  double x, y, z;       // position in odom [m]
  double yaw;           // face normal direction [rad]
  std::string class_id; // YOLO class ("0"=blue, "1"=grey, "2"=red)
  double confidence;
  // Range from CAMERA/ROBOT to armor [m]. Set during PnP from the camera-frame
  // translation vector. Use this for distance filtering and target selection.
  // DO NOT use sqrt(x*x+y*y+z*z) — those are world coords, and once ego-motion
  // is active the robot is no longer at the world origin.
  double rel_range = 0.0;
  /// Backwards-compatible accessor — returns the camera-relative range.
  double range() const { return rel_range; }
};

// ── Gimbal command output ──
struct AimResult
{
  // Absolute target angles in the IMU/odom startup reference frame [rad].
  // These are final destinations for the microcontroller, not increments.
  double abs_yaw   = 0;
  double abs_pitch = 0;

  // Current angular error from the measured head/gimbal orientation to the
  // desired target [rad]. Kept for fire gating/debug only.
  double rel_yaw   = 0;
  double rel_pitch = 0;

  double distance  = 0;  // range to selected armor face [m]

  // Selected armor face center in odom frame. This is used only for debug
  // projection/markers, so the overlay stays on the plate center while the
  // command angles can still include barrel offset and gravity compensation.
  bool   target_valid = false;
  double target_x = 0;
  double target_y = 0;
  double target_z = 0;

  bool   tracking  = false;
  bool   fire      = false;
};

// ── All config in one struct ──
struct TrackerConfig
{
  // Detection filters
  double light_ratio       = 0.85;
  double max_armor_dist    = 8.0;
  double max_armor_z       = 1.2;

  // Tracker state machine
  int    confirm_frames    = 3;     // frames to promote DETECTING → TRACKING
  double lost_timeout      = 0.4;   // seconds before TEMP_LOST → LOST

  // EKF process noise (higher = trust measurements more, respond faster)
  double q_pos             = 5.0;
  double q_yaw             = 10.0;
  double q_r               = 1e-6;

  // EKF measurement noise (higher = trust measurements less, smoother)
  double r_pos_base        = 0.05;
  double r_pos_slope       = 0.04;
  double r_yaw_base        = 0.05;
  double r_yaw_slope       = 0.005;
  double max_oblique_deg   = 65.0;

  // Velocity damping: v *= alpha each frame (at ref_freq Hz)
  double alpha_pos         = 0.99;
  double alpha_yaw         = 1.00;
  double alpha_coast       = 0.95;  // stronger damping when no measurement

  // Armor geometry
  double initial_radius    = 0.24;
  double radius_ema_alpha  = 0.05;
  // Known height step between armor pairs [m]. Faces 0,2 are at center z;
  // faces 1,3 are at center z + initial_dz. Set to the measured physical
  // offset (e.g. 0.05 for 5cm step). The EKF refines it after observing a
  // face jump, but seeding avoids shooting low on the raised pair before
  // a jump is seen. Set to 0.0 if all four faces are at the same height.
  double initial_dz        = 0.0;

  // ── BALLISTICS & PHYSICAL GEOMETRY ──
  double bullet_speed      = 25.0;  // [m/s] — MEASURE THIS on your robot!
  double gravity           = 9.8;
  double gimbal_height     = 0.325; // camera/gimbal pivot above ground [m]

  // Camera-to-barrel offset in the GIMBAL body frame [m].
  // Measured from the camera optical center to the barrel exit point.
  //
  //   barrel_offset_x: forward (positive = barrel is in front of camera)
  //   barrel_offset_y: left    (positive = barrel is left of camera)
  //   barrel_offset_z: up      (positive = barrel is above camera)
  //
  // WHY THIS MATTERS:
  //   The camera sees the target from one position but the bullet leaves
  //   from a different position. At close range (< 2m), a 10cm offset
  //   causes ~3° aiming error — bigger than the armor plate.
  //   At 5m it's ~1° (borderline). Without this correction, close-range
  //   shots systematically miss in one direction.
  //
  // HOW TO MEASURE:
  //   Use a ruler from the center of the ZED camera lens to the barrel
  //   center. Positive X = barrel is further forward, positive Z = barrel
  //   is higher than camera, positive Y = barrel is to the left.
  double barrel_offset_x   = 0.0;   // typically ~0.0 to 0.05
  double barrel_offset_y   = 0.0;   // typically ~0.0 (centered)
  double barrel_offset_z   = -0.10; // typically -0.05 to -0.15 (barrel below camera)

  // Fire gate
  double angular_window    = 0.30;
  double window_ref_dist   = 3.0;
  double min_fire_dist     = 0.3;
  double max_fire_dist     = 6.0;

  // Timing
  double time_bias         = 0.10;
  double ref_freq          = 100.0;

  // Match gates
  double max_match_dist    = 0.5;
  // yaw_jump_thresh [rad]: minimum measured yaw change to trigger handleArmorJump.
  // Below this = normal EKF update. Above this = face switch.
  // Default M_PI/3 (60°) can miss oblique face jumps. 0.5 rad (28°) detects
  // jumps earlier for faster vyaw convergence during spinning.
  double yaw_jump_thresh   = 0.50;
  double maha_threshold    = 13.3;

  // ── SPIN RATE ESTIMATION FROM FACE JUMP TIMING ──
  // vyaw estimated directly from dt between consecutive 90° face jumps.
  // This is far more accurate and converges faster than waiting for the
  // EKF to infer vyaw from noisy position measurements.
  // After 2 same-direction jumps, vyaw = (pi/2) / dt_between_jumps.
  bool   use_vyaw_from_timing  = true;
  double vyaw_timing_min_dt    = 0.020;  // ignore if jump faster than this [s]
  double vyaw_timing_max_dt    = 0.400;  // ignore if jump slower than this [s]
  // vyaw_fire_threshold [rad/s]: below this spin rate, only aim at the
  // currently visible face (single-face mode). Above this, use full four-face
  // spinning prediction. This prevents random aim during slow rotation when
  // vyaw is unreliable. 200 RPM = 20.9, 100 RPM = 10.5, 30 RPM = 3.1 rad/s.
  double vyaw_fire_threshold   = 3.0;

  // ── TARGET SWITCHING ──
  // POLICY: always prefer the closest enemy (highest hit probability).
  // Only stay on a farther target if:
  //   (a) it's confirmed TRACKING and cooldown hasn't elapsed, AND
  //   (b) the closer enemy isn't dramatically closer (within ratio)
  //
  // switch_range_ratio = 0.85 means: switch if new target is < 85% of
  // current range. This is aggressive — you almost always shoot the
  // closest robot. Set to 0.5 if you prefer to finish the current target.
  double switch_range_ratio = 0.85;
  // Hysteresis: don't switch back for N frames after switching.
  // Prevents jitter between two robots at similar range.
  int    switch_cooldown    = 10;  // ~0.33s at 30Hz
  // Spatial identity threshold [m]. If a new detection is within this distance
  // of the currently tracked target's predicted face position, it is treated
  // as the SAME physical robot (not a switch candidate). This is how the
  // tracker handles multiple enemies of the same color: class_id is color only,
  // identity comes from spatial proximity.
  // Tune up if robots can be back-to-back (false split into two tracks).
  // Tune down if two passing enemies get incorrectly merged into one track.
  double same_target_identity_dist = 1.0;
};

// =========================================================================
// Tracker — spinning-top EKF + aim solver + smart target switching.
//
// State: [xc, vxc, yc, vyc, za, vza, yaw, vyaw, r]  (9D)
// Observation: [xa, ya, za, yaw]  (4D)
//
// TARGET SWITCHING POLICY:
//   - Tracks the current target as long as it's TRACKING or TEMP_LOST
//   - Switches to a new target if:
//     (a) current target is LOST, OR
//     (b) a new detection is significantly closer (< switch_range_ratio × current range)
//     (c) and switch cooldown has elapsed
//   - This ensures you don't ignore a robot charging at you from 1m
//     while you're shooting at one at 5m
// =========================================================================
class Tracker
{
public:
  explicit Tracker(const TrackerConfig & cfg);

  /// Process one frame. dt is the real detection interval; defaults are tuned for fast auto-aim.
  /// All detections must be in odom frame (TF-transformed).
  /// now: current ROS time, used by the vyaw-from-timing estimator.
  void update(const std::vector<ArmorDetection> & detections, double dt,
              const rclcpp::Time & now);

  /// Tell the tracker where the robot is in odom frame. Used internally so
  /// targetRange() returns the distance from CAMERA to target (not from world
  /// origin), which is what shouldSwitch() and the range filter actually need.
  /// Call once per frame before update() and before computeAim().
  void setEgoPose(double robot_x, double robot_y)
  {
    ego_x_ = robot_x;
    ego_y_ = robot_y;
  }

  /// Translate the EKF target state by (dx, dy) in the world frame.
  /// Use when the node resets robot_x_/robot_y_ to zero due to drift cap.
  /// Without this shift the EKF would think the target jumped by exactly the
  /// reset amount and either reject all subsequent detections or go LOST.
  /// Pass dx = -robot_x_old, dy = -robot_y_old (the negative of the displacement
  /// that was reset away) — i.e. the target's coordinates also move by that
  /// amount in the new frame.
  void shiftWorldFrame(double dx, double dy)
  {
    x_(0) += dx;
    x_(2) += dy;
    // ego pose changes too — node will update it via setEgoPose next frame anyway,
    // but keep them consistent now to avoid one frame of bad targetRange().
    ego_x_ += dx;
    ego_y_ += dy;
    // Velocities are unaffected (purely translational shift).
  }

  /// Compute gimbal angles from current EKF state.
  /// robot_x/robot_y: current robot position in odom frame (from ego-motion integration).
  /// robot_vx/robot_vy: current robot velocity in odom/world frame [m/s].
  ///   Used to propagate the barrel position forward by (flight_time + time_bias),
  ///   so the aim accounts for our own motion during bullet flight. Pass 0,0 when
  ///   stationary or when ego_velocity_available is false.
  /// Required so the barrel-to-target vector is correct when the robot has moved
  /// and when the robot is still moving during bullet flight.
  AimResult computeAim(double current_yaw, double current_pitch,
                       double robot_x = 0.0, double robot_y = 0.0,
                       double robot_vx = 0.0, double robot_vy = 0.0) const;

  enum State { LOST, DETECTING, TRACKING, TEMP_LOST };
  State state() const { return state_; }
  const Eigen::VectorXd & ekfState() const { return x_; }
  double radius() const { return radius_; }
  double otherRadius() const { return other_radius_; }
  double dz() const { return dz_; }
  std::string targetId() const { return target_id_; }
  /// Monotonically increasing counter, bumped on every initFromDetection.
  /// The node compares this across frames to detect target switches.
  int targetGeneration() const { return target_generation_; }
  /// Current tracked target range (for debug / visualization)
  double targetRange() const;

private:
  void ekfPredict(double dt);
  Eigen::VectorXd ekfUpdate(const Eigen::Vector4d & z);
  double ekfMahalanobis(const Eigen::Vector4d & z) const;
  Eigen::Vector3d armorFromState(const Eigen::VectorXd & x) const;
  void initFromDetection(const ArmorDetection & det);
  void handleArmorJump(const ArmorDetection & det, const rclcpp::Time & now);
  double unwrapYaw(double raw_yaw);

  struct BallisticResult { double pitch, flight_time; bool valid; };
  BallisticResult solveBallistic(double ground_dist, double dz) const;

  /// Check if we should switch to a closer target
  bool shouldSwitch(const ArmorDetection & candidate) const;

  TrackerConfig cfg_;
  Eigen::VectorXd x_;
  Eigen::MatrixXd P_, P0_;

  State state_ = LOST;
  int detect_count_ = 0;
  int lost_count_ = 0;
  int lost_thresh_ = 12;
  int switch_cooldown_counter_ = 0;
  int target_generation_ = 0;
  // Robot position in odom frame (set by node each frame via setEgoPose).
  // Used so targetRange() returns the camera-to-target distance, not the
  // world-origin-to-target distance — which would be wrong once ego-motion
  // has moved the robot.
  double ego_x_ = 0.0;
  double ego_y_ = 0.0;

  double radius_ = 0.24;
  double other_radius_ = 0.28;
  double dz_ = 0.0;
  bool dz_initialized_ = false;
  double last_yaw_ = 0.0;
  std::string target_id_;

  // ── Spin rate estimation from face jump timing ──
  // Tracks the ROS time of the last face jump and the direction.
  // After two consecutive same-direction jumps, vyaw = (pi/2) / dt.
  // This is more accurate and faster-converging than EKF-only estimation,
  // especially during spin-up and at 200-300 RPM.
  rclcpp::Time last_jump_time_;
  bool last_jump_time_valid_ = false;
  double last_jump_dir_ = 0.0;   // sign of last jump (+1 CCW, -1 CW)
  int consecutive_same_dir_jumps_ = 0;
};

}  // namespace autoaim
#endif
