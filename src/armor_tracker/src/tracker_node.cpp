#include "armor_tracker/tracker_node.hpp"

#include "armor_tracker/armor.hpp"

// STD
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <opencv2/calib3d.hpp>

namespace rm_auto_aim
{
ArmorTrackerNode::ArmorTrackerNode(const rclcpp::NodeOptions & options)
: Node("armor_tracker", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting TrackerNode!");

  // Maximum allowable armor distance in the XOY plane
  max_armor_distance_ = this->declare_parameter("max_armor_distance", 10.0);

  // YOLO class IDs to track (0=blue, 1=grey/dead, 2=purple, 3=red)
  // Default: {"3"} = red only. Grey ("1") is always removed regardless of config.
  auto target_classes_vec = this->declare_parameter<std::vector<std::string>>(
    "target_classes", std::vector<std::string>{"3"});
  target_classes_ = std::set<std::string>(target_classes_vec.begin(), target_classes_vec.end());
  target_classes_.erase("1");  // grey/dead — never track
  std::string classes_str;
  for (const auto & c : target_classes_) {
    if (!classes_str.empty()) classes_str += ", ";
    classes_str += c;
  }
  RCLCPP_INFO(this->get_logger(),
    "Tracking %zu class(es): %s", target_classes_.size(), classes_str.c_str());

  // Ratio to scale YOLO bbox inward to approximate light-bar positions
  light_ratio_ = this->declare_parameter("light_ratio", 0.85);

  // Letterbox padding: when DNN encoder pads image to square with keep_aspect_ratio,
  // YOLO bbox Y coordinates are shifted by (network_h - image_h) / 2 pixels.
  // Default 80 = (640 - 480) / 2 for 640x480 input padded to 640x640.
  bbox_padding_y_ = this->declare_parameter("bbox_padding_y", 80.0);

  // Tracker
  double max_match_distance = this->declare_parameter("tracker.max_match_distance", 0.15);
  double max_track_range = this->declare_parameter("tracker.max_track_range", 6.0);
  tracker_ = std::make_unique<Tracker>(max_match_distance, max_track_range);
  tracker_->tracking_thres = this->declare_parameter("tracker.tracking_thres", 5);
  // max_yaw_oblique_angle_ is set later after max_yaw_oblique_deg_ is declared (in EKF section)
  lost_time_thres_ = this->declare_parameter("tracker.lost_time_thres", 0.3);
  // Geometry adaptation — r1 ≠ r2 (elliptical armor arrangement)
  // Scalar KFs for radius: adaptive gain (fast initial convergence, low steady-state noise)
  tracker_->initial_r1_         = this->declare_parameter("tracker.initial_r1", 0.22);
  tracker_->initial_r2_         = this->declare_parameter("tracker.initial_r2", 0.30);
  tracker_->r_kf_Q_             = this->declare_parameter("tracker.r_kf_Q", 3.3e-8);
  tracker_->r_kf_R_             = this->declare_parameter("tracker.r_kf_R", 0.0004);
  tracker_->r_kf_P_init_        = this->declare_parameter("tracker.r_kf_P_init", 0.0064);
  tracker_->r_adapt_max_dist_   = this->declare_parameter("tracker.r_adapt_max_dist", 4.0);
  tracker_->dz_adapt_alpha_     = this->declare_parameter("tracker.dz_adapt_alpha", 0.2);

  // EKF
  // xa = x_armor, xc = x_robot_center
  // state: xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r
  // measurement: xa, ya, za, yaw
  // Velocity damping: models friction/deceleration as v_new = alpha^(dt/T) * v
  // T = 1/30 (reference period), so alpha is "per-frame decay at 30Hz"
  // alpha=0.95 → half-life ~0.45s (linear), alpha=0.98 → half-life ~1.1s (yaw)
  xyz_damping_alpha_base_ = declare_parameter("ekf.xyz_damping_alpha", 0.95);
  yaw_damping_alpha_base_ = declare_parameter("ekf.yaw_damping_alpha", 0.95);
  coast_damping_factor_   = declare_parameter("ekf.coast_damping_factor", 0.85);
  damping_innov_threshold_ = declare_parameter("ekf.damping_innov_threshold", 0.10);
  yaw_innov_threshold_    = declare_parameter("ekf.yaw_innov_threshold", 0.15);
  xyz_damping_alpha_ = xyz_damping_alpha_base_;
  yaw_damping_alpha_ = yaw_damping_alpha_base_;
  // --- f: EKF Process Function (what the robot does between frames) ---
  //
  // WHY DAMPED CONSTANT-VELOCITY?
  //   We don't model acceleration as a state (that would add 3+ dimensions and
  //   require much more data to converge).  Instead we assume the robot moves
  //   at roughly constant velocity, with a per-frame velocity decay that acts
  //   like friction.  This is good enough for ~30ms prediction horizons.
  //
  //   The damping (alpha < 1) serves two purposes:
  //   1. Physical: real robots decelerate when motors stop.  Pure constant-
  //      velocity would overshoot forever.
  //   2. Practical: it makes the EKF self-correcting.  If v_yaw gets corrupted
  //      by a bad measurement, damping naturally shrinks it toward zero over
  //      a few frames, instead of letting the error persist indefinitely.
  //
  //   Alpha is normalised to a 30Hz reference so the same parameter value
  //   gives the same physical behaviour regardless of actual frame rate.
  //   alpha=0.95 at 30Hz → half-life of ~0.45 seconds.
  auto f = [this](const Eigen::VectorXd & x) {
    Eigen::VectorXd x_new = x;
    // Per-step decay factors, scaled to the actual dt so alpha is frame-rate independent.
    double b = std::pow(xyz_damping_alpha_, dt_ / (1.0 / 30.0));  // XYZ velocity decay
    double a = std::pow(yaw_damping_alpha_, dt_ / (1.0 / 30.0));  // Yaw velocity decay
    double damped_vx   = x(1) * b;
    double damped_vy   = x(3) * b;
    double damped_vz   = x(5) * b;
    double damped_vyaw = x(7) * a;
    // Integrate damped velocity into position for each axis.
    x_new(0) += damped_vx * dt_;    // xc  += v_xc * dt
    x_new(1) = damped_vx;           // v_xc = v_xc * b
    x_new(2) += damped_vy * dt_;    // yc  += v_yc * dt
    x_new(3) = damped_vy;           // v_yc = v_yc * b
    x_new(4) += damped_vz * dt_;    // za  += v_za * dt
    x_new(5) = damped_vz;           // v_za = v_za * b
    x_new(6) += damped_vyaw * dt_;  // yaw += v_yaw * dt
    x_new(7) = damped_vyaw;         // v_yaw = v_yaw * a
    // x(8) = r remains unchanged (constant radius assumption)
    return x_new;
  };
  // --- J_f: Jacobian of the Process Function ---
  // The EKF linearizes f around the current state each step using this matrix.
  // Because f is linear (constant-velocity + decay), J_f is exact (no approximation).
  // Block structure: each axis [pos, vel] is a 2×2 block [1, b*dt; 0, b].
  // Yaw uses decay 'a' instead of 'b'. Radius row/col = identity (r is constant).
  auto j_f = [this](const Eigen::VectorXd &) {
    double b = std::pow(xyz_damping_alpha_, dt_ / (1.0 / 30.0));  // XYZ decay factor
    double a = std::pow(yaw_damping_alpha_, dt_ / (1.0 / 30.0));  // Yaw decay factor
    Eigen::MatrixXd f(9, 9);
    // clang-format off
    //    xc     v_xc   yc     v_yc   za     v_za   yaw    v_yaw  r
    f <<  1,   b*dt_, 0,   0,      0,   0,      0,   0,      0,   // d(xc)/d(*)
          0,   b,     0,   0,      0,   0,      0,   0,      0,   // d(v_xc)/d(*)
          0,   0,     1,   b*dt_,  0,   0,      0,   0,      0,   // d(yc)/d(*)
          0,   0,     0,   b,      0,   0,      0,   0,      0,   // d(v_yc)/d(*)
          0,   0,     0,   0,      1,   b*dt_,  0,   0,      0,   // d(za)/d(*)
          0,   0,     0,   0,      0,   b,      0,   0,      0,   // d(v_za)/d(*)
          0,   0,     0,   0,      0,   0,      1,   a*dt_,  0,   // d(yaw)/d(*)
          0,   0,     0,   0,      0,   0,      0,   a,      0,   // d(v_yaw)/d(*)
          0,   0,     0,   0,      0,   0,      0,   0,      1;   // d(r)/d(*) — constant
    // clang-format on
    return f;
  };
  // --- h: Observation Function ---
  // Maps the 9D state to the 4D measurement that we actually observe:
  //   z = [xa, ya, za, yaw]
  //
  // The armor plate is offset from the robot center by radius r:
  //   xa = xc − r·cos(yaw)    (armor X position)
  //   ya = yc − r·sin(yaw)    (armor Y position)
  //   za = za                  (armor Z = tracked directly)
  //   yaw = yaw                (observed = internal yaw)
  //
  // This is the "spinning top" observation: the camera sees a plate on the
  // rim of a spinning disk, and we infer the disk center from it.
  auto h = [](const Eigen::VectorXd & x) {
    Eigen::VectorXd z(4);
    double xc = x(0), yc = x(2), yaw = x(6), r = x(8);
    z(0) = xc - r * cos(yaw);  // xa: armor plate X
    z(1) = yc - r * sin(yaw);  // ya: armor plate Y
    z(2) = x(4);               // za: armor plate Z (= state directly)
    z(3) = x(6);               // yaw: robot heading (= state directly)
    return z;
  };
  // --- J_h: Jacobian of the Observation Function ---
  // H = ∂h/∂x, a 4×9 matrix.  Most entries are 0 because each measurement
  // depends on only a few state variables.  Non-trivial partial derivatives:
  //
  //   ∂xa/∂xc  = 1               (armor X depends linearly on center X)
  //   ∂xa/∂yaw = r·sin(yaw)      (rotating the disk moves the plate tangentially)
  //   ∂xa/∂r   = −cos(yaw)       (larger radius pushes plate further from center)
  //
  //   ∂ya/∂yc  = 1               (same logic for Y axis)
  //   ∂ya/∂yaw = −r·cos(yaw)     (tangential coupling in Y)
  //   ∂ya/∂r   = −sin(yaw)       (radial coupling in Y)
  //
  //   ∂za/∂za  = 1               (direct pass-through)
  //   ∂yaw/∂yaw = 1              (direct pass-through)
  //
  // All velocity columns (v_xc, v_yc, v_za, v_yaw) are 0: velocities don't
  // appear in h() — they affect the *next* prediction, not the current measurement.
  auto j_h = [](const Eigen::VectorXd & x) {
    Eigen::MatrixXd h(4, 9);
    double yaw = x(6), r = x(8);
    // clang-format off
    //    xc   v_xc yc   v_yc za   v_za yaw          v_yaw r
    h <<  1,   0,   0,   0,   0,   0,   r*sin(yaw),  0,   -cos(yaw),
          0,   0,   1,   0,   0,   0,  -r*cos(yaw),  0,   -sin(yaw),
          0,   0,   0,   0,   1,   0,   0,            0,   0,
          0,   0,   0,   0,   0,   0,   1,            0,   0;
    // clang-format on
    return h;
  };
  // --- update_Q: Process Noise Covariance Matrix ---
  // Q models how much the state can change due to unmodeled dynamics (random
  // accelerations, vibrations, etc.) between consecutive frames.
  //
  // Derivation: assume piece-wise constant acceleration noise a ~ N(0, σ²).
  // Integrating over timestep dt gives the standard discrete-time noise model:
  //   Q_pos_pos   = σ² · dt⁴/4    (position uncertainty from acceleration)
  //   Q_pos_vel   = σ² · dt³/2    (cross-correlation between pos and vel)
  //   Q_vel_vel   = σ² · dt²      (velocity uncertainty from acceleration)
  //
  // Each [pos, vel] pair (xc/v_xc, yc/v_yc, za/v_za) uses σ²_xyz.
  // The [yaw, v_yaw] pair uses σ²_yaw (different because rotational dynamics
  // are noisier than translational — spinning motors, backlash, etc.).
  //
  // σ²_r is near-zero because radius is managed by external scalar KFs;
  // decoupleState() overwrites P_rr every frame anyway.
  s2qxyz_ = declare_parameter("ekf.sigma2_q_xyz", 5.0);
  s2qyaw_ = declare_parameter("ekf.sigma2_q_yaw", 10.0);
  s2qr_   = declare_parameter("ekf.sigma2_q_r",   1e-6);
  auto u_q = [this]() {
    Eigen::MatrixXd q(9, 9);
    double t = dt_, x = s2qxyz_, y = s2qyaw_, r = s2qr_;
    // XYZ block: σ²_xyz integrated over dt (pos-vel cross-correlation)
    double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
    // Yaw block: same structure but with σ²_yaw (typically larger than σ²_xyz)
    double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * y, q_vy_vy = pow(t, 2) * y;
    // Radius block: near-zero (externally managed)
    double q_r = pow(t, 4) / 4 * r;
    // clang-format off
    //    xc      v_xc    yc      v_yc    za      v_za    yaw     v_yaw   r
    q <<  q_x_x,  q_x_vx, 0,      0,      0,      0,      0,      0,      0,
          q_x_vx, q_vx_vx,0,      0,      0,      0,      0,      0,      0,
          0,      0,      q_x_x,  q_x_vx, 0,      0,      0,      0,      0,
          0,      0,      q_x_vx, q_vx_vx,0,      0,      0,      0,      0,
          0,      0,      0,      0,      q_x_x,  q_x_vx, 0,      0,      0,
          0,      0,      0,      0,      q_x_vx, q_vx_vx,0,      0,      0,
          0,      0,      0,      0,      0,      0,      q_y_y,  q_y_vy, 0,
          0,      0,      0,      0,      0,      0,      q_y_vy, q_vy_vy,0,
          0,      0,      0,      0,      0,      0,      0,      0,      q_r;
    // clang-format on
    return q;
  };
  // --- update_R: Measurement Noise Covariance Matrix ---
  // R models how noisy the PnP measurement is.  It depends on two factors:
  //
  // 1. DISTANCE: further targets have larger pixel-space error because the
  //    armor plate subtends fewer pixels → PnP becomes less accurate.
  //    R_pos = (base + slope × distance)²   (linear in distance, squared for variance)
  //    R_yaw = (base + slope × distance)²
  //
  // 2. OBLIQUE VIEWING ANGLE: when the camera views the armor plate at a
  //    steep angle, the plate appears as a thin line and PnP yaw is poorly
  //    constrained.  We inflate noise based on face_angle (0=face-on, π/2=edge):
  //    XYZ noise × 1/cos²(angle)    — geometric depth uncertainty
  //    Yaw noise × 1/cosⁿ(angle)    — PnP orientation degrades much faster
  //
  // At extreme obliquity (>65°), yaw_angle_factor → 1e6, which makes the
  // Kalman gain for yaw ≈ 0 → the filter effectively ignores the yaw
  // measurement while still fusing the XYZ position.
  r_xyz_base_  = declare_parameter("ekf.r_xyz_base",  0.04);
  r_xyz_slope_ = declare_parameter("ekf.r_xyz_slope", 0.03);
  r_yaw_base_  = declare_parameter("ekf.r_yaw_base",  0.05);
  r_yaw_slope_ = declare_parameter("ekf.r_yaw_slope", 0.002);
  r_yaw_angle_power_    = declare_parameter("ekf.r_yaw_angle_power",    4.0);
  max_yaw_oblique_deg_  = declare_parameter("ekf.max_yaw_oblique_deg",  65.0);
  auto u_r = [this](const Eigen::VectorXd & z) {
    Eigen::DiagonalMatrix<double, 4> r;
    double dist = std::sqrt(z[0]*z[0] + z[1]*z[1] + z[2]*z[2]);
    double ps = r_xyz_base_ + r_xyz_slope_ * dist;
    double ys = r_yaw_base_ + r_yaw_slope_ * dist;

    // Angle-aware noise: PnP accuracy degrades with oblique viewing angle.
    // XYZ degrades as ~1/cos²(angle); yaw degrades much faster (~1/cos⁴)
    // because at high obliquity the plate is a thin line and PnP yaw is
    // nearly unresolvable.
    double xyz_angle_factor;
    double yaw_angle_factor;
    if (z[0] * z[0] + z[1] * z[1] < 1e-12) {
      xyz_angle_factor = 1.0;
      yaw_angle_factor = 1.0;
    } else {
      double bearing_opp = std::atan2(-z[1], -z[0]);
      double face_angle = std::abs(std::remainder(z[3] - bearing_opp, 2.0 * M_PI));
      double cos_fa = std::cos(face_angle);

      // XYZ: 1/cos²(angle), capped at 25x (same as before)
      xyz_angle_factor = 1.0 / std::max(cos_fa * cos_fa, 0.04);

      // Yaw: 1/cos^p(angle) with configurable power, much steeper degradation
      double cos_pow = std::pow(std::abs(cos_fa), r_yaw_angle_power_);
      yaw_angle_factor = 1.0 / std::max(cos_pow, 1e-4);

      // Hard gate: beyond max oblique angle, yaw measurement is garbage —
      // set noise so high that Kalman gain for yaw → 0
      double max_oblique_rad = max_yaw_oblique_deg_ * M_PI / 180.0;
      if (face_angle > max_oblique_rad) {
        yaw_angle_factor = 1e6;
      }
    }

    r.diagonal() << ps*ps*xyz_angle_factor, ps*ps*xyz_angle_factor,
                     ps*ps*xyz_angle_factor, ys*ys*yaw_angle_factor;
    return r;
  };
  // P - initial error covariance
  Eigen::DiagonalMatrix<double, 9> p0;
  //       xc    v_xc  yc    v_yc  za    v_za  yaw   v_yaw  r
  // P_rr small: scalar KFs own radius estimation; decoupleState() overwrites each frame.
  p0.diagonal() << 0.1, 1.0, 0.1, 1.0, 0.1, 0.2, 0.1, 3.0, 0.001;
  tracker_->ekf = ExtendedKalmanFilter{f, h, j_f, j_h, u_q, u_r, p0};

  // Sync oblique angle threshold to tracker for yaw gate bypass
  tracker_->setMaxYawObliqueAngle(max_yaw_oblique_deg_ * M_PI / 180.0);

  // Covariance upper bounds — safety net against P explosion during TEMP_LOST
  Eigen::VectorXd max_cov(9);
  //          xc    v_xc   yc    v_yc   za    v_za   yaw    v_yaw   r
  // r max_cov >= r_kf_P_init (0.0064) so the clamp doesn't fight the scalar KF.
  max_cov << 1.0,  10.0,  1.0,  10.0,  1.0,  2.0,   1.0,   30.0,  0.01;
  tracker_->ekf.max_covariance = max_cov;

  // Reset tracker service
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  reset_tracker_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "/tracker/reset", [this](
                        const std_srvs::srv::Trigger::Request::SharedPtr,
                        std_srvs::srv::Trigger::Response::SharedPtr response) {
      tracker_->tracker_state = Tracker::LOST;
      response->success = true;
      RCLCPP_INFO(this->get_logger(), "Tracker reset!");
      return;
    });

  // Subscriber with tf2 message_filter
  // tf2 relevant
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  // Create the timer interface before call to waitForTransform,
  // to avoid a tf2_ros::CreateTimerInterfaceException exception
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  // sottoscrizione diretta a yolo, se non si aggira il filtro non funziona
  target_frame_ = this->declare_parameter("target_frame", "odom");
  armors_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
    "/detector/armors", rclcpp::SensorDataQoS(),
    std::bind(&ArmorTrackerNode::armorsCallback, this, std::placeholders::_1));

  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera_info", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
      cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
      cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
      pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
      cam_info_sub_.reset();
    });

  // Measurement publisher (for debug usage)
  info_pub_ = this->create_publisher<auto_aim_interfaces::msg::TrackerInfo>("/tracker/info", 10);

  // Publisher — RELIABLE so the trajectory solver never misses a target update
  auto target_qos = rclcpp::SensorDataQoS()
    .reliability(rclcpp::ReliabilityPolicy::Reliable);
  target_pub_ = this->create_publisher<auto_aim_interfaces::msg::Target>(
    "/tracker/target", target_qos);

  // Optimal BBox Publisher
  optimal_bbox_pub_ = this->create_publisher<vision_msgs::msg::Detection2D>(
    "/detections_output/optimal_target", rclcpp::SensorDataQoS());

  // Visualization Marker Publisher
  // See http://wiki.ros.org/rviz/DisplayTypes/Marker
  position_marker_.ns = "position";
  position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  position_marker_.pose.orientation.w = 1.0;
  position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
  position_marker_.color.a = 1.0;
  position_marker_.color.g = 1.0;
  linear_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  linear_v_marker_.ns = "linear_v";
  linear_v_marker_.scale.x = 0.03;
  linear_v_marker_.scale.y = 0.05;
  linear_v_marker_.color.a = 1.0;
  linear_v_marker_.color.r = 1.0;
  linear_v_marker_.color.g = 1.0;
  angular_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  angular_v_marker_.ns = "angular_v";
  angular_v_marker_.scale.x = 0.03;
  angular_v_marker_.scale.y = 0.05;
  angular_v_marker_.color.a = 1.0;
  angular_v_marker_.color.b = 1.0;
  angular_v_marker_.color.g = 1.0;
  armor_marker_.ns = "armors";
  armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
  armor_marker_.scale.x = 0.03;
  armor_marker_.scale.z = 0.125;
  armor_marker_.color.a = 1.0;
  armor_marker_.color.r = 1.0;
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/tracker/marker", 10);

  // Dynamic TF from gimbal feedback
  tf2_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  gimbal_yaw_ = 0.0;
  gimbal_pitch_ = 0.0;
  gimbal_height_ = declare_parameter("gimbal.height", 0.5);
  yaw_sign_      = declare_parameter("gimbal.yaw_sign", 1.0);
  pitch_sign_    = declare_parameter("gimbal.pitch_sign", 1.0);
  micro_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/micro_pose", rclcpp::SensorDataQoS(),
    std::bind(&ArmorTrackerNode::microPoseCallback, this, std::placeholders::_1));

  // IMU-based chassis rotation compensation
  enable_imu_compensation_ = declare_parameter("imu.enable", true);
  imu_gyro_alpha_ = declare_parameter("imu.gyro_alpha", 0.3);
  imu_timeout_ = declare_parameter("imu.timeout", 0.1);

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu", rclcpp::SensorDataQoS(),
    std::bind(&ArmorTrackerNode::imuCallback, this, std::placeholders::_1));
}



void ArmorTrackerNode::microPoseCallback(
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  // Extract gimbal angles from the lower computer.
  // cmd_vel_subscriber stores: position.x = -pitch, position.y = -yaw (radians)
  double p = pitch_sign_ * msg->pose.position.x;
  double y = yaw_sign_ * msg->pose.position.y;
  // Reject garbage serial data — gimbal angles physically can't exceed ~90°
  if (std::abs(p) < 1.6 && std::abs(y) < 1.6) {
    gimbal_yaw_   = y;
    gimbal_pitch_ = p;
  }
  broadcastGimbalTF(msg->header.stamp);
}

void ArmorTrackerNode::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  if (!enable_imu_compensation_) {
    return;
  }

  rclcpp::Time now = msg->header.stamp;
  last_imu_time_ = now;

  if (!imu_initialized_) {
    prev_imu_time_ = now;
    prev_gimbal_yaw_ = gimbal_yaw_;
    prev_gimbal_pitch_ = gimbal_pitch_;
    imu_initialized_ = true;
    imu_active_ = true;
    return;
  }

  double dt = (now - prev_imu_time_).seconds();
  if (dt <= 0.0 || dt > 0.5) {
    prev_imu_time_ = now;
    prev_gimbal_yaw_ = gimbal_yaw_;
    prev_gimbal_pitch_ = gimbal_pitch_;
    return;
  }

  // D455 IMU in camera_color_optical_frame (X-right, Y-down, Z-forward).
  // First, apply EMA filter to raw gyro (rad/s)
  imu_wx_filtered_ = imu_gyro_alpha_ * msg->angular_velocity.x + (1.0 - imu_gyro_alpha_) * imu_wx_filtered_;
  imu_wy_filtered_ = imu_gyro_alpha_ * msg->angular_velocity.y + (1.0 - imu_gyro_alpha_) * imu_wy_filtered_;
  imu_wz_filtered_ = imu_gyro_alpha_ * msg->angular_velocity.z + (1.0 - imu_gyro_alpha_) * imu_wz_filtered_;

  // Construct rotation from Camera Optical Frame -> Chassis Frame.
  //   q_camera_to_chassis = q_gimbal_yaw * q_gimbal_pitch * q_convention
  tf2::Quaternion q_yaw, q_pitch, q_convention;
  q_yaw.setRPY(0, 0, gimbal_yaw_);         // Rotation around world Z
  q_pitch.setRPY(0, gimbal_pitch_, 0);     // Rotation around gimbal Y
  q_convention.setRPY(-M_PI / 2.0, 0, -M_PI / 2.0); // Optical frame convention

  tf2::Quaternion q_c2chassis = q_yaw * q_pitch * q_convention;
  q_c2chassis.normalize();

  // Total angular velocity vector measured by camera IMU (in camera frame)
  tf2::Vector3 omega_c_in_c(imu_wx_filtered_, imu_wy_filtered_, imu_wz_filtered_);

  // Rotate angular velocity vector to chassis frame
  tf2::Vector3 omega_c_in_chassis = tf2::quatRotate(q_c2chassis, omega_c_in_c);

  // Reconstruct gimbal's angular velocity relative to chassis
  // Gimbal pitches around its Y axis, yaws around Chassis Z axis.
  double gimbal_wz = (gimbal_yaw_ - prev_gimbal_yaw_) / dt;
  double gimbal_wy = (gimbal_pitch_ - prev_gimbal_pitch_) / dt;
  
  // Total angular velocity = chassis velocity + gimbal velocity. 
  //   omega_c_in_chassis = omega_chassis + omega_gimbal.
  // We want to isolate omega_chassis.
  // Note: gimbal pitch is a rotation around the yawed Y axis, so we must separate them carefully.
  // A simpler and sufficiently accurate assumption: 
  //   chassis pitch/yaw velocity is total velocity minus the change in gimbal angles.
  double chassis_wy = omega_c_in_chassis.y() - gimbal_wy;
  double chassis_wz = omega_c_in_chassis.z() - gimbal_wz;

  // Integrate chassis rotation (Euler integration)
  // Yaw is pure Z rotation. Pitch is pure Y rotation. (Roll / X is ignored for trajectory)
  chassis_yaw_ += chassis_wz * dt;
  chassis_pitch_ += chassis_wy * dt;

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
      "IMU chassis_yaw=%.4f chassis_pitch=%.4f gimbal_yaw=%.4f gimbal_pitch=%.4f wz=%.4f wy=%.4f",
      chassis_yaw_, chassis_pitch_, gimbal_yaw_, gimbal_pitch_, chassis_wz, chassis_wy);

  imu_active_ = true;
  prev_imu_time_ = now;
  prev_gimbal_yaw_ = gimbal_yaw_;
  prev_gimbal_pitch_ = gimbal_pitch_;
}

// ---------------------------------------------------------------------------
// broadcastGimbalTF — publish the transform from "odom" to "camera_color_optical_frame".
//
// This transform tells TF2 where the camera is in world coordinates so that
// armor positions measured in camera frame can be converted to odom frame.
//
// Decomposition (right-to-left multiplication order):
//
//   T(odom → camera) = Translate(0, 0, gimbal_height)   ← vertical offset
//     × R_z(chassis_yaw) × R_y(chassis_pitch)           ← chassis ego-motion (from IMU)
//     × R_z(gimbal_yaw) × R_y(gimbal_pitch)             ← gimbal joints (from lower computer)
//     × R_convention                                     ← optical frame convention
//
// R_convention = RPY(−π/2, 0, −π/2) converts from the ROS camera_color_optical_frame
// convention (X-right, Y-down, Z-forward) to the robotics convention (X-forward,
// Y-left, Z-up).  When both gimbal angles are 0, this matches the old static TF.
//
// The transform is written directly into tf2_buffer_ (via setTransform) so
// that lookupTransform() later in the same callback sees it immediately,
// even before the /tf topic message is received by other subscribers.
// ---------------------------------------------------------------------------
void ArmorTrackerNode::broadcastGimbalTF(const rclcpp::Time & stamp)
{

  // Chassis rotation from IMU (identity if IMU inactive/stale)
  tf2::Quaternion q_chassis_yaw, q_chassis_pitch;
  q_chassis_yaw.setRPY(0, 0, 0);
  q_chassis_pitch.setRPY(0, 0, 0);

  if (enable_imu_compensation_ && imu_active_) {
    double staleness = (stamp - last_imu_time_).seconds();
    if (staleness < imu_timeout_) {
      q_chassis_yaw.setRPY(0, 0, chassis_yaw_);
      q_chassis_pitch.setRPY(0, chassis_pitch_, 0);
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "IMU data stale (%.3fs > %.3fs), falling back to gimbal-only TF",
        staleness, imu_timeout_);
    }
  }

  // Gimbal rotation from lower computer feedback
  tf2::Quaternion q_yaw, q_pitch, q_convention;
  q_yaw.setRPY(0, 0, gimbal_yaw_);
  q_pitch.setRPY(0, gimbal_pitch_, 0);
  // R_convention = RPY(-π/2, 0, -π/2) matches the old static TF when yaw=pitch=0
  q_convention.setRPY(-M_PI / 2.0, 0, -M_PI / 2.0);

  tf2::Quaternion q_final = q_chassis_yaw * q_chassis_pitch * q_yaw * q_pitch * q_convention;
  q_final.normalize();

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = stamp;
  t.header.frame_id = "odom";
  t.child_frame_id = "camera_color_optical_frame";
  t.transform.translation.x = 0.0;
  t.transform.translation.y = 0.0;
  t.transform.translation.z = gimbal_height_;
  t.transform.rotation = tf2::toMsg(q_final);

  // Write directly into our own buffer so the transform is available immediately
  // for the lookupTransform() call later in this same callback.
  // The broadcast still publishes on /tf for other nodes (e.g. rviz, tf2_echo).
  tf2_buffer_->setTransform(t, "armor_tracker");
  tf2_broadcaster_->sendTransform(t);
}

// ---------------------------------------------------------------------------
// armorsCallback — main pipeline entry point, called once per YOLO detection frame.
//
// Processing steps:
//   1. Broadcast gimbal TF (odom → camera) so TF2 can transform this frame.
//   2. For each YOLO bounding box:
//      a. Subtract letterbox padding (YOLO pads images to square).
//      b. Filter by YOLO class ID (only track configured enemy color).
//      c. Scale bbox inward by light_ratio_ to approximate light-bar corners.
//      d. Run PnP solver → 3D position + orientation in camera frame.
//   3. Transform all armor poses from camera frame → odom frame via TF2.
//   4. Filter out armors with physically impossible Z or beyond max range.
//   5. Feed the Armors message into the Tracker (init or update).
//   6. Publish Target msg, TrackerInfo debug msg, optimal bbox, and RViz markers.
// ---------------------------------------------------------------------------
void ArmorTrackerNode::armorsCallback(
  const vision_msgs::msg::Detection2DArray::ConstSharedPtr detection_msg)
{
  // Always broadcast gimbal TF so the transform is available for this frame.
  // When /micro_pose is active, angles update at 100Hz via microPoseCallback.
  // When /micro_pose is absent or dies, this keeps TF alive at detection rate
  // using last-known angles (0,0 at startup = old static TF equivalent).
  broadcastGimbalTF(detection_msg->header.stamp);

  // Convert 2D detections to Armors message
  auto_aim_interfaces::msg::Armors armors_msg;
  armors_msg.header = detection_msg->header;

  if (armors_msg.header.frame_id.empty()) {
    armors_msg.header.frame_id = "camera_color_optical_frame";
  }
  
  std::vector<size_t> detection_indices;

  if (pnp_solver_ == nullptr) {
    return;
  }

  size_t detection_idx = 0;
  for (const auto & detection : detection_msg->detections) {
    auto_aim_interfaces::msg::Armor armor_msg;
    // Extract bbox (decoder outputs coordinates in padded network space)
    auto center_x = detection.bbox.center.position.x;
    auto center_y = detection.bbox.center.position.y - bbox_padding_y_;
    auto width = detection.bbox.size_x;
    auto height = detection.bbox.size_y;

    // Skip degenerate detections (avoids divide-by-zero below)
    if (height < 1.0 || width < 1.0) {
      detection_idx++;
      continue;
    }

    // Filter by YOLO class ID — skip teammates, dead robots, etc.
    if (!detection.results.empty()) {
      const auto & class_id = detection.results[0].hypothesis.class_id;
      if (target_classes_.find(class_id) == target_classes_.end()) {
        detection_idx++;
        continue;
      }
    }

    // Build an Armor object from the YOLO bbox for PnP solving.
    // WHY PnP FROM BBOXES?  YOLO gives us 2D bounding boxes, but the EKF
    // needs 3D positions.  PnP (Perspective-n-Point) recovers the 3D pose
    // from 2D-3D point correspondences.  We approximate the 4 light-bar
    // corners from the bbox, then PnP finds the rotation + translation
    // that best maps the known real-world armor dimensions to those 2D points.
    Armor armor_obj;
    armor_obj.center = cv::Point2f(center_x, center_y);

    // Heuristic for Armor Type (plate aspect: small~1.12, large~1.85)
    float ratio = width / height;
    armor_obj.type = (ratio > 1.5) ? ArmorType::LARGE : ArmorType::SMALL;
    armor_obj.number = detection.results.empty() ? "unknown" : detection.results[0].hypothesis.class_id;
    armor_obj.confidence = detection.results.empty() ? 0.0 : detection.results[0].hypothesis.score;
    // Map class_id to type if possible, but ratio heuristic is good fallback
    // Set classification result for debug
    armor_obj.classfication_result = armor_obj.number;

    // Scale bbox inward by light_ratio to approximate light-bar corners.
    // YOLO bbox covers the full armor plate, which is wider than the light bars.
    const float lr = static_cast<float>(light_ratio_);
    const float lx = center_x - lr * width / 2;
    const float rx = center_x + lr * width / 2;
    const float ty = center_y - lr * height / 2;
    const float by = center_y + lr * height / 2;

    armor_obj.left_light.center = cv::Point2f(lx, center_y);
    armor_obj.left_light.top    = cv::Point2f(lx, ty);
    armor_obj.left_light.bottom = cv::Point2f(lx, by);
    armor_obj.right_light.center = cv::Point2f(rx, center_y);
    armor_obj.right_light.top   = cv::Point2f(rx, ty);
    armor_obj.right_light.bottom = cv::Point2f(rx, by);

    cv::Mat rvec, tvec;
    bool success = pnp_solver_->solvePnP(armor_obj, rvec, tvec);
    if (success) {
      armor_msg.type = ARMOR_TYPE_STR[static_cast<int>(armor_obj.type)];
      armor_msg.number = armor_obj.number;

      // Fill pose
      armor_msg.pose.position.x = tvec.at<double>(0);
      armor_msg.pose.position.y = tvec.at<double>(1);
      armor_msg.pose.position.z = tvec.at<double>(2);

      cv::Mat rotation_matrix;
      cv::Rodrigues(rvec, rotation_matrix);
      tf2::Matrix3x3 tf2_rotation_matrix(
        rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
        rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0),
        rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
        rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
        rotation_matrix.at<double>(2, 2));
      tf2::Quaternion tf2_q;
      tf2_rotation_matrix.getRotation(tf2_q);
      armor_msg.pose.orientation = tf2::toMsg(tf2_q);

      armor_msg.distance_to_image_center = pnp_solver_->calculateDistanceToCenter(armor_obj.center);

      armors_msg.armors.emplace_back(armor_msg);
      detection_indices.push_back(detection_idx);
    }
    detection_idx++;
  }

  // Pointer to the converted message
  auto armors_ptr = std::make_shared<auto_aim_interfaces::msg::Armors>(armors_msg);

  // Transform armor positions from camera frame to odom (world) frame.
  // WHY: PnP gives positions relative to the camera.  But the EKF tracks the
  // robot in world coordinates (odom frame), so we need to know where the camera
  // is in the world.  The gimbal TF (broadcast above) provides this transform.
  // Without it, every time the gimbal moves, the target would appear to jump
  // in camera frame even though it's stationary in the world.
  if (!armors_ptr->armors.empty()) {
    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
      tf_stamped = tf2_buffer_->lookupTransform(
        target_frame_, armors_ptr->header.frame_id,
        armors_ptr->header.stamp, rclcpp::Duration::from_seconds(0.01));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(get_logger(), "TF2 error: %s", ex.what());
      return;
    }
    for (auto & armor : armors_ptr->armors) {
      geometry_msgs::msg::PoseStamped ps_in, ps_out;
      ps_in.header = armors_ptr->header;
      ps_in.pose = armor.pose;
      tf2::doTransform(ps_in, ps_out, tf_stamped);
      armor.pose = ps_out.pose;
    }
  }

  // Filter physically impossible detections.
  // WHY: PnP can produce wildly wrong 3D positions from noisy bboxes, especially
  // at frame edges or with partial occlusions.  An armor at Z=2.0m (2 meters
  // above the camera) or 15m away horizontally is obviously wrong — no robot
  // is that tall or that far.  Filtering here prevents these garbage measurements
  // from ever reaching the tracker, where they could corrupt the EKF state.
  auto & armors_vec = armors_ptr->armors;

  // Log yaw for all detections (debug)
  for (size_t i = 0; i < armors_vec.size(); i++) {
    const auto & a = armors_vec[i];
    tf2::Quaternion q;
    tf2::fromMsg(a.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    RCLCPP_DEBUG(get_logger(),
      "Det[%zu] id=%s yaw=%.3f pos=(%.2f, %.2f, %.2f)",
      i, a.number.c_str(), yaw,
      a.pose.position.x, a.pose.position.y, a.pose.position.z);
  }

  for (size_t i = 0; i < armors_vec.size(); /* no increment */) {
    auto & armor = armors_vec[i];
    if (
      std::abs(armor.pose.position.z) > 1.2 ||
      Eigen::Vector2d(armor.pose.position.x, armor.pose.position.y).norm() > max_armor_distance_) {
      armors_vec.erase(armors_vec.begin() + i);
      detection_indices.erase(detection_indices.begin() + i);
    } else {
      i++;
    }
  }

  // Init message
  auto_aim_interfaces::msg::TrackerInfo info_msg;
  auto_aim_interfaces::msg::Target target_msg;
  rclcpp::Time time = armors_ptr->header.stamp;
  target_msg.header.stamp = time;
  target_msg.header.frame_id = target_frame_;

  // Update tracker
  if (tracker_->tracker_state == Tracker::LOST) {
    tracker_->init(armors_ptr);
    target_msg.tracking = false;
  } else {
    // Compute timestep dt, clamped to [10ms, 100ms].
    // Lower bound prevents division-by-zero in velocity estimates.
    // Upper bound prevents the EKF from making a huge state jump after a
    // long gap (e.g. YOLO stall), which would corrupt position and velocity.
    dt_ = std::min(std::max((time - last_time_).seconds(), 0.01), 0.10);
    // Convert lost_time_thres_ (seconds) to a frame count based on current dt.
    // E.g. 0.3s at 30fps ≈ 9 frames of coasting before LOST.
    tracker_->lost_thres = static_cast<int>(lost_time_thres_ / dt_);

    // === ADAPTIVE VELOCITY DAMPING ===
    // WHY THIS EXISTS:
    //   A pure constant-velocity EKF has a fundamental problem: when the target
    //   stops moving, the filter's velocity estimate doesn't go to zero — it
    //   slowly decays over many frames because there's no direct "acceleration"
    //   state.  Meanwhile, the filter keeps predicting the target will move,
    //   creating persistent overshooting that makes the aim wobble.
    //
    //   Conversely, if we always apply strong damping, the filter can't track
    //   fast-moving targets because it constantly brakes the velocity estimate.
    //
    // SOLUTION: Adapt the damping strength each frame based on the innovation
    //   signal (the difference between what we predicted and what we measured).
    //
    //   - If the innovation OPPOSES the velocity (overshooting), increase damping
    //     to brake the filter.
    //   - If the innovation AGREES with the velocity (still catching up), disable
    //     damping so the filter can accelerate freely.
    //   - If the target is slow/stationary, apply base damping to prevent drift.
    //   - In TEMP_LOST (no detection), apply stronger damping because the
    //     prediction is all we have and we'd rather it decays toward zero than
    //     runs away.
    if (tracker_->tracker_state == Tracker::TRACKING) {

      // --- Position damping (XYZ) ---
      // Collect current EKF velocity estimate (v_xc, v_yc, v_za).
      Eigen::Vector3d vel(
        tracker_->target_state(1),
        tracker_->target_state(3),
        tracker_->target_state(5));
      double speed = vel.norm();

      // Innovation = measurement - prediction. Its dot product with velocity tells us
      // whether the filter is overshooting:
      //   dot > 0: innovation and velocity agree → filter is still catching up (OK)
      //   dot < 0: innovation opposes velocity   → filter overshot, correct by damping
      double dot = tracker_->info_position_innov.dot(vel);
      bool pos_overshoot = (dot < 0.0) && (speed > 0.3);  // only act if speed is meaningful

      if (pos_overshoot) {
        // Proportionally increase damping with innovation magnitude (capped at 1.0).
        // At full scale (innov >= threshold), alpha drops to xyz_damping_alpha_base_,
        // the strongest damping. Between 0 and threshold, alpha interpolates linearly.
        double pos_scale = std::min(tracker_->info_position_diff / damping_innov_threshold_, 1.0);
        xyz_damping_alpha_ = 1.0 - pos_scale * (1.0 - xyz_damping_alpha_base_);
      } else if (speed > 0.3) {
        // Target is moving and filter is not overshooting: let it track freely.
        xyz_damping_alpha_ = 1.0;
      } else {
        // Target is slow or stationary: apply base damping to prevent velocity drift.
        xyz_damping_alpha_ = xyz_damping_alpha_base_;
      }

      // --- Yaw damping (decoupled from XYZ) ---
      // Same overshoot logic applied independently to the spin axis.
      // Signed innovation: positive means measured yaw > predicted yaw.
      double v_yaw = tracker_->target_state(7);
      double yaw_innov = tracker_->info_yaw_innov_signed;

      // Overshoot: yaw_innov and v_yaw have opposite signs → filter spun past the target.
      bool overshoot = (yaw_innov * v_yaw < 0) && (std::abs(v_yaw) > 0.5);

      if (overshoot) {
        // Scale damping with innovation magnitude.
        // Floor is base * coast_factor so yaw never damps harder than TEMP_LOST coasting.
        double yaw_scale = std::min(tracker_->info_yaw_diff / yaw_innov_threshold_, 1.0);
        double min_alpha = yaw_damping_alpha_base_ * coast_damping_factor_;
        yaw_damping_alpha_ = 1.0 - yaw_scale * (1.0 - min_alpha);
      } else if (std::abs(v_yaw) > 0.5) {
        // Steady spin or accelerating: no damping, let EKF track freely.
        yaw_damping_alpha_ = 1.0;
      } else {
        // Low/no spin: apply base damping to prevent yaw velocity drift.
        yaw_damping_alpha_ = yaw_damping_alpha_base_;
      }

    } else if (tracker_->tracker_state == Tracker::TEMP_LOST) {
      // No detection this frame: the EKF coasts using its last velocity estimate.
      // Apply stronger damping (base * coast_factor) so velocities decay faster
      // during blind frames, reducing prediction error when detection resumes.
      xyz_damping_alpha_ = xyz_damping_alpha_base_ * coast_damping_factor_;
      yaw_damping_alpha_ = yaw_damping_alpha_base_ * coast_damping_factor_;
    } else {
      // DETECTING state: tracker is accumulating initial frames, use base damping.
      xyz_damping_alpha_ = xyz_damping_alpha_base_;
      yaw_damping_alpha_ = yaw_damping_alpha_base_;
    }

    tracker_->update(armors_ptr);

    // Publish info
    info_msg.position_diff    = tracker_->info_position_diff;
    info_msg.yaw_diff         = tracker_->info_yaw_diff;
    info_msg.face_angle       = tracker_->info_face_angle;
    if (tracker_->measurement_valid) {
      info_msg.position.x     = tracker_->measurement(0);
      info_msg.position.y     = tracker_->measurement(1);
      info_msg.position.z     = tracker_->measurement(2);
      info_msg.yaw            = tracker_->measurement(3);
      last_measurement_time_  = time;
    }
    info_msg.xyz_damping_alpha = xyz_damping_alpha_;
    info_msg.yaw_damping_alpha = yaw_damping_alpha_;
    info_pub_->publish(info_msg);

    target_msg.tracker_state = static_cast<uint8_t>(tracker_->tracker_state);

    if (tracker_->tracker_state == Tracker::DETECTING) {
      target_msg.tracking = false;
    } else if (
      tracker_->tracker_state == Tracker::TRACKING ||
      tracker_->tracker_state == Tracker::TEMP_LOST) {
      target_msg.tracking = true;
      const auto & state = tracker_->target_state;
      target_msg.id         = tracker_->tracked_id;
      target_msg.armors_num = static_cast<int>(tracker_->tracked_armors_num);
      target_msg.position.x = state(0);
      target_msg.velocity.x = state(1);
      target_msg.position.y = state(2);
      target_msg.velocity.y = state(3);
      target_msg.position.z = state(4);
      target_msg.velocity.z = state(5);
      target_msg.yaw        = state(6);
      target_msg.v_yaw      = state(7);
      target_msg.radius_1   = state(8);
      RCLCPP_DEBUG(this->get_logger(), "target_msg.position: %f %f %f %f %f %f %f %f %f",
      target_msg.position.x, target_msg.position.y, target_msg.position.z,
      target_msg.velocity.x, target_msg.velocity.y, target_msg.velocity.z,
      target_msg.yaw, target_msg.v_yaw, target_msg.radius_1);
      target_msg.radius_2   = tracker_->another_r;
      target_msg.dz         = tracker_->dz;
      target_msg.v_yaw_variance = tracker_->ekf.getVariance(7);
      target_msg.last_measurement_stamp = last_measurement_time_;
    }
  }

  // Publish optimal bbox
  vision_msgs::msg::Detection2D optimal_bbox;
  optimal_bbox.header = detection_msg->header;
  if (target_msg.tracking) {
    size_t matched_idx = static_cast<size_t>(-1);
    double min_dist = 0.2;  // 20cm — generous enough for PnP + TF floating-point drift
    for (size_t i = 0; i < armors_ptr->armors.size(); i++) {
      const auto & armor = armors_ptr->armors[i];
      double dx = armor.pose.position.x - tracker_->tracked_armor.pose.position.x;
      double dy = armor.pose.position.y - tracker_->tracked_armor.pose.position.y;
      double dz = armor.pose.position.z - tracker_->tracked_armor.pose.position.z;
      double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (dist < min_dist) {
        min_dist = dist;
        matched_idx = i;
      }
    }
    if (matched_idx != static_cast<size_t>(-1)) {
      size_t original_detection_idx = detection_indices[matched_idx];
      if (original_detection_idx < detection_msg->detections.size()) {
        optimal_bbox = detection_msg->detections[original_detection_idx];
      }
    }
  }
  optimal_bbox_pub_->publish(optimal_bbox);

  last_time_ = time;

  target_pub_->publish(target_msg);

  publishMarkers(target_msg);
}

void ArmorTrackerNode::publishMarkers(const auto_aim_interfaces::msg::Target & target_msg)
{
  position_marker_.header = target_msg.header;
  linear_v_marker_.header = target_msg.header;
  angular_v_marker_.header = target_msg.header;
  armor_marker_.header = target_msg.header;

  visualization_msgs::msg::MarkerArray marker_array;
  if (target_msg.tracking) {
    double yaw = target_msg.yaw, r1 = target_msg.radius_1, r2 = target_msg.radius_2;
    double xc = target_msg.position.x, yc = target_msg.position.y, za = target_msg.position.z;
    double vx = target_msg.velocity.x, vy = target_msg.velocity.y, vz = target_msg.velocity.z;
    double dz = target_msg.dz;

    position_marker_.action = visualization_msgs::msg::Marker::ADD;
    position_marker_.pose.position.x = xc;
    position_marker_.pose.position.y = yc;
    position_marker_.pose.position.z = za + dz / 2;

    linear_v_marker_.action = visualization_msgs::msg::Marker::ADD;
    linear_v_marker_.points.clear();
    linear_v_marker_.points.emplace_back(position_marker_.pose.position);
    geometry_msgs::msg::Point arrow_end = position_marker_.pose.position;
    arrow_end.x += vx;
    arrow_end.y += vy;
    arrow_end.z += vz;
    linear_v_marker_.points.emplace_back(arrow_end);

    angular_v_marker_.action = visualization_msgs::msg::Marker::ADD;
    angular_v_marker_.points.clear();
    angular_v_marker_.points.emplace_back(position_marker_.pose.position);
    arrow_end = position_marker_.pose.position;
    arrow_end.z += target_msg.v_yaw / M_PI;
    angular_v_marker_.points.emplace_back(arrow_end);

    armor_marker_.action = visualization_msgs::msg::Marker::ADD;
    bool is_small = (tracker_->tracked_armor.type == "small");
    armor_marker_.scale.y = is_small ? 0.140 : 0.235;
    armor_marker_.scale.z = is_small ? 0.125 : 0.127;
    bool is_current_pair = true;
    size_t a_n = target_msg.armors_num;
    geometry_msgs::msg::Point p_a;
    double r = 0;
    for (size_t i = 0; i < a_n; i++) {
      double tmp_yaw = yaw + i * (2 * M_PI / a_n);
      // Only 4 armors has 2 radius and height
      if (a_n == 4) {
        r = is_current_pair ? r1 : r2;
        p_a.z = za + (is_current_pair ? 0 : dz);
        is_current_pair = !is_current_pair;
      } else {
        r = r1;
        p_a.z = za;
      }
      p_a.x = xc - r * cos(tmp_yaw);
      p_a.y = yc - r * sin(tmp_yaw);

      armor_marker_.id = i;
      armor_marker_.pose.position = p_a;
      tf2::Quaternion q;
      q.setRPY(0, target_msg.id == "outpost" ? -0.26 : 0.26, tmp_yaw);
      armor_marker_.pose.orientation = tf2::toMsg(q);
      marker_array.markers.emplace_back(armor_marker_);
    }
  } else {
    position_marker_.action = visualization_msgs::msg::Marker::DELETE;
    linear_v_marker_.action = visualization_msgs::msg::Marker::DELETE;
    angular_v_marker_.action = visualization_msgs::msg::Marker::DELETE;

    armor_marker_.action = visualization_msgs::msg::Marker::DELETE;
    for (int i = 0; i < 4; i++) {
      armor_marker_.id = i;
      marker_array.markers.emplace_back(armor_marker_);
    }
  }

  marker_array.markers.emplace_back(position_marker_);
  marker_array.markers.emplace_back(linear_v_marker_);
  marker_array.markers.emplace_back(angular_v_marker_);
  marker_pub_->publish(marker_array);
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorTrackerNode)
