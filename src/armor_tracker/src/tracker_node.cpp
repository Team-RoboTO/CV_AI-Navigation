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

  // Ratio to scale YOLO bbox inward to approximate light-bar positions
  light_ratio_ = this->declare_parameter("light_ratio", 0.85);

  // Tracker
  double max_match_distance = this->declare_parameter("tracker.max_match_distance", 0.15);
  tracker_ = std::make_unique<Tracker>(max_match_distance);
  tracker_->tracking_thres = this->declare_parameter("tracker.tracking_thres", 5);
  lost_time_thres_ = this->declare_parameter("tracker.lost_time_thres", 0.3);

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
  // --- f: EKF Process Function ---
  // Predicts the next state from the current state using a damped constant-velocity model.
  // Each axis follows: pos += v*dt,  v_new = v * alpha^(dt/T_ref)
  // where T_ref=1/30s normalizes the decay so alpha is "per-frame at 30Hz" regardless of dt.
  // r (radius) is constant — it only changes on armor-jump events handled in Tracker::update().
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
  // h - Observation function
  auto h = [](const Eigen::VectorXd & x) {
    Eigen::VectorXd z(4);
    double xc = x(0), yc = x(2), yaw = x(6), r = x(8);
    z(0) = xc - r * cos(yaw);  // xa
    z(1) = yc - r * sin(yaw);  // ya
    z(2) = x(4);               // za
    z(3) = x(6);               // yaw
    return z;
  };
  // J_h - Jacobian of observation function
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
  // update_Q - process noise covariance matrix
  s2qxyz_ = declare_parameter("ekf.sigma2_q_xyz", 5.0);
  s2qyaw_ = declare_parameter("ekf.sigma2_q_yaw", 10.0);
  s2qr_   = declare_parameter("ekf.sigma2_q_r",   2.0);
  auto u_q = [this]() {
    Eigen::MatrixXd q(9, 9);
    double t = dt_, x = s2qxyz_, y = s2qyaw_, r = s2qr_;
    double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
    double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * y, q_vy_vy = pow(t, 2) * y;
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
  // update_R - distance-dependent measurement noise covariance matrix
  r_xyz_base_  = declare_parameter("ekf.r_xyz_base",  0.005);
  r_xyz_slope_ = declare_parameter("ekf.r_xyz_slope", 0.03);
  r_yaw_base_  = declare_parameter("ekf.r_yaw_base",  0.015);
  r_yaw_slope_ = declare_parameter("ekf.r_yaw_slope", 0.002);
  auto u_r = [this](const Eigen::VectorXd & z) {
    Eigen::DiagonalMatrix<double, 4> r;
    double dist = std::sqrt(z[0]*z[0] + z[1]*z[1] + z[2]*z[2]);
    double ps = r_xyz_base_ + r_xyz_slope_ * dist;
    double ys = r_yaw_base_ + r_yaw_slope_ * dist;

    // Angle-aware noise: PnP accuracy degrades as ~1/cos²(oblique_angle)
    // because projected width shrinks as cos(angle), amplifying pixel errors.
    // z = [xa, ya, za, yaw]. Bearing from armor→camera is opposite of camera→armor.
    double bearing_opp = std::atan2(-z[1], -z[0]);
    double face_angle = std::abs(std::remainder(z[3] - bearing_opp, 2.0 * M_PI));
    double cos_fa = std::cos(face_angle);
    // Clamp at cos²=0.04 (≈78°) to cap inflation at 25x
    double angle_factor = 1.0 / std::max(cos_fa * cos_fa, 0.04);

    r.diagonal() << ps*ps*angle_factor, ps*ps*angle_factor,
                     ps*ps*angle_factor, ys*ys*angle_factor;
    return r;
  };
  // P - initial error covariance
  Eigen::DiagonalMatrix<double, 9> p0;
  //       xc    v_xc  yc    v_yc  za    v_za  yaw   v_yaw  r
  p0.diagonal() << 0.1, 1.0, 0.1, 1.0, 0.1, 0.2, 0.1, 3.0, 0.003;
  tracker_->ekf = ExtendedKalmanFilter{f, h, j_f, j_h, u_q, u_r, p0};

  // Covariance upper bounds — safety net against P explosion during TEMP_LOST
  Eigen::VectorXd max_cov(9);
  //          xc    v_xc   yc    v_yc   za    v_za   yaw    v_yaw   r
  max_cov << 1.0,  10.0,  1.0,  10.0,  1.0,  2.0,   1.0,   30.0,  0.03;
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
}

void ArmorTrackerNode::microPoseCallback(
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
  // Extract gimbal angles from the lower computer.
  // cmd_vel_subscriber stores: position.x = -pitch, position.y = -yaw (radians)
  gimbal_yaw_   = yaw_sign_ * msg->pose.position.y;
  gimbal_pitch_ = pitch_sign_ * msg->pose.position.x;
  broadcastGimbalTF(msg->header.stamp);
}

void ArmorTrackerNode::broadcastGimbalTF(const rclcpp::Time & stamp)
{
  // T(odom → camera) = Translate(0,0,height) × R_z(yaw) × R_y(pitch) × R_convention
  // R_convention = RPY(-π/2, 0, -π/2) matches the old static TF when yaw=pitch=0
  tf2::Quaternion q_yaw, q_pitch, q_convention;
  q_yaw.setRPY(0, 0, gimbal_yaw_);
  q_pitch.setRPY(0, gimbal_pitch_, 0);
  q_convention.setRPY(-M_PI / 2.0, 0, -M_PI / 2.0);

  tf2::Quaternion q_final = q_yaw * q_pitch * q_convention;
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
    // Extract bbox (decoder outputs coordinates in real image space)
    auto center_x = detection.bbox.center.position.x;
    auto center_y = detection.bbox.center.position.y;
    auto width = detection.bbox.size_x;
    auto height = detection.bbox.size_y;

    // Skip degenerate detections (avoids divide-by-zero below)
    if (height < 1.0 || width < 1.0) {
      detection_idx++;
      continue;
    }

    // Create a fake Armor object for PnP
    // We assume the bbox covers the armor lights
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

  // Transform armor positions from image frame to world coordinate
  // Look up the transform once and apply it to all armors (avoids N redundant buffer lookups)
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

  // Filter abnormal armors (keep detection_indices in sync)
  auto & armors_vec = armors_ptr->armors;

  // Log yaw for all detections (debug)
  for (size_t i = 0; i < armors_vec.size(); i++) {
    const auto & a = armors_vec[i];
    tf2::Quaternion q;
    tf2::fromMsg(a.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    RCLCPP_INFO(get_logger(),
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
    dt_ = std::min(std::max((time - last_time_).seconds(), 0.01), 0.10);
    tracker_->lost_thres = static_cast<int>(lost_time_thres_ / dt_);

    // --- Adaptive Velocity Damping ---
    // The EKF process model applies per-step velocity decay: v_new = alpha^(dt/T) * v_old.
    // alpha=1.0 means no decay (track freely); alpha<1.0 attenuates velocity each step.
    // We choose alpha dynamically each frame based on tracker state and innovation signal
    // to balance responsiveness (track real motion) vs. stability (damp filter runaway).
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
    info_msg.position.x       = tracker_->measurement(0);
    info_msg.position.y       = tracker_->measurement(1);
    info_msg.position.z       = tracker_->measurement(2);
    info_msg.yaw              = tracker_->measurement(3);
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
      target_msg.radius_2   = tracker_->another_r;
      target_msg.dz         = tracker_->dz;
      target_msg.v_yaw_variance = tracker_->ekf.getVariance(7);
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
