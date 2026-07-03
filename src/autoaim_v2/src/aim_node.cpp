// =============================================================================
// aim_node — single-process 1v1 auto-aim (see DESIGN.md).
//
// Hot path (vision thread): camera -> TRT pose -> PnP + yaw search -> EKF ->
// aimer/fire -> serial TX. Never touches DDS. ROS is used only for parameters
// and optional throttled debug topics.
//
// Serial protocol, sign conventions and parameter names are kept compatible
// with the old autoaim/serial_bridge pipeline so existing tuning carries over.
// =============================================================================

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <opencv2/imgproc.hpp>
#include <autoaim/msg/aim_debug_armor.hpp>
#include <autoaim/msg/aim_debug_frame.hpp>
#include <autoaim/msg/armor_keypoint_array.hpp>

#include "autoaim_v2/aimer.hpp"
#include "autoaim_v2/gimbal_buffer.hpp"
#include "autoaim_v2/protocol.hpp"
#include "autoaim_v2/serial_port.hpp"
#include "autoaim_v2/solver.hpp"
#include "autoaim_v2/tracker.hpp"
#include "autoaim_v2/types.hpp"

#if AIM_WITH_ZED_TRT || AIM_WITH_RS_TRT
#include <cuda_runtime_api.h>
#include "autoaim_v2/detector_trt.hpp"
#endif

#if AIM_WITH_ZED_TRT
#include "autoaim_v2/zed_camera.hpp"
#endif

#if AIM_WITH_RS_TRT
#include "autoaim_v2/rs_camera.hpp"
#endif

namespace aim
{

using std::placeholders::_1;

class AimNode : public rclcpp::Node
{
public:
  AimNode() : rclcpp::Node("aim_v2"), t_start_(Clock::now())
  {
    declare_params();
    load_params();

    serial_thread_ = std::thread([this] { serial_rx_loop(); });
    heartbeat_thread_ = std::thread([this] { heartbeat_loop(); });

    if (get_parameter("sim_gimbal").as_bool()) {
      RCLCPP_WARN(get_logger(),
                  "sim_gimbal ON — zero gimbal angles injected (BENCH ONLY)");
      sim_gimbal_thread_ = std::thread([this] {
        while (running_) {
          GimbalSample s;
          s.t = Clock::now();
          gimbal_buf_.push(s);
          std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
      });
    }

    if (debug_enable_) {
      dbg_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
        "/aimv2/debug", rclcpp::SensorDataQoS());
      if (debug_full_) {
        dbg_frame_pub_ = create_publisher<autoaim::msg::AimDebugFrame>(
          "/aimv2/debug_frame", rclcpp::SensorDataQoS());
      }
      if (debug_markers_) {
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
          "/aimv2/markers", 10);
      }
    }
    if (debug_image_enable_) {
      debug_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
        debug_image_topic_, rclcpp::SensorDataQoS());
    }

    if (input_mode_ == "ros_topics") {
      caminfo_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera_info", rclcpp::QoS(1).reliable().transient_local(),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {
          if (solver_ready_) return;
          solver_->set_intrinsics(m->k[0], m->k[4], m->k[2], m->k[5],
                                  std::vector<double>(m->d.begin(), m->d.end()));
          camera_width_ = static_cast<int>(m->width);
          camera_height_ = static_cast<int>(m->height);
          camera_fx_ = m->k[0];
          camera_fy_ = m->k[4];
          camera_cx_ = m->k[2];
          camera_cy_ = m->k[5];
          solver_ready_ = true;
        });
      kpt_sub_ = create_subscription<autoaim::msg::ArmorKeypointArray>(
        "/detector/armors_keypoints", rclcpp::SensorDataQoS(),
        std::bind(&AimNode::kpt_callback, this, _1));
      RCLCPP_INFO(get_logger(), "aim_v2 in ros_topics mode (A/B behind old detector)");
    } else if (input_mode_ == "rs_trt") {
#if AIM_WITH_RS_TRT
      vision_thread_ = std::thread([this] { rs_loop(); });
      RCLCPP_INFO(get_logger(), "aim_v2 in rs_trt mode (RealSense + TensorRT)");
#else
      RCLCPP_FATAL(get_logger(),
                   "built without RealSense/TensorRT — set input_mode:=ros_topics");
      rclcpp::shutdown();
#endif
    } else {
#if AIM_WITH_ZED_TRT
      vision_thread_ = std::thread([this] { zed_loop(); });
      RCLCPP_INFO(get_logger(), "aim_v2 in zed_trt mode");
#else
      RCLCPP_FATAL(get_logger(),
                   "built without ZED/TensorRT — set input_mode:=ros_topics");
      rclcpp::shutdown();
#endif
    }
  }

  ~AimNode() override
  {
    running_ = false;
    if (vision_thread_.joinable()) vision_thread_.join();
    if (serial_thread_.joinable()) serial_thread_.join();
    if (heartbeat_thread_.joinable()) heartbeat_thread_.join();
    if (sim_gimbal_thread_.joinable()) sim_gimbal_thread_.join();
  }

private:
  // ────────────────────────────── parameters ──────────────────────────────
  void declare_params()
  {
    // Input
    declare_parameter("input_mode", std::string("zed_trt"));  // zed_trt | rs_trt | ros_topics
    declare_parameter("engine_path", std::string(
      "/workspaces/isaac_ros-dev/AI-models/yolov26_keypoints.engine"));
    declare_parameter("conf_threshold", 0.15);
    declare_parameter("nms_iou", 0.2);
    // Camera
    declare_parameter("resolution", std::string("SVGA"));
    declare_parameter("fps", 120);
    declare_parameter("image_flip", true);
    declare_parameter("exposure_time_us", 3000);
    declare_parameter("gain", 80);
    declare_parameter("auto_white_balance", true);
    // RealSense camera
    declare_parameter("rs_serial_no", std::string(""));
    declare_parameter("rs_width", 640);
    declare_parameter("rs_height", 360);
    declare_parameter("rs_fps", 90);
    declare_parameter("rs_auto_exposure", false);
    declare_parameter("rs_exposure", 60);
    declare_parameter("rs_gain", 50);
    declare_parameter("rs_pixel_format", std::string("yuyv"));
    declare_parameter("rs_flip_180", true);
    // Serial
    declare_parameter("serial_port", std::string("/dev/ttyACM0"));
    declare_parameter("serial_baudrate", 500000);
    declare_parameter("serial_parity", std::string("even"));
    declare_parameter("shooting_active", false);   // master fire switch
    declare_parameter("rotating_chassis", false);
    declare_parameter("send_gimbal_feedforward", false);
    declare_parameter("cmd_timeout", 0.3);
    declare_parameter("heartbeat_hz", 100.0);
    // Micro conventions (same names/semantics as the old pipeline)
    declare_parameter("gimbal.yaw_sign", 1.0);
    declare_parameter("gimbal.pitch_sign", -1.0);
    declare_parameter("micro_pitch_feedback_opposite_sign", true);
    declare_parameter("micro_pitch_lock_opposite_sign", false);
    declare_parameter("target_color_status_index", 4);
    declare_parameter("bullet_speed_index", -1);
    declare_parameter("fixed_target_class", -1);  // -1 auto from micro; 0 blue; 2 red
    // Geometry
    declare_parameter("gimbal_height", 0.42);
    declare_parameter("cam_offset_x", 0.0);
    declare_parameter("cam_offset_y", 0.0);
    declare_parameter("cam_offset_z", 0.0);
    declare_parameter("cam_trim_yaw_deg", 0.0);
    declare_parameter("cam_trim_pitch_deg", 0.0);
    declare_parameter("cam_trim_roll_deg", 0.0);
    declare_parameter("assume_small_armor", true);
    declare_parameter("armor_pitch_deg", 15.0);
    declare_parameter("max_reproj_error", 25.0);
    declare_parameter("min_confidence", 0.30);
    declare_parameter("max_armor_dist", 8.0);
    // Tracker
    declare_parameter("q_accel_xy", 60.0);
    declare_parameter("q_accel_z", 4.0);
    declare_parameter("q_alpha", 300.0);
    declare_parameter("r_bearing", 0.004);
    declare_parameter("r_dist_base", 0.03);
    declare_parameter("r_dist_slope", 0.05);
    declare_parameter("maha_gate", 16.0);
    declare_parameter("confirm_frames", 2);
    declare_parameter("lost_timeout", 0.6);
    declare_parameter("initial_radius", 0.24);
    // Ballistics / aimer
    declare_parameter("bullet_speed", 24.0);
    declare_parameter("drag_k", 0.0196);
    declare_parameter("actuation_latency", 0.025);
    declare_parameter("feeder_delay", 0.035);
    declare_parameter("feeder_sigma", 0.006);
    declare_parameter("disp_bullet_mrad", 3.0);
    declare_parameter("disp_system_mrad", 4.0);
    declare_parameter("omega_track", 2.0);
    declare_parameter("omega_timed", 12.0);
    declare_parameter("p_spray_min", 0.50);
    declare_parameter("p_timed_min", 0.35);
    declare_parameter("coming_angle_deg", 55.0);
    declare_parameter("leaving_angle_deg", 20.0);
    declare_parameter("min_fire_dist", 0.3);
    declare_parameter("max_fire_dist", 6.5);
    declare_parameter("shot_pulse", 0.020);
    declare_parameter("shot_min_interval", 0.080);
    declare_parameter("barrel_offset_x", 0.0);
    declare_parameter("barrel_offset_y", 0.0);
    declare_parameter("barrel_offset_z", -0.05);
    declare_parameter("yaw_offset_deg", 0.0);
    declare_parameter("pitch_offset_deg", 0.0);
    declare_parameter("fire_lock_yaw", 0.05);
    declare_parameter("fire_lock_pitch", 0.04);
    // Command shaping / safety
    declare_parameter("cmd_deadband_yaw", 0.005);
    declare_parameter("cmd_deadband_pitch", 0.008);
    declare_parameter("cmd_max_delta_yaw", 0.80);
    declare_parameter("cmd_max_delta_pitch", 0.60);
    declare_parameter("cmd_hold_time", 0.25);
    // Ego motion (same semantics as old pipeline)
    declare_parameter("use_ego_motion_compensation", true);
    declare_parameter("ego_velocity_available", false);
    declare_parameter("ego_velocity_body_frame", true);
    declare_parameter("ego_velocity_deadband", 0.05);
    declare_parameter("ego_velocity_lpf_alpha", 0.6);
    declare_parameter("ego_velocity_max", 3.5);
    declare_parameter("chassis_heading_index", -1);
    // Debug
    declare_parameter("debug_enable", false);
    declare_parameter("debug_full", true);
    declare_parameter("debug_markers", false);
    declare_parameter("debug_every", 4);
    declare_parameter("debug_image_enable", false);
    declare_parameter("debug_image_every", 4);
    declare_parameter("debug_image_topic", std::string("/aimv2/debug_image"));
    // Bench-only: feed zero gimbal angles at 500 Hz so the pipeline runs
    // without the micro attached. NEVER enable on the robot — a live match
    // needs the real angles for the camera->world transform.
    declare_parameter("sim_gimbal", false);
  }

  void load_params()
  {
    input_mode_ = get_parameter("input_mode").as_string();

    SolverParams sp;
    sp.assume_small_armor = get_parameter("assume_small_armor").as_bool();
    sp.armor_pitch_deg = get_parameter("armor_pitch_deg").as_double();
    sp.max_reproj_err_px = get_parameter("max_reproj_error").as_double();
    sp.t_cam2gimbal = Eigen::Vector3d(get_parameter("cam_offset_x").as_double(),
                                      get_parameter("cam_offset_y").as_double(),
                                      get_parameter("cam_offset_z").as_double());
    sp.trim_yaw_deg = get_parameter("cam_trim_yaw_deg").as_double();
    sp.trim_pitch_deg = get_parameter("cam_trim_pitch_deg").as_double();
    sp.trim_roll_deg = get_parameter("cam_trim_roll_deg").as_double();
    sp.gimbal_height = get_parameter("gimbal_height").as_double();
    solver_ = std::make_unique<Solver>(sp);

    TrackerParams tp;
    tp.q_accel_xy = get_parameter("q_accel_xy").as_double();
    tp.q_accel_z = get_parameter("q_accel_z").as_double();
    tp.q_alpha = get_parameter("q_alpha").as_double();
    tp.r_bearing = get_parameter("r_bearing").as_double();
    tp.r_dist_base = get_parameter("r_dist_base").as_double();
    tp.r_dist_slope = get_parameter("r_dist_slope").as_double();
    tp.maha_gate = get_parameter("maha_gate").as_double();
    tp.confirm_frames = static_cast<int>(get_parameter("confirm_frames").as_int());
    tp.lost_timeout = get_parameter("lost_timeout").as_double();
    tp.initial_r = get_parameter("initial_radius").as_double();
    tp.gimbal_height = sp.gimbal_height;
    tracker_ = std::make_unique<Tracker>(tp);

    AimerParams ap;
    ap.ballistics.bullet_speed = get_parameter("bullet_speed").as_double();
    ap.ballistics.drag_k = get_parameter("drag_k").as_double();
    ap.actuation_latency = get_parameter("actuation_latency").as_double();
    ap.feeder_delay = get_parameter("feeder_delay").as_double();
    ap.feeder_sigma = get_parameter("feeder_sigma").as_double();
    ap.disp_bullet_mrad = get_parameter("disp_bullet_mrad").as_double();
    ap.disp_system_mrad = get_parameter("disp_system_mrad").as_double();
    ap.armor_pitch_deg = sp.armor_pitch_deg;
    ap.omega_track = get_parameter("omega_track").as_double();
    ap.omega_timed = get_parameter("omega_timed").as_double();
    ap.p_spray_min = get_parameter("p_spray_min").as_double();
    ap.p_timed_min = get_parameter("p_timed_min").as_double();
    ap.coming_angle_deg = get_parameter("coming_angle_deg").as_double();
    ap.leaving_angle_deg = get_parameter("leaving_angle_deg").as_double();
    ap.min_fire_dist = get_parameter("min_fire_dist").as_double();
    ap.max_fire_dist = get_parameter("max_fire_dist").as_double();
    ap.shot_pulse = get_parameter("shot_pulse").as_double();
    ap.shot_min_interval = get_parameter("shot_min_interval").as_double();
    ap.barrel_x = get_parameter("barrel_offset_x").as_double();
    ap.barrel_y = get_parameter("barrel_offset_y").as_double();
    ap.barrel_z = get_parameter("barrel_offset_z").as_double();
    ap.gimbal_height = sp.gimbal_height;
    ap.yaw_offset_deg = get_parameter("yaw_offset_deg").as_double();
    ap.pitch_offset_deg = get_parameter("pitch_offset_deg").as_double();
    ap.fire_lock_yaw = get_parameter("fire_lock_yaw").as_double();
    ap.fire_lock_pitch = get_parameter("fire_lock_pitch").as_double();
    ap.yaw_sign = get_parameter("gimbal.yaw_sign").as_double();
    ap.pitch_sign = get_parameter("gimbal.pitch_sign").as_double();
    ap.micro_pitch_lock_opposite_sign =
      get_parameter("micro_pitch_lock_opposite_sign").as_bool();
    aimer_ = std::make_unique<Aimer>(ap);

    yaw_sign_ = ap.yaw_sign;
    pitch_sign_ = ap.pitch_sign;
    pitch_feedback_opposite_ = get_parameter("micro_pitch_feedback_opposite_sign").as_bool();

    serial_device_ = get_parameter("serial_port").as_string();
    serial_baud_ = static_cast<int>(get_parameter("serial_baudrate").as_int());
    serial_parity_ = get_parameter("serial_parity").as_string();
    shooting_active_ = get_parameter("shooting_active").as_bool();
    rotating_chassis_ = get_parameter("rotating_chassis").as_bool();
    send_ff_ = get_parameter("send_gimbal_feedforward").as_bool();
    cmd_timeout_ = get_parameter("cmd_timeout").as_double();
    heartbeat_hz_ = get_parameter("heartbeat_hz").as_double();

    color_index_ = static_cast<int>(get_parameter("target_color_status_index").as_int());
    bullet_speed_index_ = static_cast<int>(get_parameter("bullet_speed_index").as_int());
    fixed_target_class_ = static_cast<int>(get_parameter("fixed_target_class").as_int());
    min_confidence_ = get_parameter("min_confidence").as_double();
    max_armor_dist_ = get_parameter("max_armor_dist").as_double();

    cmd_deadband_yaw_ = get_parameter("cmd_deadband_yaw").as_double();
    cmd_deadband_pitch_ = get_parameter("cmd_deadband_pitch").as_double();
    cmd_max_delta_yaw_ = get_parameter("cmd_max_delta_yaw").as_double();
    cmd_max_delta_pitch_ = get_parameter("cmd_max_delta_pitch").as_double();
    cmd_hold_time_ = get_parameter("cmd_hold_time").as_double();

    ego_enable_ = get_parameter("use_ego_motion_compensation").as_bool() &&
                  get_parameter("ego_velocity_available").as_bool();
    ego_body_frame_ = get_parameter("ego_velocity_body_frame").as_bool();
    ego_deadband_ = get_parameter("ego_velocity_deadband").as_double();
    ego_lpf_alpha_ = get_parameter("ego_velocity_lpf_alpha").as_double();
    ego_vmax_ = get_parameter("ego_velocity_max").as_double();
    chassis_heading_index_ = static_cast<int>(get_parameter("chassis_heading_index").as_int());

    debug_enable_ = get_parameter("debug_enable").as_bool();
    debug_full_ = get_parameter("debug_full").as_bool();
    debug_markers_ = get_parameter("debug_markers").as_bool();
    debug_every_ = std::max(1, static_cast<int>(get_parameter("debug_every").as_int()));
    debug_image_enable_ = get_parameter("debug_image_enable").as_bool();
    debug_image_every_ =
      std::max(1, static_cast<int>(get_parameter("debug_image_every").as_int()));
    debug_image_topic_ = get_parameter("debug_image_topic").as_string();
  }

  // ────────────────────────────── serial RX ───────────────────────────────
  void serial_rx_loop()
  {
    std::vector<uint8_t> buf;
    buf.reserve(4096);
    uint8_t chunk[512];

    while (running_) {
      if (!port_.is_open()) {
        if (!port_.open(serial_device_, serial_baud_, serial_parity_, true)) {
          std::this_thread::sleep_for(std::chrono::seconds(2));
          continue;
        }
        buf.clear();
      }

      int n = port_.read_bytes(chunk, sizeof(chunk), 20);
      if (n < 0) {
        port_.close();
        continue;
      }
      if (n == 0) continue;
      buf.insert(buf.end(), chunk, chunk + n);

      // Keep only the tail if we somehow fall behind.
      if (buf.size() > 40 * 64) {
        buf.erase(buf.begin(), buf.end() - protocol::RX_BYTES);
      }

      size_t off = 0;
      while (buf.size() - off >= protocol::RX_BYTES) {
        float v[protocol::RX_FLOATS];
        if (protocol::parse_rx(buf.data() + off, v)) {
          handle_rx(v);
          off += protocol::RX_BYTES;
        } else {
          off += 1;  // byte-shifted: resync
        }
      }
      buf.erase(buf.begin(), buf.begin() + off);
    }
    port_.close();
  }

  void handle_rx(const float * v)
  {
    const TimePoint now = Clock::now();

    GimbalSample s;
    s.t = now;
    s.yaw_raw = v[0];
    s.pitch_raw = v[1];
    const double pitch_internal_raw = pitch_feedback_opposite_ ? -v[1] : v[1];
    s.yaw = yaw_sign_ * v[0];
    s.pitch = pitch_sign_ * pitch_internal_raw;
    gimbal_buf_.push(s);

    {
      std::lock_guard<std::mutex> lk(status_mtx_);
      status_.vx = v[2];
      status_.vy = v[3];
      const int ci = std::clamp(color_index_, 0, 9);
      status_.color = static_cast<int>(std::lround(v[ci]));
      status_.game_progress = static_cast<int>(std::lround(v[5]));
      status_.hp = v[6];
      status_.reserved9 = v[9];
      status_.valid = true;

      if (bullet_speed_index_ >= 0 && bullet_speed_index_ < 10) {
        const double bs = v[bullet_speed_index_];
        // Handed to the vision thread via atomic — aimer_ is not thread-safe.
        if (bs > 10.0 && bs < 35.0) bullet_speed_rx_.store(bs);
      }
    }

    update_ego(v, now, s.yaw);
  }

  void update_ego(const float * v, TimePoint now, double gimbal_yaw)
  {
    std::lock_guard<std::mutex> lk(ego_mtx_);
    if (!ego_enable_) {
      ego_vel_.setZero();
      return;
    }
    if (ego_last_t_.time_since_epoch().count() == 0) {
      ego_last_t_ = now;
      return;
    }
    double dt = std::clamp(seconds(now, ego_last_t_), 0.0, 0.05);
    ego_last_t_ = now;
    if (dt <= 0) return;

    double vx = v[2], vy = v[3];
    double vn = std::hypot(vx, vy);
    if (vn > ego_vmax_ && vn > 1e-6) {
      vx *= ego_vmax_ / vn;
      vy *= ego_vmax_ / vn;
      vn = ego_vmax_;
    }
    if (ego_deadband_ > 0) {
      if (vn < ego_deadband_) {
        vx = vy = 0;
      } else if (vn > 1e-6) {
        const double keep = (vn - ego_deadband_) / vn;
        vx *= keep;
        vy *= keep;
      }
    }
    const double a = std::clamp(ego_lpf_alpha_, 0.0, 1.0);
    ego_vf_.x() = a * vx + (1 - a) * ego_vf_.x();
    ego_vf_.y() = a * vy + (1 - a) * ego_vf_.y();

    double heading = gimbal_yaw;  // fallback: head frame
    if (chassis_heading_index_ >= 0 && chassis_heading_index_ < 10) {
      heading = v[chassis_heading_index_];
    }
    Eigen::Vector2d vw = ego_vf_;
    if (ego_body_frame_) {
      const double c = std::cos(heading), s = std::sin(heading);
      vw = Eigen::Vector2d(c * ego_vf_.x() - s * ego_vf_.y(),
                           s * ego_vf_.x() + c * ego_vf_.y());
    }
    ego_pos_ += vw * dt;
    ego_vel_ = vw;
  }

  // ────────────────────────────── serial TX ───────────────────────────────
  void send_command(const GimbalCommand & cmd, bool fresh)
  {
    protocol::TxPacket p;
    {
      std::lock_guard<std::mutex> lk(tx_mtx_);
      if (fresh) {
        last_cmd_ = cmd;
        last_cmd_t_ = Clock::now();
      }
      const bool stale =
        seconds(Clock::now(), last_cmd_t_) > cmd_timeout_ || !last_cmd_.valid;

      p.t = static_cast<float>(seconds(Clock::now(), t_start_));
      if (last_cmd_.valid) {
        p.yaw = static_cast<float>(last_cmd_.yaw_micro);
        p.pitch = static_cast<float>(last_cmd_.pitch_micro);
      } else {
        const GimbalSample g = gimbal_buf_.latest();
        p.yaw = static_cast<float>(g.yaw_raw);
        p.pitch = static_cast<float>(g.pitch_raw);
      }
      // A shoot flag is only ever sent on a FRESH command from the aim loop.
      // Keepalive re-sends force it to zero: firing is a per-frame decision,
      // never a latched state (a stalled vision thread must stop the feeder).
      p.shoot = (shooting_active_ && fresh && !stale && last_cmd_.shoot) ? 1.f : 0.f;
      if (send_ff_) {
        p.nav_x = static_cast<float>(stale ? 0.0 : last_cmd_.ff_yaw_rate);
        p.nav_y = static_cast<float>(stale ? 0.0 : last_cmd_.ff_pitch_rate);
      }
      p.rot = rotating_chassis_ ? 1.f : 0.f;
    }
    uint8_t bytes[protocol::TX_BYTES];
    protocol::pack(p, bytes);
    port_.write_bytes(bytes, sizeof(bytes));
  }

  void heartbeat_loop()
  {
    // Keeps the micro's command watchdog fed between vision frames, kills the
    // shoot flag if the vision thread stalls (camera hang etc.).
    const auto period =
      std::chrono::microseconds(static_cast<int64_t>(1e6 / std::max(heartbeat_hz_, 1.0)));
    while (running_) {
      std::this_thread::sleep_for(period);
      if (!port_.is_open() || !gimbal_buf_.hasData()) continue;
      const double since_tx = seconds(Clock::now(), last_tx_time());
      if (since_tx * heartbeat_hz_ >= 0.9) {
        send_command(GimbalCommand{}, false);
      }
    }
  }

  TimePoint last_tx_time()
  {
    std::lock_guard<std::mutex> lk(tx_mtx_);
    return last_cmd_t_;
  }

  // ────────────────────────────── vision core ─────────────────────────────
  int enemy_class() const
  {
    if (fixed_target_class_ >= 0) return fixed_target_class_;
    std::lock_guard<std::mutex> lk(status_mtx_);
    if (!status_.valid || status_.color < 0) return -1;
    return status_.color == 0 ? 0 : 2;  // we RED -> shoot blue(0); we BLUE -> red(2)
  }

  struct ArmorDebugSample
  {
    ArmorPx px;
    bool accepted = false;
    bool solved = false;
    std::string reject_reason;
    ArmorWorld world;
  };

  // The whole per-frame chain, shared by both input modes.
  void process_frame(const std::vector<ArmorPx> & dets, TimePoint t_capture)
  {
    const int enemy = enemy_class();

    GimbalSample gs_cap;
    if (!gimbal_buf_.query(t_capture, gs_cap)) {
      if (debug_enable_ && debug_full_ && dbg_frame_pub_ &&
          (frame_count_++ % debug_every_ == 0)) {
        publish_debug_frame_no_gimbal(dets, enemy, t_capture);
      }
      return;  // no micro yet
    }

    Eigen::Vector2d ego, ego_vel;
    {
      std::lock_guard<std::mutex> lk(ego_mtx_);
      ego = ego_pos_;
      ego_vel = ego_vel_;
    }

    std::vector<ArmorWorld> armors;
    std::vector<ArmorDebugSample> armor_debug;
    armor_debug.reserve(dets.size());
    for (const auto & d : dets) {
      ArmorDebugSample ad;
      ad.px = d;

      if (enemy < 0) {
        ad.reject_reason = "enemy_class_unknown";
      } else if (d.class_id != enemy) {
        ad.reject_reason = "not_enemy_class";
      } else if (d.confidence < min_confidence_) {
        ad.reject_reason = "below_min_confidence";
      } else {
        auto aw = solver_->solve(d, gs_cap.yaw, gs_cap.pitch, ego);
        if (!aw) {
          ad.reject_reason = "pnp_failed";
        } else {
          ad.solved = true;
          ad.world = *aw;
          if (aw->pos_cam.norm() > max_armor_dist_) {
            ad.reject_reason = "beyond_max_distance";
          } else {
            ad.accepted = true;
            ad.reject_reason = "accepted";
            armors.push_back(*aw);
          }
        }
      }
      armor_debug.push_back(ad);
    }

    const Eigen::Vector3d cam0 =
      solver_->cam_origin_world(gs_cap.yaw, gs_cap.pitch, ego);
    tracker_->update(armors, t_capture, cam0);

    const double bs = bullet_speed_rx_.exchange(0.0);
    if (bs > 0.0) aimer_->set_bullet_speed(bs);

    const TimePoint t_now = Clock::now();
    const GimbalSample gs_now = gimbal_buf_.latest();
    AimDebug dbg;
    GimbalCommand cmd = aimer_->plan(*tracker_, t_now, gs_now, ego, ego_vel, dbg);
    const GimbalCommand planned_cmd = cmd;

    shape_and_send(cmd, gs_now, t_now);

    if (debug_enable_ && (frame_count_++ % debug_every_ == 0)) {
      GimbalCommand tx_cmd;
      {
        std::lock_guard<std::mutex> lk(tx_mtx_);
        tx_cmd = last_cmd_;
      }
      publish_debug(dbg, dets.size(), armors.size(), t_now, t_capture, gs_cap,
                    gs_now, ego, ego_vel, enemy, armor_debug, planned_cmd, tx_cmd);
    }
  }

  void shape_and_send(GimbalCommand cmd, const GimbalSample & gs_now, TimePoint t_now)
  {
    if (cmd.valid) {
      // Deadbands vs current micro feedback (identical to old pipeline).
      const double micro_pitch_for_lock =
        aimer_->params().micro_pitch_lock_opposite_sign ? -gs_now.pitch_raw
                                                        : gs_now.pitch_raw;
      double yaw_err = ang_diff(cmd.yaw_micro, gs_now.yaw_raw);
      double pitch_err = cmd.pitch_micro - micro_pitch_for_lock;
      if (std::fabs(yaw_err) < cmd_deadband_yaw_) cmd.yaw_micro = gs_now.yaw_raw;
      if (std::fabs(pitch_err) < cmd_deadband_pitch_) cmd.pitch_micro = micro_pitch_for_lock;

      // Max step clamps (protects against estimator glitches).
      if (cmd_max_delta_yaw_ > 0) {
        const double d = std::clamp(ang_diff(cmd.yaw_micro, gs_now.yaw_raw),
                                    -cmd_max_delta_yaw_, cmd_max_delta_yaw_);
        cmd.yaw_micro = gs_now.yaw_raw + d;
      }
      if (cmd_max_delta_pitch_ > 0) {
        const double d = std::clamp(cmd.pitch_micro - micro_pitch_for_lock,
                                    -cmd_max_delta_pitch_, cmd_max_delta_pitch_);
        cmd.pitch_micro = micro_pitch_for_lock + d;
      }
      if (!std::isfinite(cmd.yaw_micro) || !std::isfinite(cmd.pitch_micro)) {
        cmd.valid = false;
      }
      hold_until_ = t_now + std::chrono::duration_cast<Clock::duration>(
                              std::chrono::duration<double>(cmd_hold_time_));
    }

    if (!cmd.valid) {
      // Target lost: hold the last valid command for cmd_hold_time, then
      // follow the gimbal's own angles (freeze in place, never snap).
      {
        std::lock_guard<std::mutex> lk(tx_mtx_);
        if (t_now < hold_until_ && last_cmd_.valid) {
          // keep last_cmd_ as is, but stop shooting
          last_cmd_.shoot = false;
          last_cmd_t_ = t_now;
        } else {
          last_cmd_.valid = false;
        }
      }
      send_command(cmd, false);
      return;
    }

    if (cmd.shoot_scheduled && Clock::now() < cmd.shoot_at) {
      // TIMED regime: update the angles now, then precision-sleep to the
      // scheduled instant and send the shoot pulse (<= ~9 ms away). Coarse
      // sleep to 1 ms before, spin the rest — ~0.1 ms accuracy vs the ~6 ms
      // systematic-early error of frame-quantized firing.
      GimbalCommand pre = cmd;
      pre.shoot = false;
      send_command(pre, true);
      std::this_thread::sleep_until(cmd.shoot_at - std::chrono::milliseconds(1));
      while (Clock::now() < cmd.shoot_at) {
        // spin (bounded by the 1 ms coarse sleep above)
      }
      GimbalCommand fire = cmd;
      fire.shoot = true;
      send_command(fire, true);
      return;
    }

    send_command(cmd, true);
  }

  // ────────────────────────────── input modes ─────────────────────────────
  void kpt_callback(autoaim::msg::ArmorKeypointArray::ConstSharedPtr msg)
  {
    if (!solver_ready_) return;

    // Map the (system-clock) capture stamp onto the steady clock.
    const auto sys_now = std::chrono::system_clock::now();
    const TimePoint steady_now = Clock::now();
    const int64_t stamp_ns =
      static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL +
      msg->header.stamp.nanosec;
    const int64_t sys_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                             sys_now.time_since_epoch())
                             .count();
    const int64_t age = std::max<int64_t>(sys_ns - stamp_ns, 0);
    const TimePoint t_capture =
      steady_now - std::chrono::nanoseconds(std::min<int64_t>(age, 250000000LL));

    const float w = static_cast<float>(msg->image_width);
    const float h = static_cast<float>(msg->image_height);
    std::vector<ArmorPx> dets;
    for (const auto & k : msg->detections) {
      ArmorPx a;
      a.class_id = k.class_id;
      a.confidence = k.confidence;
      bool ok = true;
      for (int i = 0; i < 4; i++) {
        const float kx = k.keypoints_xy[2 * i], ky = k.keypoints_xy[2 * i + 1];
        // Border rejection: the old detector CLAMPS out-of-frame corners to
        // the edge; such a quad PnP-solves to a confident but wrong pose.
        if (!k.keypoint_valid[i] || kx < 2.f || kx > w - 2.f || ky < 2.f ||
            ky > h - 2.f) {
          ok = false;
          break;
        }
        a.corners[i] = cv::Point2f(kx, ky);
      }
      if (ok) dets.push_back(a);
    }
    process_frame(dets, t_capture);
  }

#if AIM_WITH_ZED_TRT
  void zed_loop()
  {
    ZedParams zp;
    zp.resolution = get_parameter("resolution").as_string();
    zp.fps = static_cast<int>(get_parameter("fps").as_int());
    zp.image_flip = get_parameter("image_flip").as_bool();
    zp.exposure_time_us = static_cast<int>(get_parameter("exposure_time_us").as_int());
    zp.gain = static_cast<int>(get_parameter("gain").as_int());
    zp.auto_white_balance = get_parameter("auto_white_balance").as_bool();

    ZedCamera cam(zp);
    while (running_ && !cam.open()) {
      RCLCPP_ERROR(get_logger(), "ZED open failed: %s — retrying", cam.error().c_str());
      std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    if (!running_) return;

    solver_->set_intrinsics(cam.fx(), cam.fy(), cam.cx(), cam.cy());
    camera_width_ = cam.width();
    camera_height_ = cam.height();
    camera_fx_ = cam.fx();
    camera_fy_ = cam.fy();
    camera_cx_ = cam.cx();
    camera_cy_ = cam.cy();
    solver_ready_ = true;

    DetectorParams dp;
    dp.engine_path = get_parameter("engine_path").as_string();
    dp.conf_threshold = static_cast<float>(get_parameter("conf_threshold").as_double());
    dp.nms_iou = static_cast<float>(get_parameter("nms_iou").as_double());
    dp.native_w = cam.width();
    dp.native_h = cam.height();
    DetectorTRT det(dp);
    if (!det.ok()) {
      RCLCPP_FATAL(get_logger(), "TensorRT init failed: %s", det.error().c_str());
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(get_logger(), "detector ready: net %dx%d, camera %dx%d@%d",
                det.net_size(), det.net_size(), cam.width(), cam.height(), zp.fps);

    cudaStream_t stream;
    cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking);

    int grab_fails = 0;
    while (running_) {
      const uint8_t * d_bgra = nullptr;
      size_t pitch = 0;
      TimePoint t_capture;
      if (!cam.grab(&d_bgra, &pitch, &t_capture)) {
        // Failed grabs can return immediately — pace the retry loop.
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        if (++grab_fails > 200) {  // ~1 s of failures: reopen
          RCLCPP_ERROR(get_logger(), "camera stalled — reopening");
          cam.close();
          while (running_ && !cam.open()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
          }
          grab_fails = 0;
        }
        continue;
      }
      grab_fails = 0;

      auto dets = det.infer(d_bgra, pitch, stream);
      process_frame(dets, t_capture);
    }
    cudaStreamDestroy(stream);
  }
#endif

#if AIM_WITH_RS_TRT
  void rs_loop()
  {
    RsParams rp;
    rp.serial_no = get_parameter("rs_serial_no").as_string();
    rp.width = static_cast<int>(get_parameter("rs_width").as_int());
    rp.height = static_cast<int>(get_parameter("rs_height").as_int());
    rp.fps = static_cast<int>(get_parameter("rs_fps").as_int());
    rp.auto_exposure = get_parameter("rs_auto_exposure").as_bool();
    rp.exposure = static_cast<int>(get_parameter("rs_exposure").as_int());
    rp.gain = static_cast<int>(get_parameter("rs_gain").as_int());
    const auto pf = get_parameter("rs_pixel_format").as_string();
    rp.use_yuyv = (pf == "yuyv" || pf == "YUYV");
    rp.flip_180 = get_parameter("rs_flip_180").as_bool();

    RsCamera cam(rp);
    while (running_ && !cam.open()) {
      RCLCPP_ERROR(get_logger(), "RealSense open failed: %s — retrying",
                   cam.error().c_str());
      std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    if (!running_) return;

    solver_->set_intrinsics(cam.fx(), cam.fy(), cam.cx(), cam.cy());
    camera_width_ = cam.width();
    camera_height_ = cam.height();
    camera_fx_ = cam.fx();
    camera_fy_ = cam.fy();
    camera_cx_ = cam.cx();
    camera_cy_ = cam.cy();
    solver_ready_ = true;

    DetectorParams dp;
    dp.engine_path = get_parameter("engine_path").as_string();
    dp.conf_threshold = static_cast<float>(get_parameter("conf_threshold").as_double());
    dp.nms_iou = static_cast<float>(get_parameter("nms_iou").as_double());
    dp.native_w = cam.width();
    dp.native_h = cam.height();
    DetectorTRT det(dp);
    if (!det.ok()) {
      RCLCPP_FATAL(get_logger(), "TensorRT init failed: %s", det.error().c_str());
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(get_logger(), "detector ready: net %dx%d, RS camera %dx%d@%d (%s%s)",
                det.net_size(), det.net_size(), cam.width(), cam.height(), rp.fps,
                cam.yuyv() ? "YUYV" : "BGR8", cam.flip() ? " flip180" : "");

    cudaStream_t stream;
    cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking);

    const int bpp = cam.yuyv() ? 2 : 3;
    const size_t color_bytes = static_cast<size_t>(cam.width()) * cam.height() * bpp;
    uint8_t * d_color = nullptr;
    cudaMalloc(reinterpret_cast<void **>(&d_color), color_bytes);

    int grab_fails = 0;
    while (running_) {
      const uint8_t * h_data = nullptr;
      size_t data_bytes = 0;
      TimePoint t_capture;
      if (!cam.grab(&h_data, &data_bytes, &t_capture)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        if (++grab_fails > 200) {
          RCLCPP_ERROR(get_logger(), "RealSense stalled — reopening");
          cam.close();
          while (running_ && !cam.open()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
          }
          grab_fails = 0;
        }
        continue;
      }
      grab_fails = 0;

      cudaMemcpyAsync(d_color, h_data, data_bytes, cudaMemcpyHostToDevice, stream);
      auto dets = det.infer_contiguous(d_color, cam.yuyv(), cam.flip(), stream);
      maybe_publish_rs_debug_image(h_data, cam.yuyv(), cam.flip(), cam.width(),
                                   cam.height(), dets);
      process_frame(dets, t_capture);
    }
    if (d_color) cudaFree(d_color);
    cudaStreamDestroy(stream);
  }
#endif

  // ────────────────────────────── debug ───────────────────────────────────
  autoaim::msg::AimDebugArmor make_armor_debug_msg(const ArmorDebugSample & ad) const
  {
    autoaim::msg::AimDebugArmor msg;
    msg.class_id = ad.px.class_id;
    msg.confidence = ad.px.confidence;
    for (int i = 0; i < 4; i++) {
      msg.corners_xy[2 * i] = ad.px.corners[i].x;
      msg.corners_xy[2 * i + 1] = ad.px.corners[i].y;
    }
    msg.accepted = ad.accepted;
    msg.solved = ad.solved;
    msg.reject_reason = ad.reject_reason;
    if (ad.solved) {
      msg.pos_cam_x = ad.world.pos_cam.x();
      msg.pos_cam_y = ad.world.pos_cam.y();
      msg.pos_cam_z = ad.world.pos_cam.z();
      msg.pos_world_x = ad.world.pos_world.x();
      msg.pos_world_y = ad.world.pos_world.y();
      msg.pos_world_z = ad.world.pos_world.z();
      msg.yaw = ad.world.yaw;
      msg.pitch = ad.world.pitch;
      msg.distance = ad.world.dist;
      msg.plate_theta = ad.world.theta_a;
      msg.theta_sigma = ad.world.theta_sigma;
      msg.reprojection_error = ad.world.reproj_err;
    }
    return msg;
  }

  void fill_command_debug(autoaim::msg::AimDebugFrame & m,
                          const GimbalCommand & planned_cmd,
                          const GimbalCommand & tx_cmd) const
  {
    m.planned_command_valid = planned_cmd.valid;
    m.planned_yaw_micro = planned_cmd.yaw_micro;
    m.planned_pitch_micro = planned_cmd.pitch_micro;
    m.planned_shoot = planned_cmd.shoot;
    m.planned_shoot_scheduled = planned_cmd.shoot_scheduled;
    m.planned_ff_yaw_rate = planned_cmd.ff_yaw_rate;
    m.planned_ff_pitch_rate = planned_cmd.ff_pitch_rate;

    m.tx_command_valid = tx_cmd.valid;
    m.tx_yaw_micro = tx_cmd.yaw_micro;
    m.tx_pitch_micro = tx_cmd.pitch_micro;
    m.tx_shoot = tx_cmd.shoot;
    m.tx_shoot_scheduled = tx_cmd.shoot_scheduled;
    m.tx_ff_yaw_rate = tx_cmd.ff_yaw_rate;
    m.tx_ff_pitch_rate = tx_cmd.ff_pitch_rate;
  }

  void publish_debug_frame_no_gimbal(const std::vector<ArmorPx> & dets, int enemy,
                                     TimePoint t_capture)
  {
    autoaim::msg::AimDebugFrame m;
    m.header.stamp = this->now();
    m.header.frame_id = "camera";
    m.frame_id = frame_count_;
    m.input_mode = input_mode_;
    m.enemy_class = enemy;
    m.solver_ready = solver_ready_.load();
    m.gimbal_sample_valid = false;
    {
      std::lock_guard<std::mutex> lk(status_mtx_);
      m.status_valid = status_.valid;
      m.micro_vx = status_.vx;
      m.micro_vy = status_.vy;
      m.micro_color = status_.color;
      m.micro_game_progress = status_.game_progress;
      m.micro_hp = status_.hp;
      m.micro_reserved9 = status_.reserved9;
    }
    m.serial_open = port_.is_open();
    m.shooting_active = shooting_active_;
    m.image_width = static_cast<uint32_t>(std::max(camera_width_, 0));
    m.image_height = static_cast<uint32_t>(std::max(camera_height_, 0));
    m.camera_fx = camera_fx_;
    m.camera_fy = camera_fy_;
    m.camera_cx = camera_cx_;
    m.camera_cy = camera_cy_;
    m.detection_count = static_cast<uint32_t>(dets.size());
    m.accepted_count = 0;
    m.capture_age_ms = seconds(Clock::now(), t_capture) * 1000.0;
    m.pipeline_latency_ms = m.capture_age_ms;

    m.armors.reserve(dets.size());
    for (const auto & d : dets) {
      ArmorDebugSample ad;
      ad.px = d;
      ad.reject_reason = "no_gimbal_sample";
      m.armors.push_back(make_armor_debug_msg(ad));
    }
    dbg_frame_pub_->publish(m);
  }

  void maybe_publish_rs_debug_image(const uint8_t * data, bool is_yuyv, bool flip180,
                                    int width, int height,
                                    const std::vector<ArmorPx> & dets)
  {
    if (!debug_image_pub_ || !debug_image_enable_) return;
    if ((debug_image_count_++ % static_cast<uint64_t>(debug_image_every_)) != 0) return;

    cv::Mat bgr;
    if (is_yuyv) {
      cv::Mat yuyv(height, width, CV_8UC2, const_cast<uint8_t *>(data));
      cv::cvtColor(yuyv, debug_image_bgr_, cv::COLOR_YUV2BGR_YUYV);
      if (flip180) cv::flip(debug_image_bgr_, debug_image_bgr_, -1);
      bgr = debug_image_bgr_;
    } else {
      cv::Mat raw(height, width, CV_8UC3, const_cast<uint8_t *>(data));
      if (flip180) {
        cv::flip(raw, debug_image_bgr_, -1);
        bgr = debug_image_bgr_;
      } else {
        bgr = raw;
      }
    }

    sensor_msgs::msg::Image img;
    img.header.stamp = this->now();
    img.header.frame_id = "camera";
    img.height = static_cast<uint32_t>(height);
    img.width = static_cast<uint32_t>(width);
    img.encoding = "bgr8";
    img.is_bigendian = 0;
    img.step = static_cast<uint32_t>(width * 3);
    const size_t n = static_cast<size_t>(img.step) * height;
    img.data.assign(bgr.data, bgr.data + n);
    (void)dets;  // The viewer overlays detections from /aimv2/debug_frame.
    debug_image_pub_->publish(img);
  }

  void publish_debug(const AimDebug & d, size_t n_detections, size_t n_armors,
                     TimePoint t_now, TimePoint t_capture,
                     const GimbalSample & gs_cap, const GimbalSample & gs_now,
                     const Eigen::Vector2d & ego, const Eigen::Vector2d & ego_vel,
                     int enemy, const std::vector<ArmorDebugSample> & armor_debug,
                     const GimbalCommand & planned_cmd, const GimbalCommand & tx_cmd)
  {
    std_msgs::msg::Float32MultiArray m;
    const auto & x = tracker_->x();
    m.data = {
      static_cast<float>(d.regime),
      static_cast<float>(tracker_->state()),
      static_cast<float>(d.face),
      static_cast<float>(d.p_hit),
      static_cast<float>(d.omega),
      static_cast<float>(d.dist),
      static_cast<float>(d.flight_time),
      static_cast<float>(d.horizon),
      static_cast<float>(d.locked ? 1 : 0),
      static_cast<float>(n_armors),
      static_cast<float>(seconds(t_now, t_capture) * 1000.0),  // pipeline ms
      static_cast<float>(x(0)), static_cast<float>(x(2)), static_cast<float>(x(4)),
      static_cast<float>(x(6)), static_cast<float>(x(7)), static_cast<float>(x(8)),
      static_cast<float>(x(9)), static_cast<float>(x(10)),
      static_cast<float>(d.next_shot_in),
    };
    dbg_pub_->publish(m);

    if (debug_full_ && dbg_frame_pub_) {
      autoaim::msg::AimDebugFrame f;
      f.header.stamp = this->now();
      f.header.frame_id = "camera";
      f.frame_id = frame_count_;
      f.input_mode = input_mode_;
      f.enemy_class = enemy;
      f.solver_ready = solver_ready_.load();
      f.gimbal_sample_valid = true;
      f.serial_open = port_.is_open();
      f.shooting_active = shooting_active_;

      f.image_width = static_cast<uint32_t>(std::max(camera_width_, 0));
      f.image_height = static_cast<uint32_t>(std::max(camera_height_, 0));
      f.camera_fx = camera_fx_;
      f.camera_fy = camera_fy_;
      f.camera_cx = camera_cx_;
      f.camera_cy = camera_cy_;

      f.detection_count = static_cast<uint32_t>(n_detections);
      f.accepted_count = static_cast<uint32_t>(n_armors);
      f.armors.reserve(armor_debug.size());
      for (const auto & ad : armor_debug) {
        f.armors.push_back(make_armor_debug_msg(ad));
      }

      f.capture_age_ms = seconds(t_now, t_capture) * 1000.0;
      f.pipeline_latency_ms = f.capture_age_ms;
      if (last_debug_publish_t_.time_since_epoch().count() != 0) {
        f.publish_period_ms = seconds(t_now, last_debug_publish_t_) * 1000.0;
      }
      last_debug_publish_t_ = t_now;

      f.gimbal_capture_yaw = gs_cap.yaw;
      f.gimbal_capture_pitch = gs_cap.pitch;
      f.gimbal_capture_yaw_raw = gs_cap.yaw_raw;
      f.gimbal_capture_pitch_raw = gs_cap.pitch_raw;
      f.gimbal_now_yaw = gs_now.yaw;
      f.gimbal_now_pitch = gs_now.pitch;
      f.gimbal_now_yaw_raw = gs_now.yaw_raw;
      f.gimbal_now_pitch_raw = gs_now.pitch_raw;

      {
        std::lock_guard<std::mutex> lk(status_mtx_);
        f.status_valid = status_.valid;
        f.micro_vx = status_.vx;
        f.micro_vy = status_.vy;
        f.micro_color = status_.color;
        f.micro_game_progress = status_.game_progress;
        f.micro_hp = status_.hp;
        f.micro_reserved9 = status_.reserved9;
      }
      f.ego_x = ego.x();
      f.ego_y = ego.y();
      f.ego_vx = ego_vel.x();
      f.ego_vy = ego_vel.y();

      f.tracker_state = static_cast<int32_t>(tracker_->state());
      f.tracker_tracking = tracker_->tracking();
      f.tracker_generation = tracker_->generation();
      f.tracker_last_face = tracker_->last_matched_face();
      f.tracker_matched_count = tracker_->matched_count();
      const auto & P = tracker_->P();
      for (int i = 0; i < 11; i++) {
        f.tracker_x[i] = x(i);
        f.tracker_p_diag[i] = P(i, i);
      }

      f.fire_regime = static_cast<int32_t>(d.regime);
      f.selected_face = d.face;
      f.hit_probability = d.p_hit;
      f.target_omega = d.omega;
      f.target_distance = d.dist;
      f.flight_time = d.flight_time;
      f.prediction_horizon = d.horizon;
      f.lock_error_yaw = d.lock_err_yaw;
      f.lock_error_pitch = d.lock_err_pitch;
      f.locked = d.locked;
      f.aim_point_x = d.aim_point.x();
      f.aim_point_y = d.aim_point.y();
      f.aim_point_z = d.aim_point.z();
      f.next_shot_in = d.next_shot_in;
      fill_command_debug(f, planned_cmd, tx_cmd);
      dbg_frame_pub_->publish(f);
    }

    if (debug_markers_ && marker_pub_ && tracker_->tracking()) {
      visualization_msgs::msg::MarkerArray arr;
      auto mk = [&](int id, double px, double py, double pz, float r, float g,
                    float b, double s) {
        visualization_msgs::msg::Marker mm;
        mm.header.frame_id = "odom";
        mm.header.stamp = this->now();
        mm.ns = "aimv2";
        mm.id = id;
        mm.type = visualization_msgs::msg::Marker::SPHERE;
        mm.action = visualization_msgs::msg::Marker::ADD;
        mm.pose.position.x = px;
        mm.pose.position.y = py;
        mm.pose.position.z = pz;
        mm.pose.orientation.w = 1.0;
        mm.scale.x = mm.scale.y = mm.scale.z = s;
        mm.color.a = 1.0;
        mm.color.r = r;
        mm.color.g = g;
        mm.color.b = b;
        return mm;
      };
      arr.markers.push_back(mk(0, x(0), x(2), x(4), 0, 1, 0, 0.10));
      for (int i = 0; i < 4; i++) {
        const auto p = Tracker::armor_pos(x, i);
        arr.markers.push_back(mk(1 + i, p.x(), p.y(), p.z(), 1, 0, 0, 0.07));
      }
      arr.markers.push_back(
        mk(5, d.aim_point.x(), d.aim_point.y(), d.aim_point.z(), 0, 1, 1, 0.09));
      marker_pub_->publish(arr);
    }
  }

  // ────────────────────────────── members ─────────────────────────────────
  TimePoint t_start_;
  std::atomic<bool> running_{true};

  std::unique_ptr<Solver> solver_;
  std::unique_ptr<Tracker> tracker_;
  std::unique_ptr<Aimer> aimer_;
  std::atomic<bool> solver_ready_{false};

  GimbalBuffer gimbal_buf_;
  SerialPort port_;

  mutable std::mutex status_mtx_;
  RobotStatus status_;
  std::atomic<double> bullet_speed_rx_{0.0};

  std::mutex ego_mtx_;
  Eigen::Vector2d ego_pos_{0, 0};
  Eigen::Vector2d ego_vel_{0, 0};
  Eigen::Vector2d ego_vf_{0, 0};
  TimePoint ego_last_t_{};

  std::mutex tx_mtx_;
  GimbalCommand last_cmd_;
  TimePoint last_cmd_t_{};
  TimePoint hold_until_{};

  std::thread serial_thread_, heartbeat_thread_, vision_thread_, sim_gimbal_thread_;

  // Params
  std::string input_mode_, serial_device_, serial_parity_;
  int serial_baud_ = 500000;
  bool shooting_active_ = false, rotating_chassis_ = false, send_ff_ = false;
  double cmd_timeout_ = 0.3, heartbeat_hz_ = 100.0;
  double yaw_sign_ = 1.0, pitch_sign_ = -1.0;
  bool pitch_feedback_opposite_ = true;
  int color_index_ = 4, bullet_speed_index_ = -1, fixed_target_class_ = -1;
  double min_confidence_ = 0.3, max_armor_dist_ = 8.0;
  double cmd_deadband_yaw_ = 0.005, cmd_deadband_pitch_ = 0.008;
  double cmd_max_delta_yaw_ = 0.8, cmd_max_delta_pitch_ = 0.6;
  double cmd_hold_time_ = 0.25;
  bool ego_enable_ = false, ego_body_frame_ = true;
  double ego_deadband_ = 0.05, ego_lpf_alpha_ = 0.6, ego_vmax_ = 3.5;
  int chassis_heading_index_ = -1;
  bool debug_enable_ = false, debug_full_ = true, debug_markers_ = false;
  bool debug_image_enable_ = false;
  int debug_every_ = 4, debug_image_every_ = 4;
  std::string debug_image_topic_{"/aimv2/debug_image"};
  uint64_t frame_count_ = 0;
  uint64_t debug_image_count_ = 0;
  TimePoint last_debug_publish_t_{};
  int camera_width_ = 0, camera_height_ = 0;
  double camera_fx_ = 0.0, camera_fy_ = 0.0, camera_cx_ = 0.0, camera_cy_ = 0.0;
  cv::Mat debug_image_bgr_;

  rclcpp::Subscription<autoaim::msg::ArmorKeypointArray>::SharedPtr kpt_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr dbg_pub_;
  rclcpp::Publisher<autoaim::msg::AimDebugFrame>::SharedPtr dbg_frame_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace aim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<aim::AimNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
