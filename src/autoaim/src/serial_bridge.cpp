// =============================================================================
// serial_bridge.cpp
//
// Serial bridge for RoboTO CV + Navigation architecture.
//
// This node lives in the CV container. It is the only node that talks to the
// microcontroller over serial.
//
// TX (Jetson -> Micro): 7 x float32 LE = 28 bytes
//   [0] timestamp since node start [s]
//   [1] turret yaw   absolute target in micro frame [rad]
//   [2] turret pitch absolute target [rad]
//   [3] shoot flag
//   [4] nav_x
//   [5] nav_y
//   [6] slot for autospinning of the chassis, if ROTATING_CHASSIS = False it doesn't rotate, if 1 it rotates
//
// RX (Micro -> Jetson): 10 x float32 LE = 40 bytes
//   [0] yaw
//   [1] pitch
//   [2] vx
//   [3] vy
//   [4] color: 0 RED, 1 BLUE
//   [5] game_progress: 1 preparation, 2 init 15s, 3 countdown 5s,
//       4 in match, 5 match ended/settling
//   [6] HP
//   [7] resupply status
//   [8] center status: 0 free, 1 ours, 2 enemy
//   [9] reserved
//
// /micro_status layout:
//   RX[0..9] + TX echo [turret_yaw, turret_pitch, shoot, nav_x, nav_y, nav_angle]
//   = 16 floats.
//
// LAB OVERRIDE:
//   If lab_override_micro_status=true, this node overwrites only the published
//   /micro_status fields [4], [5], [6], [7], [8] before publishing them to ROS.
//   It does NOT change the serial data sent to the microcontroller.
//   This lets you start/stop the match and change HP/center/team from terminal.
//
// Runtime pipeline switches:
//   enable_nav_pipeline:     false -> nav_x/nav_y forced to 0
//   enable_turret_pipeline:  false -> shoot forced to 0, yaw/pitch held or zeroed
//
// Subscribes:
//   /turret/cmd   geometry_msgs/Twist
//     angular.z = turret yaw absolute in micro frame [rad]
//     angular.y = turret pitch [rad]
//     angular.x = shoot flag
//
//   /cmd_vel_NAV  geometry_msgs/Twist
//     linear.x = nav_x
//     linear.y = nav_y
//
// Publishes:
//   /micro_status std_msgs/Float32MultiArray
// =============================================================================

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <linux/serial.h>
#include <pthread.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace
{
constexpr size_t TX_NUM_VALUES = 7;
constexpr size_t TX_PACKET_SIZE = TX_NUM_VALUES * 4;
constexpr size_t RX_NUM_VALUES = 10;
constexpr size_t RX_PACKET_SIZE = RX_NUM_VALUES * 4;

constexpr uint8_t TX_FRAME_HEADER[2] = {0xA5, 0x5A};
constexpr uint8_t RX_FRAME_HEADER[2] = {0x5A, 0xA5};
constexpr size_t RX_FRAME_SIZE = 2 + RX_PACKET_SIZE + 1;
constexpr size_t RX_BUFFER_CAP = RX_FRAME_SIZE * 64;

constexpr bool SHOOTING_ACTIVE = false;
constexpr bool ROTATING_CHASSIS = false;

uint8_t crc8(const uint8_t * data, size_t len)
{
  uint8_t crc = 0x00;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; i++) {
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x07)
                         : static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}

speed_t baud_to_speed(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 921600: return B921600;
    case 1000000: return B1000000;
    case 2000000: return B2000000;
    default: return 0;
  }
}
}  // namespace

class SerialBridgeNode : public rclcpp::Node
{
public:
  SerialBridgeNode()
  : Node("micro_communications_node"),
    t_start_(std::chrono::steady_clock::now())
  {
    port_ = declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
    baudrate_ = static_cast<int>(declare_parameter<int>("serial_baudrate", 500000));
    reconnect_interval_ = declare_parameter<double>("serial_reconnect_interval", 2.0);
    rx_timeout_ = declare_parameter<double>("serial_rx_timeout", 3.0);
    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.3);
    use_framed_ = declare_parameter<bool>("use_framed_protocol", false);
    const double tx_hz = declare_parameter<double>("serial_tx_hz", 100.0);

    parity_ = declare_parameter<std::string>("serial_parity", "even");
    low_latency_ = declare_parameter<bool>("low_latency", false);
    thread_priority_ = static_cast<int>(declare_parameter<int>("thread_priority", 0));

    turret_cmd_topic_ = declare_parameter<std::string>("turret_cmd_topic", "/turret/cmd");
    nav_cmd_topic_ = declare_parameter<std::string>("nav_cmd_topic", "/cmd_vel_NAV");
    micro_status_topic_ = declare_parameter<std::string>("micro_status_topic", "/micro_status");

    shooting_active_ = declare_parameter<bool>("shooting_active", false);
    rotating_chassis_ = declare_parameter<bool>("rotating_chassis", false);



    declare_parameter<bool>("enable_nav_pipeline", true);
    declare_parameter<bool>("enable_turret_pipeline", true);
    declare_parameter<bool>("hold_last_turret_when_disabled", true);

    // LAB OVERRIDE PARAMETERS
    // These affect only the ROS topic /micro_status, not the serial TX packet.
    declare_parameter<bool>("lab_override_micro_status", true);
    declare_parameter<double>("lab_override_team", 1.0);           // 0 red, 1 blue
    declare_parameter<double>("lab_override_game_progress", 1.0);  // 1 wait, 4 in match, 5 ended
    declare_parameter<double>("lab_override_health", 200.0);
    declare_parameter<double>("lab_override_resupply_status", 0.0);
    declare_parameter<double>("lab_override_center_status", 0.0);  // 0 free, 1 ours, 2 enemy

    rx_buf_.reserve(RX_BUFFER_CAP);

    sub_turret_ = create_subscription<geometry_msgs::msg::Twist>(
      turret_cmd_topic_, rclcpp::SystemDefaultsQoS(),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lk(mtx_);
        turret_shoot_ = msg->angular.x;
        turret_pitch_ = msg->angular.y;
        turret_yaw_ = msg->angular.z;
        last_turret_time_ = mono_now();
        have_turret_cmd_ = true;
      });

    sub_nav_ = create_subscription<geometry_msgs::msg::Twist>(
      nav_cmd_topic_, rclcpp::SystemDefaultsQoS(),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lk(mtx_);
        nav_x_ = msg->linear.x;
        nav_y_ = msg->linear.y;
        nav_angle_ = 0.0;
        last_nav_time_ = mono_now();
        have_nav_cmd_ = true;
      });

    pub_status_ = create_publisher<std_msgs::msg::Float32MultiArray>(micro_status_topic_, 10);

    last_rx_time_ = mono_now();
    last_turret_time_ = mono_now();
    last_nav_time_ = mono_now();

    openSerial();

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(tx_hz, 1.0)),
      std::bind(&SerialBridgeNode::serialTick, this));

    RCLCPP_INFO(
      get_logger(),
      "serial_bridge started: turret_cmd=%s nav_cmd=%s micro_status=%s",
      turret_cmd_topic_.c_str(), nav_cmd_topic_.c_str(), micro_status_topic_.c_str());
  }

  ~SerialBridgeNode() override
  {
    if (fd_ >= 0) {
      ::close(fd_);
      RCLCPP_INFO(get_logger(), "Serial port closed.");
    }
  }

  void applyThreadPriority()
  {
    if (thread_priority_ <= 0) {return;}

    sched_param sp{};
    sp.sched_priority = thread_priority_;

    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
      RCLCPP_WARN(
        get_logger(),
        "Could not set SCHED_FIFO priority %d. Need rtprio rlimit or CAP_SYS_NICE.",
        thread_priority_);
    } else {
      RCLCPP_INFO(get_logger(), "Serial bridge running at SCHED_FIFO priority %d.",
        thread_priority_);
    }
  }

private:
  double mono_now() const
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  bool getBoolParam(const std::string & name, bool fallback)
  {
    rclcpp::Parameter p;
    if (get_parameter(name, p) && p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      return p.as_bool();
    }
    return fallback;
  }

  double getNumericParam(const std::string & name, double fallback)
  {
    rclcpp::Parameter p;
    if (!get_parameter(name, p)) {
      return fallback;
    }

    if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      return p.as_double();
    }

    if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      return static_cast<double>(p.as_int());
    }

    return fallback;
  }

  bool openSerial()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }

    const speed_t speed = baud_to_speed(baudrate_);
    if (speed == 0) {
      RCLCPP_ERROR(get_logger(), "Unsupported baudrate %d", baudrate_);
      return false;
    }

    int fd = ::open(port_.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0) {
      RCLCPP_WARN(get_logger(),
        "Failed to open serial port %s: %s — will retry every %.1fs",
        port_.c_str(), std::strerror(errno), reconnect_interval_);
      return false;
    }

    termios tio{};
    if (tcgetattr(fd, &tio) != 0) {
      RCLCPP_WARN(get_logger(), "tcgetattr failed on %s: %s", port_.c_str(),
        std::strerror(errno));
      ::close(fd);
      return false;
    }

    cfmakeraw(&tio);
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);

    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CRTSCTS;

    if (parity_ == "even") {
      tio.c_cflag |= PARENB;
      tio.c_cflag &= ~PARODD;
      tio.c_iflag |= IGNPAR;
    } else if (parity_ == "odd") {
      tio.c_cflag |= PARENB | PARODD;
      tio.c_iflag |= IGNPAR;
    } else {
      tio.c_cflag &= ~PARENB;
    }

    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
      RCLCPP_WARN(get_logger(), "tcsetattr failed on %s: %s", port_.c_str(),
        std::strerror(errno));
      ::close(fd);
      return false;
    }

    if (low_latency_) {
      serial_struct ss{};
      if (ioctl(fd, TIOCGSERIAL, &ss) == 0) {
        ss.flags |= ASYNC_LOW_LATENCY;
        if (ioctl(fd, TIOCSSERIAL, &ss) != 0) {
          RCLCPP_WARN(
            get_logger(),
            "ASYNC_LOW_LATENCY not supported on %s. This is normal on many CDC-ACM devices.",
            port_.c_str());
        }
      }
    }

    tcflush(fd, TCIOFLUSH);

    fd_ = fd;
    last_rx_time_ = mono_now();
    consecutive_errors_ = 0;
    rx_buf_.clear();

    RCLCPP_INFO(get_logger(), "Serial port %s opened successfully at %d baud.",
      port_.c_str(), baudrate_);
    return true;
  }

  void closeAndScheduleReconnect(const std::string & reason)
  {
    RCLCPP_WARN(get_logger(), "Serial link lost (%s) — will reconnect.", reason.c_str());
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
    consecutive_errors_ = 0;
  }

  void serialTick()
  {
    const double now = mono_now();

    if (fd_ < 0) {
      if (now - last_reconnect_attempt_ >= reconnect_interval_) {
        last_reconnect_attempt_ = now;
        RCLCPP_INFO(get_logger(), "Attempting serial reconnect on %s...", port_.c_str());
        openSerial();
      }
      return;
    }

    if (now - last_rx_time_ > rx_timeout_) {
      closeAndScheduleReconnect("no valid RX for " + std::to_string(rx_timeout_) + "s");
      last_reconnect_attempt_ = now;
      return;
    }

    const bool enable_nav = getBoolParam("enable_nav_pipeline", true);
    const bool enable_turret = getBoolParam("enable_turret_pipeline", true);
    const bool hold_last_turret = getBoolParam("hold_last_turret_when_disabled", true);

    float tx[TX_NUM_VALUES];
    {
      std::lock_guard<std::mutex> lk(mtx_);

      const bool turret_stale = (now - last_turret_time_) > cmd_timeout_;

      tx[0] = static_cast<float>(
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start_).count());

      if (enable_turret) {
        tx[1] = static_cast<float>(turret_yaw_);
        tx[2] = static_cast<float>(turret_pitch_);
        tx[3] = static_cast<float>(shooting_active_ ? (turret_stale ? 0.0 : turret_shoot_) : 0.0);
      } else {
        if (hold_last_turret) {
          tx[1] = static_cast<float>(turret_yaw_);
          tx[2] = static_cast<float>(turret_pitch_);
        } else {
          tx[1] = 0.0f;
          tx[2] = 0.0f;
        }
        tx[3] = 0.0f;
      }

      if (enable_nav) {
        tx[4] = static_cast<float>(nav_x_);
        tx[5] = static_cast<float>(nav_y_);
      } else {
        tx[4] = 0.0f;
        tx[5] = 0.0f;
      }

      tx[6] = static_cast<float>(rotating_chassis_ ? 1.0 : 0.0);

      tx_echo_turret_yaw_ = tx[1];
      tx_echo_turret_pitch_ = tx[2];
      tx_echo_turret_shoot_ = tx[3];
      tx_echo_nav_x_ = tx[4];
      tx_echo_nav_y_ = tx[5];
      tx_echo_nav_angle_ = tx[6];
    }

    uint8_t frame[2 + TX_PACKET_SIZE + 1];
    const uint8_t * out_ptr = nullptr;
    size_t out_len = 0;

    if (use_framed_) {
      frame[0] = TX_FRAME_HEADER[0];
      frame[1] = TX_FRAME_HEADER[1];
      std::memcpy(frame + 2, tx, TX_PACKET_SIZE);
      frame[2 + TX_PACKET_SIZE] = crc8(frame + 2, TX_PACKET_SIZE);
      out_ptr = frame;
      out_len = sizeof(frame);
    } else {
      out_ptr = reinterpret_cast<const uint8_t *>(tx);
      out_len = TX_PACKET_SIZE;
    }

    ssize_t written = ::write(fd_, out_ptr, out_len);
    if (written != static_cast<ssize_t>(out_len)) {
      consecutive_errors_++;
      RCLCPP_WARN(get_logger(), "Serial TX error (%d): %s",
        consecutive_errors_, written < 0 ? std::strerror(errno) : "short write");
      if (consecutive_errors_ >= 5) {
        closeAndScheduleReconnect(std::to_string(consecutive_errors_) +
          " consecutive TX/RX errors");
        last_reconnect_attempt_ = now;
      }
      return;
    }

    consecutive_errors_ = 0;

    const bool rx_ok = use_framed_ ? drainRxFramed() : drainRxLegacy();
    if (!rx_ok) {
      consecutive_errors_++;
      RCLCPP_WARN(get_logger(), "Serial RX error (%d): %s",
        consecutive_errors_, std::strerror(errno));
      if (consecutive_errors_ >= 5) {
        closeAndScheduleReconnect(std::to_string(consecutive_errors_) +
          " consecutive TX/RX errors");
        last_reconnect_attempt_ = now;
      }
    }
  }

  static bool sanityOk(const float * v, size_t n)
  {
    for (size_t i = 0; i < n; i++) {
      if (!(std::fabs(v[i]) < 1e6f)) {return false;}
    }

    // Pitch feedback should normally be in a small radian range.
    if (std::fabs(v[1]) > 2.0f) {return false;}

    return true;
  }

  void applyLabOverride(std_msgs::msg::Float32MultiArray & msg)
  {
    if (!getBoolParam("lab_override_micro_status", false)) {
      return;
    }

    if (msg.data.size() > 4) {
      msg.data[4] = static_cast<float>(getNumericParam("lab_override_team", 0.0));
    }

    if (msg.data.size() > 5) {
      msg.data[5] = static_cast<float>(getNumericParam("lab_override_game_progress", 1.0));
    }

    if (msg.data.size() > 6) {
      msg.data[6] = static_cast<float>(getNumericParam("lab_override_health", 100.0));
    }

    if (msg.data.size() > 7) {
      msg.data[7] = static_cast<float>(getNumericParam("lab_override_resupply_status", 0.0));
    }

    if (msg.data.size() > 8) {
      msg.data[8] = static_cast<float>(getNumericParam("lab_override_center_status", 0.0));
    }
  }

  void publishStatus(const float * rx)
  {
    std_msgs::msg::Float32MultiArray msg;
    msg.data.reserve(RX_NUM_VALUES + 6);
    msg.data.assign(rx, rx + RX_NUM_VALUES);

    applyLabOverride(msg);

    {
      std::lock_guard<std::mutex> lk(mtx_);
      msg.data.push_back(static_cast<float>(tx_echo_turret_yaw_));
      msg.data.push_back(static_cast<float>(tx_echo_turret_pitch_));
      msg.data.push_back(static_cast<float>(tx_echo_turret_shoot_));
      msg.data.push_back(static_cast<float>(tx_echo_nav_x_));
      msg.data.push_back(static_cast<float>(tx_echo_nav_y_));
      msg.data.push_back(static_cast<float>(tx_echo_nav_angle_));
    }

    pub_status_->publish(msg);
    last_rx_time_ = mono_now();
    consecutive_errors_ = 0;
  }

  int bytesAvailable()
  {
    int n = 0;
    if (ioctl(fd_, FIONREAD, &n) != 0) {return -1;}
    return n;
  }

  bool drainRxLegacy()
  {
    const int avail = bytesAvailable();
    if (avail < 0) {return false;}
    if (static_cast<size_t>(avail) < RX_PACKET_SIZE) {return true;}

    uint8_t scratch[256];
    size_t stale = (static_cast<size_t>(avail) / RX_PACKET_SIZE - 1) * RX_PACKET_SIZE;

    while (stale > 0) {
      const ssize_t r = ::read(fd_, scratch, std::min(stale, sizeof(scratch)));
      if (r <= 0) {return false;}
      stale -= static_cast<size_t>(r);
    }

    uint8_t pkt[RX_PACKET_SIZE];
    size_t got = 0;

    while (got < RX_PACKET_SIZE) {
      const ssize_t r = ::read(fd_, pkt + got, RX_PACKET_SIZE - got);
      if (r < 0) {return false;}
      if (r == 0) {return true;}
      got += static_cast<size_t>(r);
    }

    float vals[RX_NUM_VALUES];
    std::memcpy(vals, pkt, RX_PACKET_SIZE);

    if (sanityOk(vals, RX_NUM_VALUES)) {
      publishStatus(vals);
    } else {
      RCLCPP_WARN(get_logger(), "RX sanity check failed — flushing input buffer.");
      tcflush(fd_, TCIFLUSH);
    }

    return true;
  }

  bool drainRxFramed()
  {
    const int avail = bytesAvailable();
    if (avail < 0) {return false;}

    if (avail > 0) {
      const size_t old = rx_buf_.size();
      rx_buf_.resize(old + static_cast<size_t>(avail));

      ssize_t r = ::read(fd_, rx_buf_.data() + old, static_cast<size_t>(avail));
      if (r < 0) {
        rx_buf_.resize(old);
        return false;
      }

      rx_buf_.resize(old + static_cast<size_t>(r));
    }

    if (rx_buf_.size() > RX_BUFFER_CAP) {
      rx_buf_.erase(rx_buf_.begin(), rx_buf_.end() - RX_FRAME_SIZE);
    }

    float latest[RX_NUM_VALUES];
    bool have_latest = false;

    while (true) {
      auto it = std::search(
        rx_buf_.begin(), rx_buf_.end(), RX_FRAME_HEADER, RX_FRAME_HEADER + 2);

      if (it == rx_buf_.end()) {
        if (rx_buf_.size() > 1) {
          rx_buf_.erase(rx_buf_.begin(), rx_buf_.end() - 1);
        }
        break;
      }

      if (it != rx_buf_.begin()) {
        rx_buf_.erase(rx_buf_.begin(), it);
      }

      if (rx_buf_.size() < RX_FRAME_SIZE) {
        break;
      }

      const uint8_t * payload = rx_buf_.data() + 2;
      const uint8_t rx_crc = rx_buf_[2 + RX_PACKET_SIZE];

      if (crc8(payload, RX_PACKET_SIZE) == rx_crc) {
        float vals[RX_NUM_VALUES];
        std::memcpy(vals, payload, RX_PACKET_SIZE);

        if (sanityOk(vals, RX_NUM_VALUES)) {
          std::memcpy(latest, vals, sizeof(vals));
          have_latest = true;
        }

        rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + RX_FRAME_SIZE);
      } else {
        rx_buf_.erase(rx_buf_.begin());
      }
    }

    if (have_latest) {
      publishStatus(latest);
    }

    return true;
  }

  std::mutex mtx_;

  double turret_yaw_ = 0.0;
  double turret_pitch_ = 0.0;
  double turret_shoot_ = 0.0;
  double nav_x_ = 0.0;
  double nav_y_ = 0.0;
  double nav_angle_ = 0.0;

  double tx_echo_turret_yaw_ = 0.0;
  double tx_echo_turret_pitch_ = 0.0;
  double tx_echo_turret_shoot_ = 0.0;
  double tx_echo_nav_x_ = 0.0;
  double tx_echo_nav_y_ = 0.0;
  double tx_echo_nav_angle_ = 0.0;

  double last_turret_time_ = 0.0;
  double last_nav_time_ = 0.0;
  bool have_turret_cmd_ = false;
  bool have_nav_cmd_ = false;
  bool shooting_active_ = false;
  bool rotating_chassis_ = false;

  std::string port_;
  std::string parity_;
  std::string turret_cmd_topic_;
  std::string nav_cmd_topic_;
  std::string micro_status_topic_;

  int baudrate_ = 500000;
  double reconnect_interval_ = 2.0;
  double rx_timeout_ = 3.0;
  double cmd_timeout_ = 0.3;
  bool use_framed_ = false;
  bool low_latency_ = false;
  int thread_priority_ = 0;

  int fd_ = -1;
  double last_rx_time_ = 0.0;
  double last_reconnect_attempt_ = 0.0;
  int consecutive_errors_ = 0;

  std::vector<uint8_t> rx_buf_;
  std::chrono::steady_clock::time_point t_start_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_turret_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_nav_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_status_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SerialBridgeNode>();
  node->applyThreadPriority();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
