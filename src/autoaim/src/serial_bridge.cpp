// =============================================================================
// serial_bridge.cpp — C++ port of serial_bridge.py (drop-in replacement)
//
// WHY C++: the Python bridge sits inside the tightest part of the latency
// budget (±17 ms total at 150 RPM). CPython GC pauses and scheduler jitter on
// a loaded Jetson add 1–10 ms of unpredictable delay exactly where it hurts.
// This port keeps the protocol and behavior byte-identical and adds two
// OPT-IN extras (low_latency ioctl, SCHED_FIFO priority), both off by default.
//
// PROTOCOL (unchanged — no firmware modification needed):
//   TX (Jetson -> Micro): 7 x float32 LE = 28 bytes
//     [0] timestamp since node start [s]
//     [1] yaw   absolute target [rad]
//     [2] pitch absolute target [rad]
//     [3] shoot flag (watchdog-forced to 0 if /cmd_vel_AI stale > cmd_timeout)
//     [4] nav_x   [5] nav_y   [6] nav_angle
//   RX (Micro -> Jetson): 10 x float32 LE = 40 bytes
//     [0] yaw, [1] pitch, [2] vx, [3] vy, [4..9] other status
//
//   Framed mode (use_framed_protocol: true, requires updated firmware):
//     TX: 0xA5 0x5A | 28B payload | CRC8(payload)   = 31 bytes
//     RX: 0x5A 0xA5 | 40B payload | CRC8(payload)   = 43 bytes
//     CRC8 poly 0x07 init 0x00 (same as CHANGES.md snippet).
//
// /micro_status layout (unchanged): RX[0..9] + TX echo [yaw, pitch, shoot,
//   nav_x, nav_y, nav_angle] = 16 floats.
//
// Node name, topic names and ALL parameter names match serial_bridge.py, so
// the launch files only need executable="serial_bridge".
// =============================================================================

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <linux/serial.h>  // ASYNC_LOW_LATENCY (opt-in)
#include <pthread.h>

#include <algorithm>
#include <chrono>
#include <cmath>
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
constexpr size_t TX_PACKET_SIZE = TX_NUM_VALUES * 4;   // 28
constexpr size_t RX_NUM_VALUES = 10;
constexpr size_t RX_PACKET_SIZE = RX_NUM_VALUES * 4;   // 40

constexpr uint8_t TX_FRAME_HEADER[2] = {0xA5, 0x5A};
constexpr uint8_t RX_FRAME_HEADER[2] = {0x5A, 0xA5};
constexpr size_t RX_FRAME_SIZE = 2 + RX_PACKET_SIZE + 1;  // 43
constexpr size_t RX_BUFFER_CAP = RX_FRAME_SIZE * 64;

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

// Same pipeline switches as the Python file. Kept as compile-time constants to
// match current behavior exactly; flip NAV_PIPELINE when navigation goes live.
constexpr bool AI_PIPELINE = true;
constexpr bool NAV_PIPELINE = false;

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
    // ── Parameters (names identical to serial_bridge.py) ──
    port_ = declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
    baudrate_ = static_cast<int>(declare_parameter<int>("serial_baudrate", 500000));
    reconnect_interval_ = declare_parameter<double>("serial_reconnect_interval", 2.0);
    rx_timeout_ = declare_parameter<double>("serial_rx_timeout", 3.0);
    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.3);
    use_framed_ = declare_parameter<bool>("use_framed_protocol", false);
    const double tx_hz = declare_parameter<double>("serial_tx_hz", 100.0);

    // New (C++ only). Defaults preserve the Python behavior:
    //   serial_parity: pyserial was opened with PARITY_EVEN — kept as default.
    //     Set to "none" from the launch file if the micro uses 8N1.
    //   low_latency: sets ASYNC_LOW_LATENCY on the tty (mainly helps FTDI-style
    //     adapters; harmless no-op on USB CDC-ACM). Off by default.
    //   thread_priority: >0 requests SCHED_FIFO at that priority for the spin
    //     thread. Needs rtprio rlimit or CAP_SYS_NICE; logs a warning if denied.
    parity_ = declare_parameter<std::string>("serial_parity", "even");
    low_latency_ = declare_parameter<bool>("low_latency", false);
    thread_priority_ = static_cast<int>(declare_parameter<int>("thread_priority", 0));

    rx_buf_.reserve(RX_BUFFER_CAP);

    sub_ai_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_AI", rclcpp::SystemDefaultsQoS(),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        if (!AI_PIPELINE) {return;}
        std::lock_guard<std::mutex> lk(mtx_);
        ai_shoot_ = msg->angular.x;   // shoot flag
        ai_pitch_ = msg->angular.y;   // absolute pitch target [rad]
        ai_yaw_ = msg->angular.z;     // absolute yaw target [rad]
        last_ai_time_ = mono_now();
      });

    sub_nav_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_NAV", rclcpp::SystemDefaultsQoS(),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        if (!NAV_PIPELINE) {return;}
        std::lock_guard<std::mutex> lk(mtx_);
        nav_x_ = msg->linear.x;
        nav_y_ = msg->linear.y;
        nav_angle_ = msg->angular.z;
      });

    pub_status_ = create_publisher<std_msgs::msg::Float32MultiArray>("/micro_status", 10);

    last_rx_time_ = mono_now();
    openSerial();

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(tx_hz, 1.0)),
      std::bind(&SerialBridgeNode::serialTick, this));
  }

  ~SerialBridgeNode() override
  {
    if (fd_ >= 0) {
      ::close(fd_);
      RCLCPP_INFO(get_logger(), "Serial port closed.");
    }
  }

  // Called from main() after construction, before spin.
  void applyThreadPriority()
  {
    if (thread_priority_ <= 0) {return;}
    sched_param sp{};
    sp.sched_priority = thread_priority_;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
      RCLCPP_WARN(
        get_logger(),
        "Could not set SCHED_FIFO priority %d (need rtprio rlimit or CAP_SYS_NICE) "
        "— continuing with default scheduling.", thread_priority_);
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

  // ── Serial open / reconnect ─────────────────────────────────────────────
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

    // Blocking open, but VMIN=0/VTIME=0 below makes read() non-blocking.
    // write() can only block if the kernel tx buffer fills, which at
    // 100 Hz * 31 B = 3.1 kB/s on a 500 kbaud link cannot happen.
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

    cfmakeraw(&tio);                       // raw 8-bit clean channel
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CSTOPB;                // 1 stop bit
    tio.c_cflag &= ~CRTSCTS;               // no HW flow control

    // Parity — pyserial used PARITY_EVEN; pyserial does not enable input
    // parity *checking* by default, so we generate parity on TX and ignore
    // parity errors on RX (IGNPAR) for identical behavior.
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

    tio.c_cc[VMIN] = 0;                    // non-blocking read
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
          RCLCPP_WARN(get_logger(), "ASYNC_LOW_LATENCY not supported on %s (ok on CDC-ACM).",
            port_.c_str());
        }
      }
    }

    tcflush(fd, TCIOFLUSH);                // flush stale data from a previous session

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

  // ── Timer: TX + RX drain ────────────────────────────────────────────────
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

    // --- Build TX packet ---
    float tx[TX_NUM_VALUES];
    {
      std::lock_guard<std::mutex> lk(mtx_);
      const bool ai_stale = (now - last_ai_time_) > cmd_timeout_;
      // SAFETY: force shoot=0 when /cmd_vel_AI is stale. Yaw/pitch are still
      // re-sent (gimbal holds position) but firing stops.
      tx[0] = static_cast<float>(
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start_).count());
      tx[1] = static_cast<float>(ai_yaw_);
      tx[2] = static_cast<float>(ai_pitch_);
      tx[3] = static_cast<float>(ai_stale ? 0.0 : ai_shoot_);
      tx[4] = static_cast<float>(nav_x_);
      tx[5] = static_cast<float>(nav_y_);
      tx[6] = static_cast<float>(nav_angle_);
    }

    uint8_t frame[2 + TX_PACKET_SIZE + 1];
    const uint8_t * out_ptr;
    size_t out_len;
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

    // --- Drain RX ---
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

  // ── RX parsing ──────────────────────────────────────────────────────────
  static bool sanityOk(const float * v, size_t n)
  {
    // Reject obviously corrupted packets (misalignment, reflash garbage).
    // !(|v| < 1e6) also rejects NaN and inf.
    for (size_t i = 0; i < n; i++) {
      if (!(std::fabs(v[i]) < 1e6f)) {return false;}
    }
    // Gimbal pitch is mechanically limited; outside ±2 rad is garbage.
    if (std::fabs(v[1]) > 2.0f) {return false;}
    return true;
  }

  void publishStatus(const float * rx)
  {
    std_msgs::msg::Float32MultiArray msg;
    msg.data.reserve(RX_NUM_VALUES + 6);
    msg.data.assign(rx, rx + RX_NUM_VALUES);
    {
      std::lock_guard<std::mutex> lk(mtx_);
      msg.data.push_back(static_cast<float>(ai_yaw_));
      msg.data.push_back(static_cast<float>(ai_pitch_));
      msg.data.push_back(static_cast<float>(ai_shoot_));
      msg.data.push_back(static_cast<float>(nav_x_));
      msg.data.push_back(static_cast<float>(nav_y_));
      msg.data.push_back(static_cast<float>(nav_angle_));
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
    // Original raw 40-byte packets, no header/CRC (current firmware).
    const int avail = bytesAvailable();
    if (avail < 0) {return false;}
    if (static_cast<size_t>(avail) < RX_PACKET_SIZE) {return true;}

    // Drain to the most recent complete packet so the autoaim node always
    // gets fresh gimbal angles.
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
      if (r == 0) {return true;}  // partial packet — leave for next tick
      got += static_cast<size_t>(r);
    }

    float vals[RX_NUM_VALUES];
    std::memcpy(vals, pkt, RX_PACKET_SIZE);
    if (sanityOk(vals, RX_NUM_VALUES)) {
      publishStatus(vals);
    } else {
      RCLCPP_WARN(get_logger(), "RX sanity check failed (garbage values) — flushing buffer");
      tcflush(fd_, TCIFLUSH);
    }
    return true;
  }

  bool drainRxFramed()
  {
    // Framed packets: header + payload + CRC8. Self-resyncing.
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

    // Cap buffer growth (link flooding / persistent desync).
    if (rx_buf_.size() > RX_BUFFER_CAP) {
      rx_buf_.erase(rx_buf_.begin(), rx_buf_.end() - RX_FRAME_SIZE);
    }

    float latest[RX_NUM_VALUES];
    bool have_latest = false;

    while (true) {
      auto it = std::search(
        rx_buf_.begin(), rx_buf_.end(), RX_FRAME_HEADER, RX_FRAME_HEADER + 2);
      if (it == rx_buf_.end()) {
        // No header: keep only the last byte (could be a split header).
        if (rx_buf_.size() > 1) {
          rx_buf_.erase(rx_buf_.begin(), rx_buf_.end() - 1);
        }
        break;
      }
      if (it != rx_buf_.begin()) {
        rx_buf_.erase(rx_buf_.begin(), it);
      }
      if (rx_buf_.size() < RX_FRAME_SIZE) {break;}  // incomplete — wait

      const uint8_t * payload = rx_buf_.data() + 2;
      const uint8_t rx_crc = rx_buf_[2 + RX_PACKET_SIZE];
      if (crc8(payload, RX_PACKET_SIZE) == rx_crc) {
        float vals[RX_NUM_VALUES];
        std::memcpy(vals, payload, RX_PACKET_SIZE);
        if (sanityOk(vals, RX_NUM_VALUES)) {
          std::memcpy(latest, vals, sizeof(vals));  // keep only the newest
          have_latest = true;
        }
        rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + RX_FRAME_SIZE);
      } else {
        rx_buf_.erase(rx_buf_.begin());  // bad CRC: drop one byte, rescan
      }
    }

    if (have_latest) {publishStatus(latest);}
    return true;
  }

  // ── Members ─────────────────────────────────────────────────────────────
  std::mutex mtx_;
  double ai_yaw_ = 0.0, ai_pitch_ = 0.0, ai_shoot_ = 0.0;
  double nav_x_ = 0.0, nav_y_ = 0.0, nav_angle_ = 0.0;
  double last_ai_time_ = 0.0;

  std::string port_, parity_;
  int baudrate_ = 500000;
  double reconnect_interval_ = 2.0, rx_timeout_ = 3.0, cmd_timeout_ = 0.3;
  bool use_framed_ = false, low_latency_ = false;
  int thread_priority_ = 0;

  int fd_ = -1;
  double last_rx_time_ = 0.0, last_reconnect_attempt_ = 0.0;
  int consecutive_errors_ = 0;
  std::vector<uint8_t> rx_buf_;
  std::chrono::steady_clock::time_point t_start_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_ai_, sub_nav_;
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
