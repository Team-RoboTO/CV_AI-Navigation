#include "autoaim_v2/serial_port.hpp"

#include <fcntl.h>
#include <linux/serial.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cstring>

namespace aim
{
namespace
{
speed_t to_speed(int baud)
{
  switch (baud) {
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 921600: return B921600;
    case 1000000: return B1000000;
    case 2000000: return B2000000;
    default: return B500000;
  }
}
}  // namespace

SerialPort::~SerialPort() { close(); }

bool SerialPort::open(const std::string & device, int baud,
                      const std::string & parity, bool low_latency)
{
  close();
  int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY);
  if (fd < 0) return false;

  termios tio{};
  if (tcgetattr(fd, &tio) != 0) {
    ::close(fd);
    return false;
  }
  cfmakeraw(&tio);
  cfsetispeed(&tio, to_speed(baud));
  cfsetospeed(&tio, to_speed(baud));
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~CSTOPB;
  tio.c_cflag &= ~CRTSCTS;
  if (parity == "even") {
    tio.c_cflag |= PARENB;
    tio.c_cflag &= ~PARODD;
    tio.c_iflag |= IGNPAR;
  } else if (parity == "odd") {
    tio.c_cflag |= PARENB | PARODD;
    tio.c_iflag |= IGNPAR;
  } else {
    tio.c_cflag &= ~PARENB;
  }
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  if (tcsetattr(fd, TCSANOW, &tio) != 0) {
    ::close(fd);
    return false;
  }

  if (low_latency) {
    serial_struct ss{};
    if (ioctl(fd, TIOCGSERIAL, &ss) == 0) {
      ss.flags |= ASYNC_LOW_LATENCY;
      ioctl(fd, TIOCSSERIAL, &ss);  // best effort; CDC-ACM often lacks it
    }
  }

  tcflush(fd, TCIOFLUSH);
  fd_ = fd;
  return true;
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

int SerialPort::write_bytes(const uint8_t * data, size_t len)
{
  if (fd_ < 0) return -1;
  ssize_t n = ::write(fd_, data, len);
  return static_cast<int>(n);
}

int SerialPort::read_bytes(uint8_t * data, size_t maxlen, int timeout_ms)
{
  if (fd_ < 0) return -1;
  pollfd pfd{fd_, POLLIN, 0};
  int pr = ::poll(&pfd, 1, timeout_ms);
  if (pr < 0) return -1;
  if (pr == 0) return 0;
  ssize_t n = ::read(fd_, data, maxlen);
  if (n < 0) return -1;
  return static_cast<int>(n);
}

}  // namespace aim
