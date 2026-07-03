#ifndef AUTOAIM_V2__SERIAL_PORT_HPP_
#define AUTOAIM_V2__SERIAL_PORT_HPP_

#include <cstdint>
#include <string>

namespace aim
{

// Minimal CDC-ACM serial wrapper (USB: baud/parity are cosmetic, kept for
// compatibility with UART bridges). Blocking reads with a poll timeout so the
// RX thread can exit cleanly.
class SerialPort
{
public:
  ~SerialPort();

  bool open(const std::string & device, int baud = 500000,
            const std::string & parity = "even", bool low_latency = true);
  void close();
  bool is_open() const { return fd_ >= 0; }

  // Returns bytes written or -1.
  int write_bytes(const uint8_t * data, size_t len);
  // Blocks up to timeout_ms; returns bytes read (0 on timeout) or -1 on error.
  int read_bytes(uint8_t * data, size_t maxlen, int timeout_ms);

private:
  int fd_ = -1;
};

}  // namespace aim

#endif  // AUTOAIM_V2__SERIAL_PORT_HPP_
