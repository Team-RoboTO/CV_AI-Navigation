#ifndef AUTOAIM_V2__PROTOCOL_HPP_
#define AUTOAIM_V2__PROTOCOL_HPP_

// Byte-exact legacy protocol of serial_bridge.cpp / MiniPC.c. Do not change
// without changing the firmware: the micro memcpy's these floats directly.
//
// TX (Jetson -> micro): 7 x float32 LE = 28 bytes, no header, no CRC.
//   [0] timestamp [s]  — MUST change every packet: the firmware only latches
//       yaw/pitch when this value differs from the previous packet.
//   [1] turret yaw absolute target, micro frame [rad, unbounded]
//   [2] turret pitch target [rad, micro convention]
//   [3] shoot flag (1 = feeder may fire at its configured frequency)
//   [4] nav_x — unused on standard; optional yaw-rate feedforward [rad/s]
//   [5] nav_y — unused on standard; optional pitch-rate feedforward [rad/s]
//   [6] chassis autospin flag
//
// RX (micro -> Jetson): 10 x float32 LE = 40 bytes at ~1 kHz.
//   [0] yaw [rad, unbounded]   [1] pitch [rad]
//   [2] vx [m/s]               [3] vy [m/s]
//   [4] color 0=RED 1=BLUE     [5] game_progress (4 = in match)
//   [6] HP                     [7] resupply flag
//   [8] center flag            [9] reserved (fw-defined)

#include <cstdint>
#include <cstring>
#include <cmath>

namespace aim::protocol
{

constexpr size_t TX_FLOATS = 7;
constexpr size_t TX_BYTES = TX_FLOATS * 4;
constexpr size_t RX_FLOATS = 10;
constexpr size_t RX_BYTES = RX_FLOATS * 4;

struct TxPacket
{
  float t = 0;
  float yaw = 0;
  float pitch = 0;
  float shoot = 0;
  float nav_x = 0;
  float nav_y = 0;
  float rot = 0;
};
static_assert(sizeof(TxPacket) == TX_BYTES, "TxPacket must be packed 28 bytes");

inline void pack(const TxPacket & p, uint8_t out[TX_BYTES])
{
  std::memcpy(out, &p, TX_BYTES);
}

// Parse one RX packet. Returns false when the values fail the same sanity
// rules the old bridge used (|v| < 1e6, |pitch| < 2 rad) — a failed check
// means we are byte-shifted and the caller should resynchronize.
inline bool parse_rx(const uint8_t in[RX_BYTES], float out[RX_FLOATS])
{
  std::memcpy(out, in, RX_BYTES);
  for (size_t i = 0; i < RX_FLOATS; i++) {
    if (!std::isfinite(out[i]) || !(std::fabs(out[i]) < 1e6f)) return false;
  }
  if (std::fabs(out[1]) > 2.0f) return false;
  return true;
}

}  // namespace aim::protocol

#endif  // AUTOAIM_V2__PROTOCOL_HPP_
