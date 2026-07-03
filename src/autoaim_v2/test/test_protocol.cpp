#include <gtest/gtest.h>

#include <cstring>

#include "autoaim_v2/protocol.hpp"

using namespace aim::protocol;

TEST(Protocol, TxLayoutIsLegacy28Bytes)
{
  TxPacket p;
  p.t = 1.5f;
  p.yaw = 34.25f;
  p.pitch = -0.12f;
  p.shoot = 1.f;
  p.nav_x = 0.f;
  p.nav_y = 0.f;
  p.rot = 1.f;

  uint8_t bytes[TX_BYTES];
  pack(p, bytes);

  float v[TX_FLOATS];
  std::memcpy(v, bytes, TX_BYTES);
  EXPECT_FLOAT_EQ(v[0], 1.5f);    // timestamp
  EXPECT_FLOAT_EQ(v[1], 34.25f);  // yaw
  EXPECT_FLOAT_EQ(v[2], -0.12f);  // pitch
  EXPECT_FLOAT_EQ(v[3], 1.f);     // shoot
  EXPECT_FLOAT_EQ(v[6], 1.f);     // rot
}

TEST(Protocol, RxParseAndSanity)
{
  float src[RX_FLOATS] = {12.3f, -0.2f, 0.5f, -0.5f, 1.f, 4.f, 400.f, 0.f, 0.f, 0.f};
  uint8_t bytes[RX_BYTES];
  std::memcpy(bytes, src, RX_BYTES);

  float out[RX_FLOATS];
  ASSERT_TRUE(parse_rx(bytes, out));
  EXPECT_FLOAT_EQ(out[0], 12.3f);
  EXPECT_FLOAT_EQ(out[6], 400.f);

  // Pitch out of range -> reject (byte-shift detector).
  src[1] = 3.0f;
  std::memcpy(bytes, src, RX_BYTES);
  EXPECT_FALSE(parse_rx(bytes, out));

  // NaN -> reject.
  src[1] = -0.2f;
  src[5] = NAN;
  std::memcpy(bytes, src, RX_BYTES);
  EXPECT_FALSE(parse_rx(bytes, out));
}
