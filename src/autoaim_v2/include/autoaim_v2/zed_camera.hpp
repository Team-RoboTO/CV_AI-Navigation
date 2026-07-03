#ifndef AUTOAIM_V2__ZED_CAMERA_HPP_
#define AUTOAIM_V2__ZED_CAMERA_HPP_

#include <cstdint>
#include <memory>
#include <string>

#include "autoaim_v2/types.hpp"

namespace aim
{

struct ZedParams
{
  std::string resolution = "SVGA";  // VGA | SVGA | HD720 | HD1080 | HD1200
  int fps = 120;
  bool image_flip = true;           // camera mounted upside-down
  // Exposure: on ZED X (GMSL) EXPOSURE_TIME is microseconds. Short exposure
  // kills motion blur on spinners; raise gain to compensate.
  int exposure_time_us = 3000;      // <= frame period; -1 -> auto exposure
  int gain = 80;                    // 0..100, manual mode only
  bool auto_white_balance = true;
};

// Thin ZED SDK wrapper: grab loop support, GPU BGRA left image, capture
// timestamps mapped to the steady clock, rectified intrinsics.
// Requires ZED SDK >= 4.0 (uses the CUDA primary context).
class ZedCamera
{
public:
  explicit ZedCamera(const ZedParams & p);
  ~ZedCamera();

  bool open();
  void close();
  bool is_open() const;
  const std::string & error() const { return error_; }

  int width() const { return width_; }
  int height() const { return height_; }
  double fx() const { return fx_; }
  double fy() const { return fy_; }
  double cx() const { return cx_; }
  double cy() const { return cy_; }

  // Blocking grab. On success returns a device BGRA pointer valid until the
  // next grab, its pitch in bytes, and the capture time on the steady clock.
  bool grab(const uint8_t ** d_bgra, size_t * pitch, TimePoint * t_capture);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  ZedParams p_;
  std::string error_;
  int width_ = 0, height_ = 0;
  double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;
};

}  // namespace aim

#endif  // AUTOAIM_V2__ZED_CAMERA_HPP_
