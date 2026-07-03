#ifndef AUTOAIM_V2__RS_CAMERA_HPP_
#define AUTOAIM_V2__RS_CAMERA_HPP_

#include <cstdint>
#include <memory>
#include <string>

#include "autoaim_v2/types.hpp"

namespace aim
{

struct RsParams
{
  std::string serial_no = "";   // "" = first connected device
  int width = 640;
  int height = 360;
  int fps = 90;
  bool auto_exposure = false;
  int exposure = 60;            // RS2_OPTION_EXPOSURE (microseconds)
  int gain = 50;
  bool use_yuyv = true;         // false = BGR8 from librealsense
  bool flip_180 = true;         // upside-down mount
};

// librealsense2 camera wrapper: COLOR stream only (no depth/IR),
// CPU-side frame data, capture timestamps on the steady clock.
class RsCamera
{
public:
  explicit RsCamera(const RsParams & p);
  ~RsCamera();

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
  bool yuyv() const { return p_.use_yuyv; }
  bool flip() const { return p_.flip_180; }

  // Blocking grab. On success returns a HOST pointer to the color data
  // (BGR8: 3 bpp; YUYV: 2 bpp), its size in bytes, and the capture time
  // on the steady clock. Pointer valid until next grab().
  bool grab(const uint8_t ** h_data, size_t * data_bytes, TimePoint * t_capture);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  RsParams p_;
  std::string error_;
  int width_ = 0, height_ = 0;
  double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;
};

}  // namespace aim

#endif  // AUTOAIM_V2__RS_CAMERA_HPP_
