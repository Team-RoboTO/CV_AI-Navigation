#include "autoaim_v2/rs_camera.hpp"

#include <librealsense2/rs.hpp>

namespace aim
{

struct RsCamera::Impl
{
  rs2::pipeline pipe;
  rs2::frameset last_fs;  // keeps frame data alive until next grab()
  bool opened = false;
};

RsCamera::RsCamera(const RsParams & p) : impl_(new Impl), p_(p) {}

RsCamera::~RsCamera() { close(); }

bool RsCamera::is_open() const { return impl_->opened; }

bool RsCamera::open()
{
  rs2::config cfg;
  if (!p_.serial_no.empty()) cfg.enable_device(p_.serial_no);
  const rs2_format fmt = p_.use_yuyv ? RS2_FORMAT_YUYV : RS2_FORMAT_BGR8;
  cfg.enable_stream(RS2_STREAM_COLOR, p_.width, p_.height, fmt, p_.fps);

  rs2::pipeline_profile profile;
  try {
    profile = impl_->pipe.start(cfg);
  } catch (const rs2::error & e) {
    error_ = std::string("RealSense pipe.start: ") + e.what();
    return false;
  }

  try {
    rs2::color_sensor cs = profile.get_device().first<rs2::color_sensor>();
    if (cs.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE))
      cs.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, p_.auto_exposure ? 1.f : 0.f);
    if (cs.supports(RS2_OPTION_AUTO_EXPOSURE_PRIORITY))
      cs.set_option(RS2_OPTION_AUTO_EXPOSURE_PRIORITY, 0.f);
    if (!p_.auto_exposure) {
      if (cs.supports(RS2_OPTION_EXPOSURE))
        cs.set_option(RS2_OPTION_EXPOSURE, static_cast<float>(p_.exposure));
      if (cs.supports(RS2_OPTION_GAIN))
        cs.set_option(RS2_OPTION_GAIN, static_cast<float>(p_.gain));
    }
  } catch (const rs2::error &) {
    // best-effort — some models lack these options
  }

  auto vsp = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
  rs2_intrinsics intr = vsp.get_intrinsics();
  width_ = intr.width;
  height_ = intr.height;

  fx_ = intr.fx;
  fy_ = intr.fy;
  cx_ = intr.ppx;
  cy_ = intr.ppy;
  if (p_.flip_180) {
    cx_ = (width_ - 1) - intr.ppx;
    cy_ = (height_ - 1) - intr.ppy;
  }

  impl_->opened = true;
  return true;
}

void RsCamera::close()
{
  if (impl_ && impl_->opened) {
    try { impl_->pipe.stop(); } catch (...) {}
    impl_->opened = false;
  }
}

bool RsCamera::grab(const uint8_t ** h_data, size_t * data_bytes,
                     TimePoint * t_capture)
{
  if (!impl_->opened) return false;

  try {
    impl_->last_fs = impl_->pipe.wait_for_frames(1000);
  } catch (const rs2::error &) {
    return false;
  }
  rs2::video_frame color = impl_->last_fs.get_color_frame();
  if (!color) return false;

  const rs2_timestamp_domain dom = color.get_frame_timestamp_domain();
  if (dom == RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME || dom == RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME) {
    const auto sys_now = std::chrono::system_clock::now();
    const TimePoint steady_now = Clock::now();
    const int64_t stamp_ns = static_cast<int64_t>(color.get_timestamp() * 1e6);
    const int64_t sys_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                             sys_now.time_since_epoch()).count();
    const int64_t age = std::max<int64_t>(sys_ns - stamp_ns, 0);
    *t_capture = steady_now - std::chrono::nanoseconds(
                   std::min<int64_t>(age, 250000000LL));
  } else {
    *t_capture = Clock::now();
  }

  const int bpp = p_.use_yuyv ? 2 : 3;
  *data_bytes = static_cast<size_t>(width_) * height_ * bpp;
  *h_data = static_cast<const uint8_t *>(color.get_data());
  return true;
}

}  // namespace aim
