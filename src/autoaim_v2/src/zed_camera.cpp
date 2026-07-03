#include "autoaim_v2/zed_camera.hpp"

#include <sl/Camera.hpp>

#include <map>

namespace aim
{

struct ZedCamera::Impl
{
  sl::Camera zed;
  sl::Mat left_gpu;
  bool opened = false;
};

ZedCamera::ZedCamera(const ZedParams & p) : impl_(new Impl), p_(p) {}

ZedCamera::~ZedCamera() { close(); }

bool ZedCamera::is_open() const { return impl_->opened; }

bool ZedCamera::open()
{
  static const std::map<std::string, sl::RESOLUTION> res_map = {
    {"VGA", sl::RESOLUTION::VGA},       {"SVGA", sl::RESOLUTION::SVGA},
    {"HD720", sl::RESOLUTION::HD720},   {"HD1080", sl::RESOLUTION::HD1080},
    {"HD1200", sl::RESOLUTION::HD1200},
  };
  auto it = res_map.find(p_.resolution);
  if (it == res_map.end()) {
    error_ = "unknown ZED resolution: " + p_.resolution;
    return false;
  }

  sl::InitParameters init;
  init.camera_resolution = it->second;
  init.camera_fps = p_.fps;
  init.depth_mode = sl::DEPTH_MODE::NONE;
  init.camera_image_flip = p_.image_flip ? sl::FLIP_MODE::ON : sl::FLIP_MODE::OFF;
  init.sdk_verbose = 0;

  auto err = impl_->zed.open(init);
  if (err != sl::ERROR_CODE::SUCCESS) {
    error_ = std::string("ZED open failed: ") + sl::toString(err).c_str();
    return false;
  }

  // Exposure control. ZED X / X Mini (GMSL): EXPOSURE_TIME in microseconds.
  // Short exposure = less motion blur on fast spinners; compensate with gain.
  if (p_.exposure_time_us > 0) {
    impl_->zed.setCameraSettings(sl::VIDEO_SETTINGS::AEC_AGC, 0);
    auto e = impl_->zed.setCameraSettings(sl::VIDEO_SETTINGS::EXPOSURE_TIME,
                                          p_.exposure_time_us);
    if (e != sl::ERROR_CODE::SUCCESS) {
      // USB models: EXPOSURE_TIME unsupported -> approximate with EXPOSURE %.
      const double frame_us = 1e6 / std::max(p_.fps, 1);
      int pct = static_cast<int>(100.0 * p_.exposure_time_us / frame_us);
      impl_->zed.setCameraSettings(sl::VIDEO_SETTINGS::EXPOSURE,
                                   std::max(1, std::min(100, pct)));
    }
    impl_->zed.setCameraSettings(sl::VIDEO_SETTINGS::GAIN, p_.gain);
  } else {
    impl_->zed.setCameraSettings(sl::VIDEO_SETTINGS::AEC_AGC, 1);
  }
  impl_->zed.setCameraSettings(sl::VIDEO_SETTINGS::WHITEBALANCE_AUTO,
                               p_.auto_white_balance ? 1 : 0);

  const auto conf = impl_->zed.getCameraInformation().camera_configuration;
  const auto & cal = conf.calibration_parameters.left_cam;
  width_ = static_cast<int>(conf.resolution.width);
  height_ = static_cast<int>(conf.resolution.height);
  fx_ = cal.fx;
  fy_ = cal.fy;
  cx_ = cal.cx;
  cy_ = cal.cy;

  impl_->opened = true;
  return true;
}

void ZedCamera::close()
{
  if (impl_ && impl_->opened) {
    impl_->left_gpu.free();
    impl_->zed.close();
    impl_->opened = false;
  }
}

bool ZedCamera::grab(const uint8_t ** d_bgra, size_t * pitch, TimePoint * t_capture)
{
  if (!impl_->opened) return false;
  if (impl_->zed.grab() != sl::ERROR_CODE::SUCCESS) return false;

  // Capture time: the SDK stamp is on the system clock; map it onto the
  // steady clock using the offset sampled right now (sub-ms accurate, and
  // immune to NTP steps unlike using the system clock everywhere).
  const uint64_t t_img_ns =
    impl_->zed.getTimestamp(sl::TIME_REFERENCE::IMAGE).getNanoseconds();
  const uint64_t t_sys_now_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch())
      .count();
  const TimePoint t_steady_now = Clock::now();
  const int64_t age_ns =
    static_cast<int64_t>(t_sys_now_ns) - static_cast<int64_t>(t_img_ns);
  *t_capture = t_steady_now - std::chrono::nanoseconds(std::max<int64_t>(age_ns, 0));

  if (impl_->zed.retrieveImage(impl_->left_gpu, sl::VIEW::LEFT, sl::MEM::GPU) !=
      sl::ERROR_CODE::SUCCESS) {
    return false;
  }
  *d_bgra = impl_->left_gpu.getPtr<sl::uchar1>(sl::MEM::GPU);
  *pitch = impl_->left_gpu.getStepBytes(sl::MEM::GPU);
  return true;
}

}  // namespace aim
