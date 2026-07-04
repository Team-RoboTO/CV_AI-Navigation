// High-performance RealSense + TensorRT YOLO-pose detector (competition path).
//
// This is the C++ twin of zed_detector.py. It uses librealsense2 DIRECTLY (no
// realsense2_camera ROS node, no image-topic round trip) and the COLOR stream
// only, runs the SAME TensorRT engine through a CUDA letterbox preprocess, and
// publishes the SAME functional topics / message types that autoaim consumes:
//
//   /detector/armors               vision_msgs/Detection2DArray  (legacy bbox)
//   /detector/armors_keypoints     autoaim/ArmorKeypointArray    (tracking input)
//   /camera_info                   sensor_msgs/CameraInfo        (color intrinsics)
//
// Debug outputs are opt-in safe-by-default:
//
//   /detector/armors_keypoints_json std_msgs/String              (debug)
//   /yolo/debug_image              sensor_msgs/Image             (throttled debug)
//   /camera/image_raw/compressed   sensor_msgs/CompressedImage   (throttled JPEG,
//                                  clean frame for dataset recording)
//
// Tracking never reads the JSON string; it consumes the typed
// ArmorKeypointArray above. Keeping debug publishers off by default makes a
// normal launch a competition launch while still allowing per-output diagnosis.
//
// The class-id convention (0 blue, 1 grey, 2 red) and keypoint order
// (TL,TR,BR,BL) are identical to the ZED path — autoaim does not need to know
// which camera produced the detections.
//
// Decode/preprocess logic is a faithful port of zed_detector.py (raw YOLO-pose
// and post-NMS/end2end output layouts auto-detected from the engine).

#include <algorithm>
#include <atomic>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <cuda_runtime.h>
#include <NvInfer.h>
#include <NvInferPlugin.h>
#include <librealsense2/rs.hpp>
#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgcodecs.hpp>  // imencode (JPEG dataset-recording image)
#include <opencv2/imgproc.hpp>  // cvtColor (YUYV->BGR debug image)

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "autoaim/msg/armor_keypoint.hpp"
#include "autoaim/msg/armor_keypoint_array.hpp"
#include "autoaim_realsense/realsense_preprocess.h"

namespace {

// ── Model / dataset convention (TRUE invariants — must match the ZED path) ──
constexpr int kNumKeypoints = 4;
const std::array<const char*, kNumKeypoints> kKeypointNames = {"TL", "TR", "BR", "BL"};
const char* class_name(int id) {
  switch (id) {
    case 0: return "blue_armor";
    case 1: return "grey_armor";
    case 2: return "red_armor";
    default: return "unknown";
  }
}
constexpr int kNumClassesPostNms = 3;  // post-NMS width does not encode class count

// Decode-stage sizing. Algorithm invariants, not field-tuning values.
constexpr int kMaxDetections = 30;
constexpr float kKeypointScoreThreshold = 0.05f;

#define CUDA_CHECK(call)                                                       \
  do {                                                                         \
    cudaError_t _e = (call);                                                   \
    if (_e != cudaSuccess) {                                                   \
      throw std::runtime_error(std::string("CUDA error: ") +                   \
                               cudaGetErrorString(_e) + " @ " #call);          \
    }                                                                          \
  } while (0)

// Runtime ROS log gate. Launch files default enable_ros_logs=false in
// competition; these wrappers avoid entering RCLCPP_* macros at all when the
// flag is off, while keeping the original log call sites maintainable.
#define AA_RCLCPP_LOG_IF(enabled, macro_name, ...) \
  do { \
    if (enabled) { \
      macro_name(__VA_ARGS__); \
    } \
  } while (0)
#define AA_RCLCPP_DEBUG(enabled, ...) AA_RCLCPP_LOG_IF(enabled, RCLCPP_DEBUG, __VA_ARGS__)
#define AA_RCLCPP_INFO(enabled, ...) AA_RCLCPP_LOG_IF(enabled, RCLCPP_INFO, __VA_ARGS__)
#define AA_RCLCPP_WARN(enabled, ...) AA_RCLCPP_LOG_IF(enabled, RCLCPP_WARN, __VA_ARGS__)
#define AA_RCLCPP_ERROR(enabled, ...) AA_RCLCPP_LOG_IF(enabled, RCLCPP_ERROR, __VA_ARGS__)
#define AA_RCLCPP_FATAL(enabled, ...) AA_RCLCPP_LOG_IF(enabled, RCLCPP_FATAL, __VA_ARGS__)
#define AA_RCLCPP_INFO_THROTTLE(enabled, ...) AA_RCLCPP_LOG_IF(enabled, RCLCPP_INFO_THROTTLE, __VA_ARGS__)
#define AA_RCLCPP_WARN_THROTTLE(enabled, ...) AA_RCLCPP_LOG_IF(enabled, RCLCPP_WARN_THROTTLE, __VA_ARGS__)

std::atomic_bool g_realsense_ros_logs_enabled{false};

// Minimal TensorRT logger (warnings and above, like the Python detector).
class TrtLogger : public nvinfer1::ILogger {
 public:
  void log(Severity severity, const char* msg) noexcept override {
    if (severity <= Severity::kWARNING) {
      AA_RCLCPP_WARN(g_realsense_ros_logs_enabled.load(std::memory_order_relaxed), rclcpp::get_logger("realsense_detector.trt"), "%s", msg);
    }
  }
};

template <typename T>
struct TrtDeleter {
  void operator()(T* p) const { delete p; }  // TRT 10: objects deleted with delete
};
template <typename T>
using TrtPtr = std::unique_ptr<T, TrtDeleter<T>>;

std::string resolve_engine_path(const std::string& param_value) {
  if (!param_value.empty()) return param_value;
  const char* env = std::getenv("AUTOAIM_ENGINE_PATH");
  if (env && env[0] != '\0') return env;
  return "/workspaces/isaac_ros-dev/AI-models/yolov26_keypoints.engine";
}

// One decoded armor in native color-image pixels.
struct Armor {
  int cls;
  float conf;
  float L, T, W, H;                              // bbox left/top/width/height
  std::array<float, kNumKeypoints * 3> kp;       // x,y,score per keypoint
};

}  // namespace

class RealSenseDetector : public rclcpp::Node {
 public:
  RealSenseDetector() : rclcpp::Node("realsense_detector") {
    // ── Parameters (see config/realsense.yaml) ──
    engine_path_ = resolve_engine_path(declare_parameter<std::string>("engine_path", ""));
    threshold_ = static_cast<float>(declare_parameter<double>("threshold", 0.15));
    nms_iou_ = static_cast<float>(declare_parameter<double>("nms_iou", 0.2));
    // Runtime log gate. False skips AA_RCLCPP_* wrappers entirely while the
    // detector still publishes functional bbox/keypoint/CameraInfo topics.
    enable_ros_logs_ = declare_parameter<bool>("enable_ros_logs", false);
    g_realsense_ros_logs_enabled.store(enable_ros_logs_, std::memory_order_relaxed);
    serial_no_ = declare_parameter<std::string>("serial_no", "");
    req_width_ = declare_parameter<int>("width", 640);
    req_height_ = declare_parameter<int>("height", 480);
    req_fps_ = declare_parameter<int>("fps", 60);
    auto_exposure_ = declare_parameter<bool>("auto_exposure", false);
    // exposure (microseconds) and gain are integer-valued RealSense options;
    // declared int so the natural YAML form (e.g. exposure: 6000) loads.
    exposure_ = declare_parameter<int>("exposure", 6000);
    gain_ = declare_parameter<int>("gain", 64);
    // Color pixel format: "bgr8" (default; librealsense converts YUYV->BGR on
    // the host CPU) or "yuyv" (raw sensor format; this node converts YUYV->RGB
    // on the GPU instead, saving CPU). USB bandwidth is identical either way.
    pixel_format_ = declare_parameter<std::string>("pixel_format", "bgr8");
    use_yuyv_ = (pixel_format_ == "yuyv" || pixel_format_ == "YUYV");
    color_bpp_ = use_yuyv_ ? 2 : 3;
    // Detector consumes the COLOR (RGB) sensor only. These gate opening the
    // extra streams; all default false (no stereo IR, no IMU, no depth).
    enable_depth_ = declare_parameter<bool>("enable_depth", false);
    enable_infrared_ = declare_parameter<bool>("enable_infrared", false);
    enable_imu_ = declare_parameter<bool>("enable_imu", false);
    // Rotate the color image 180 deg for an upside-down camera mount.
    flip_180_ = declare_parameter<bool>("flip_180", false);
    // Debug image cadence: 0 means no debug-image publisher at all. The typed
    // keypoint and bbox topics below are the functional detector path and are
    // never gated by this safe-by-default debug knob.
    publish_debug_every_ = declare_parameter<int>("publish_debug_every", 0);
    // Dataset-recording cadence: 0 means no recording publisher at all. Unlike
    // the debug image this is the CLEAN camera frame (no overlays), published
    // JPEG-compressed so an autostart rosbag can record training images
    // without eating disk (raw 640x360 BGR is ~0.7 MB/frame, JPEG q85 ~25 KB).
    publish_record_every_ = declare_parameter<int>("publish_record_every", 0);
    record_jpeg_quality_ = declare_parameter<int>("record_jpeg_quality", 85);
    // JSON is a viewer/echo aid only. The tracker reads the typed
    // /detector/armors_keypoints message, never this string, so disabling it
    // cannot break detection -> tracking -> command -> serial -> fire.
    publish_json_ = declare_parameter<bool>("publish_json", false);
    camera_info_every_ = declare_parameter<int>("camera_info_every", 1);
    frame_id_ = declare_parameter<std::string>("frame_id", "camera_color_optical_frame");

    auto qos = rclcpp::SensorDataQoS();
    det_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>("/detector/armors", qos);
    kpt_pub_ = create_publisher<autoaim::msg::ArmorKeypointArray>("/detector/armors_keypoints", qos);
    if (publish_json_) {
      json_pub_ = create_publisher<std_msgs::msg::String>("/detector/armors_keypoints_json", qos);
    }
    if (publish_debug_every_ > 0) {
      img_pub_ = create_publisher<sensor_msgs::msg::Image>("/yolo/debug_image", 1);
    }
    if (publish_record_every_ > 0) {
      // Reliable keep-last so the rosbag recorder does not silently drop
      // frames; messages are small JPEGs so reliability costs nothing here.
      rec_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
          "/camera/image_raw/compressed", rclcpp::QoS(10).reliable());
    }
    // Camera intrinsics are static and published sparsely (camera_info_every).
    // Latch them (transient-local + reliable) so a late-joining or respawned
    // autoaim node receives the last CameraInfo immediately, instead of waiting
    // up to camera_info_every frames for the next one.
    auto cam_info_qos = rclcpp::QoS(1).reliable().transient_local();
    cam_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("/camera_info", cam_info_qos);

    initRealSense();   // opens color stream, fills native_w/h_, cam_info_msg_
    initTensorRT();    // deserializes engine, parses IO shapes
    allocateBuffers(); // device/pinned buffers + letterbox geometry + stream

    running_ = true;
    worker_ = std::thread(&RealSenseDetector::captureLoop, this);
    AA_RCLCPP_INFO(enable_ros_logs_, get_logger(),
                "RealSense detector started: %dx%d@%d, engine=%s",
                native_w_, native_h_, req_fps_, engine_path_.c_str());
  }

  ~RealSenseDetector() override {
    running_ = false;
    if (worker_.joinable()) worker_.join();
    try { pipe_.stop(); } catch (...) {}
    if (stream_) cudaStreamDestroy(stream_);
    if (d_in_) cudaFree(d_in_);
    if (d_out_) cudaFree(d_out_);
    if (d_color_) cudaFree(d_color_);
    if (h_out_) cudaFreeHost(h_out_);
  }

 private:
  // ─────────────────────────────────────────────────────────────────────────
  void initRealSense() {
    rs2::config cfg;
    if (!serial_no_.empty()) cfg.enable_device(serial_no_);
    const rs2_format color_fmt = use_yuyv_ ? RS2_FORMAT_YUYV : RS2_FORMAT_BGR8;
    cfg.enable_stream(RS2_STREAM_COLOR, req_width_, req_height_, color_fmt, req_fps_);
    // Extra streams are opened only when requested; the detector never reads
    // them. Default OFF -> the camera runs as a single RGB sensor.
    if (enable_depth_) {
      cfg.enable_stream(RS2_STREAM_DEPTH, req_width_, req_height_, RS2_FORMAT_Z16, req_fps_);
    }
    if (enable_infrared_) {
      // Stereo IR pair (left=1, right=2).
      cfg.enable_stream(RS2_STREAM_INFRARED, 1, req_width_, req_height_, RS2_FORMAT_Y8, req_fps_);
      cfg.enable_stream(RS2_STREAM_INFRARED, 2, req_width_, req_height_, RS2_FORMAT_Y8, req_fps_);
    }
    if (enable_imu_) {
      // Motion streams run at their own native rates (not req_fps_).
      cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
      cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
    }
    rs2::pipeline_profile profile = pipe_.start(cfg);

    // Apply exposure/gain on the color sensor (best-effort; options may be
    // unsupported on some models/firmwares).
    try {
      rs2::color_sensor cs = profile.get_device().first<rs2::color_sensor>();
      if (cs.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
        cs.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, auto_exposure_ ? 1.f : 0.f);
      }
      // Lock the framerate: with priority OFF the auto-exposure algorithm may
      // not lengthen exposure past the frame interval, so the color stream
      // holds the requested fps (e.g. 90) instead of dropping in dim light.
      if (cs.supports(RS2_OPTION_AUTO_EXPOSURE_PRIORITY)) {
        cs.set_option(RS2_OPTION_AUTO_EXPOSURE_PRIORITY, 0.f);
      }
      if (!auto_exposure_) {
        if (cs.supports(RS2_OPTION_EXPOSURE)) {
          cs.set_option(RS2_OPTION_EXPOSURE, static_cast<float>(exposure_));
        }
        if (cs.supports(RS2_OPTION_GAIN)) {
          cs.set_option(RS2_OPTION_GAIN, static_cast<float>(gain_));
        }
      }
    } catch (const rs2::error& e) {
      AA_RCLCPP_WARN(enable_ros_logs_, get_logger(), "RealSense option set failed: %s", e.what());
    }

    // Color intrinsics -> cached CameraInfo (intrinsics do not change at runtime).
    auto vsp = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    rs2_intrinsics intr = vsp.get_intrinsics();
    native_w_ = intr.width;
    native_h_ = intr.height;

    // A 180 deg image flip moves the principal point to (W-1-cx, H-1-cy);
    // focal lengths are unchanged. Keeps CameraInfo consistent with flip_180_.
    double cx = intr.ppx, cy = intr.ppy;
    if (flip_180_) {
      cx = (native_w_ - 1) - intr.ppx;
      cy = (native_h_ - 1) - intr.ppy;
    }

    cam_info_msg_.header.frame_id = frame_id_;
    cam_info_msg_.width = static_cast<uint32_t>(native_w_);
    cam_info_msg_.height = static_cast<uint32_t>(native_h_);
    cam_info_msg_.k = {intr.fx, 0.0, cx,
                       0.0, intr.fy, cy,
                       0.0, 0.0, 1.0};
    cam_info_msg_.d.assign(intr.coeffs, intr.coeffs + 5);
    switch (intr.model) {
      case RS2_DISTORTION_KANNALA_BRANDT4:
        cam_info_msg_.distortion_model = "equidistant";
        break;
      case RS2_DISTORTION_NONE:
      case RS2_DISTORTION_BROWN_CONRADY:
      case RS2_DISTORTION_MODIFIED_BROWN_CONRADY:
      case RS2_DISTORTION_INVERSE_BROWN_CONRADY:
      default:
        cam_info_msg_.distortion_model = "plumb_bob";
        break;
    }
    AA_RCLCPP_INFO(enable_ros_logs_, get_logger(),
                "RealSense color intrinsics %dx%d fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
                native_w_, native_h_, intr.fx, intr.fy, intr.ppx, intr.ppy);
  }

  // ─────────────────────────────────────────────────────────────────────────
  void initTensorRT() {
    std::ifstream f(engine_path_, std::ios::binary);
    if (!f) {
      throw std::runtime_error("Could not open TensorRT engine: " + engine_path_);
    }
    std::vector<char> blob((std::istreambuf_iterator<char>(f)),
                           std::istreambuf_iterator<char>());

    initLibNvInferPlugins(&logger_, "");
    runtime_.reset(nvinfer1::createInferRuntime(logger_));
    if (!runtime_) throw std::runtime_error("createInferRuntime failed");
    engine_.reset(runtime_->deserializeCudaEngine(blob.data(), blob.size()));
    if (!engine_) throw std::runtime_error("deserializeCudaEngine failed: " + engine_path_);
    context_.reset(engine_->createExecutionContext());
    if (!context_) throw std::runtime_error("createExecutionContext failed");

    // Identify the first input and first output tensor.
    const int n = engine_->getNbIOTensors();
    for (int i = 0; i < n; ++i) {
      const char* name = engine_->getIOTensorName(i);
      if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT && in_name_.empty()) {
        in_name_ = name;
      } else if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kOUTPUT &&
                 out_name_.empty()) {
        out_name_ = name;
      }
    }
    if (in_name_.empty() || out_name_.empty()) {
      throw std::runtime_error("Engine missing input/output tensor");
    }

    parseInputShape(engine_->getTensorShape(in_name_.c_str()));
    parseOutputShape(engine_->getTensorShape(out_name_.c_str()));
  }

  void parseInputShape(const nvinfer1::Dims& d) {
    if (d.nbDims != 4) {
      throw std::runtime_error("Unsupported engine input rank (expected NCHW)");
    }
    for (int i = 0; i < d.nbDims; ++i) {
      if (d.d[i] <= 0) throw std::runtime_error("Dynamic engine input; export with static shapes");
    }
    // [1, C, H, W] with H == W.
    if ((d.d[1] == 1 || d.d[1] == 3) && d.d[2] == d.d[3]) {
      input_channels_ = static_cast<int>(d.d[1]);
      img_size_ = static_cast<int>(d.d[2]);
    } else {
      throw std::runtime_error("Could not infer square NCHW input from engine");
    }
    channel_stride_ = img_size_ * img_size_;
  }

  // Port of zed_detector.py _parse_output_shape.
  void parseOutputShape(const nvinfer1::Dims& d) {
    std::vector<int> dims;
    out_count_ = 1;
    for (int i = 0; i < d.nbDims; ++i) {
      if (d.d[i] <= 0) throw std::runtime_error("Dynamic engine output; export with static shapes");
      out_count_ *= static_cast<size_t>(d.d[i]);
      if (d.d[i] != 1) dims.push_back(static_cast<int>(d.d[i]));
    }
    if (dims.size() != 2) {
      throw std::runtime_error("Unsupported engine output shape");
    }
    const int a = dims[0], b = dims[1];
    const int post_nms_c = 6 + kNumKeypoints * 3;  // 18

    if (b == post_nms_c) {
      output_mode_post_nms_ = true; output_channels_ = b; num_anchors_ = a; channels_first_ = false;
    } else if (a == post_nms_c) {
      output_mode_post_nms_ = true; output_channels_ = a; num_anchors_ = b; channels_first_ = true;
    } else if (a <= 256 && b > a) {
      output_mode_post_nms_ = false; output_channels_ = a; num_anchors_ = b; channels_first_ = true;
    } else if (b <= 256 && a > b) {
      output_mode_post_nms_ = false; output_channels_ = b; num_anchors_ = a; channels_first_ = false;
    } else {
      throw std::runtime_error("Could not infer engine output layout");
    }

    if (output_mode_post_nms_) {
      num_classes_ = kNumClassesPostNms;
    } else {
      num_classes_ = output_channels_ - 4 - kNumKeypoints * 3;
      if (num_classes_ <= 0) throw std::runtime_error("Invalid raw pose output channels");
    }
    AA_RCLCPP_INFO(enable_ros_logs_, get_logger(),
                "Engine net=%d in_ch=%d out_mode=%s out_ch=%d anchors=%d classes=%d",
                img_size_, input_channels_, output_mode_post_nms_ ? "post_nms" : "raw",
                output_channels_, num_anchors_, num_classes_);
  }

  // ─────────────────────────────────────────────────────────────────────────
  void allocateBuffers() {
    // Letterbox geometry (landscape native -> square net), same as ZED path.
    scale_net_to_native_ = static_cast<float>(native_w_) / static_cast<float>(img_size_);
    unscaled_h_net_ = static_cast<int>(std::lround(native_h_ / scale_net_to_native_));
    pad_y_net_ = (img_size_ - unscaled_h_net_) / 2;
    if (pad_y_net_ < 0) pad_y_net_ = 0;

    CUDA_CHECK(cudaMalloc(&d_in_, static_cast<size_t>(input_channels_) * channel_stride_ * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_out_, out_count_ * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_color_, static_cast<size_t>(native_w_) * native_h_ * color_bpp_));
    CUDA_CHECK(cudaMallocHost(&h_out_, out_count_ * sizeof(float)));
    CUDA_CHECK(cudaStreamCreate(&stream_));

    // Tensor addresses are fixed for the life of the context.
    context_->setTensorAddress(in_name_.c_str(), d_in_);
    context_->setTensorAddress(out_name_.c_str(), d_out_);

    // Preallocate decode scratch (reused every frame, no per-frame alloc).
    armors_.reserve(256);
    nms_rects_.reserve(256);
    nms_scores_.reserve(256);
    nms_armors_.reserve(256);
    nms_keep_.reserve(256);
  }

  // h_out_ row/channel accessor honoring the engine layout.
  inline float at(int row, int ch) const {
    return channels_first_ ? h_out_[static_cast<size_t>(ch) * num_anchors_ + row]
                           : h_out_[static_cast<size_t>(row) * output_channels_ + ch];
  }
  inline float net_x_to_native(float x) const { return x * scale_net_to_native_; }
  inline float net_y_to_native(float y) const { return (y - pad_y_net_) * scale_net_to_native_; }

  // ─────────────────────────────────────────────────────────────────────────
  void captureLoop() {
    while (running_ && rclcpp::ok()) {
      rs2::frameset fs;
      try {
        fs = pipe_.wait_for_frames(1000);
      } catch (const rs2::error& e) {
        AA_RCLCPP_WARN_THROTTLE(enable_ros_logs_, get_logger(), *get_clock(), 2000,
                             "RealSense wait_for_frames: %s", e.what());
        continue;
      }
      rs2::video_frame color = fs.get_color_frame();
      if (!color) continue;

      // Capture timestamp (mirrors the ZED capture-time stamping). RealSense
      // SYSTEM_TIME/GLOBAL_TIME domains are host-epoch aligned with the node
      // clock; hardware-clock domains are not, so fall back to now().
      rclcpp::Time stamp;
      const rs2_timestamp_domain dom = color.get_frame_timestamp_domain();
      if (dom == RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME || dom == RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME) {
        stamp = rclcpp::Time(static_cast<int64_t>(color.get_timestamp() * 1e6));
      } else {
        stamp = now();
      }

      processFrame(color, stamp);
      ++frame_count_;
    }
  }

  void processFrame(const rs2::video_frame& color, const rclcpp::Time& stamp) {
    // src points at the native color data; for BGR8 it may be redirected to a
    // CPU-flipped copy. For YUYV the flip is folded into the kernel instead.
    const uint8_t* src = static_cast<const uint8_t*>(color.get_data());

    // H2D color, preprocess, inference, D2H — all on one reused stream.
    if (use_yuyv_) {
      // Raw YUY2 cannot be flipped with cv::flip (it scrambles the U/Y/V
      // macropixel order), so upload as-is and let the kernel fold the 180 flip
      // into its source sampling.
      CUDA_CHECK(cudaMemcpyAsync(d_color_, src,
                                 static_cast<size_t>(native_w_) * native_h_ * 2,
                                 cudaMemcpyHostToDevice, stream_));
      launch_preprocess_yuyv(d_color_, d_in_, native_w_, native_h_, img_size_, img_size_,
                             pad_y_net_, unscaled_h_net_, scale_net_to_native_,
                             flip_180_, stream_);
    } else {
      // BGR8: flip on the CPU so the network input, debug image and detection
      // pixel coords stay mutually consistent (one flip feeds them all).
      if (flip_180_) {
        cv::Mat in_mat(native_h_, native_w_, CV_8UC3, const_cast<void*>(color.get_data()));
        cv::flip(in_mat, flip_buf_, -1);  // flipCode -1 = both axes = 180 deg
        src = flip_buf_.data;
      }
      CUDA_CHECK(cudaMemcpyAsync(d_color_, src,
                                 static_cast<size_t>(native_w_) * native_h_ * 3,
                                 cudaMemcpyHostToDevice, stream_));
      launch_preprocess_bgr8(d_color_, d_in_, native_w_, native_h_, img_size_, img_size_,
                             pad_y_net_, unscaled_h_net_, scale_net_to_native_, stream_);
    }
    if (!context_->enqueueV3(stream_)) {
      AA_RCLCPP_WARN_THROTTLE(enable_ros_logs_, get_logger(), *get_clock(), 2000, "enqueueV3 failed");
      return;
    }
    CUDA_CHECK(cudaMemcpyAsync(h_out_, d_out_, out_count_ * sizeof(float),
                               cudaMemcpyDeviceToHost, stream_));
    CUDA_CHECK(cudaStreamSynchronize(stream_));

    armors_.clear();
    if (output_mode_post_nms_) {
      decodePostNms();
    } else {
      decodeRaw();
    }

    publishDetections(stamp);

    if (camera_info_every_ > 0 && (frame_count_ % camera_info_every_) == 0) {
      cam_info_msg_.header.stamp = stamp;
      cam_info_pub_->publish(cam_info_msg_);
    }
    const bool want_debug =
        publish_debug_every_ > 0 && (frame_count_ % publish_debug_every_) == 0;
    const bool want_record =
        publish_record_every_ > 0 && (frame_count_ % publish_record_every_) == 0;
    if (want_debug || want_record) {
      const uint8_t* bgr = src;
      if (use_yuyv_) {
        // Build a BGR image from the raw YUY2 frame (throttled, so this CPU
        // conversion is cheap), matching the network input orientation.
        cv::Mat yuyv(native_h_, native_w_, CV_8UC2,
                     const_cast<void*>(color.get_data()));
        cv::cvtColor(yuyv, debug_bgr_, cv::COLOR_YUV2BGR_YUYV);
        if (flip_180_) cv::flip(debug_bgr_, debug_bgr_, -1);
        bgr = debug_bgr_.data;
      }
      if (want_debug) publishDebugImage(bgr, stamp);
      if (want_record) publishRecordImage(bgr, stamp);
    }
  }

  // Port of _decode_post_nms_rows (engine already ran NMS).
  void decodePostNms() {
    for (int r = 0; r < num_anchors_; ++r) {
      const float conf = at(r, 4);
      if (!std::isfinite(conf) || conf <= threshold_) continue;

      float x1 = at(r, 0), y1 = at(r, 1), x2 = at(r, 2), y2 = at(r, 3);
      // Normalized-coordinate guard: scale up if coords look like [0,1].
      const float m = std::max(std::max(std::fabs(x1), std::fabs(y1)),
                               std::max(std::fabs(x2), std::fabs(y2)));
      if (std::isfinite(m) && m <= 2.0f) {
        x1 *= img_size_; y1 *= img_size_; x2 *= img_size_; y2 *= img_size_;
      }
      const float x1n = net_x_to_native(x1), x2n = net_x_to_native(x2);
      const float y1n = net_y_to_native(y1), y2n = net_y_to_native(y2);

      Armor a;
      a.cls = static_cast<int>(at(r, 5));
      a.conf = conf;
      a.L = std::min(x1n, x2n);
      a.T = std::min(y1n, y2n);
      a.W = std::fabs(x2n - x1n);
      a.H = std::fabs(y2n - y1n);
      fillKeypoints(r, 6, a);
      armors_.push_back(a);
      if (static_cast<int>(armors_.size()) >= kMaxDetections) break;
    }
  }

  // Port of _decode_raw_pose_candidates + cv2 NMS.
  void decodeRaw() {
    nms_rects_.clear();
    nms_scores_.clear();
    nms_armors_.clear();
    const int kpt_start = 4 + num_classes_;
    for (int r = 0; r < num_anchors_; ++r) {
      int best_c = 0;
      float best = at(r, 4);
      for (int c = 1; c < num_classes_; ++c) {
        const float s = at(r, 4 + c);
        if (s > best) { best = s; best_c = c; }
      }
      if (!(best > threshold_)) continue;

      const float cx = at(r, 0), cy = at(r, 1), w = at(r, 2), h = at(r, 3);
      Armor a;
      a.cls = best_c;
      a.conf = best;
      a.W = w * scale_net_to_native_;
      a.H = h * scale_net_to_native_;
      a.L = cx * scale_net_to_native_ - a.W * 0.5f;
      a.T = (cy - pad_y_net_) * scale_net_to_native_ - a.H * 0.5f;
      fillKeypoints(r, kpt_start, a);

      nms_armors_.push_back(a);
      nms_rects_.emplace_back(a.L, a.T, a.W, a.H);
      nms_scores_.push_back(a.conf);
    }
    if (nms_armors_.empty()) return;

    nms_keep_.clear();
    cv::dnn::NMSBoxes(nms_rects_, nms_scores_, threshold_, nms_iou_, nms_keep_);
    for (int idx : nms_keep_) {
      armors_.push_back(nms_armors_[idx]);
      if (static_cast<int>(armors_.size()) >= kMaxDetections) break;
    }
  }

  // Keypoints start at channel kp0 (raw: 4+classes, post_nms: 6); 3 vals each.
  void fillKeypoints(int row, int kp0, Armor& a) {
    // Detect normalized keypoints once per detection.
    float maxabs = 0.0f;
    for (int k = 0; k < kNumKeypoints; ++k) {
      maxabs = std::max(maxabs, std::fabs(at(row, kp0 + k * 3 + 0)));
      maxabs = std::max(maxabs, std::fabs(at(row, kp0 + k * 3 + 1)));
    }
    const bool normalized = std::isfinite(maxabs) && maxabs <= 2.0f;
    for (int k = 0; k < kNumKeypoints; ++k) {
      float x = at(row, kp0 + k * 3 + 0);
      float y = at(row, kp0 + k * 3 + 1);
      const float score = at(row, kp0 + k * 3 + 2);
      if (normalized) { x *= img_size_; y *= img_size_; }
      a.kp[k * 3 + 0] = net_x_to_native(x);
      a.kp[k * 3 + 1] = net_y_to_native(y);
      a.kp[k * 3 + 2] = score;
    }
  }

  // ─────────────────────────────────────────────────────────────────────────
  void publishDetections(const rclcpp::Time& stamp) {
    vision_msgs::msg::Detection2DArray det_arr;
    det_arr.header.stamp = stamp;
    det_arr.header.frame_id = frame_id_;

    autoaim::msg::ArmorKeypointArray kpt_arr;
    kpt_arr.header.stamp = stamp;
    kpt_arr.header.frame_id = frame_id_;
    kpt_arr.image_width = static_cast<uint32_t>(native_w_);
    kpt_arr.image_height = static_cast<uint32_t>(native_h_);
    kpt_arr.keypoint_order = {"TL", "TR", "BR", "BL"};

    std::unique_ptr<std::ostringstream> json;
    if (publish_json_) {
      json = std::make_unique<std::ostringstream>();
      *json << "{\"header\":{\"stamp\":{\"sec\":" << stamp.seconds() << "},\"frame_id\":\""
            << frame_id_ << "\"},\"image_width\":" << native_w_ << ",\"image_height\":"
            << native_h_
            << ",\"keypoint_order\":[\"TL\",\"TR\",\"BR\",\"BL\"],\"detections\":[";
    }

    bool first = true;
    for (const Armor& a : armors_) {
      const float L = std::max(0.0f, std::min(static_cast<float>(native_w_), a.L));
      const float T = std::max(0.0f, std::min(static_cast<float>(native_h_), a.T));
      const float R = std::max(0.0f, std::min(static_cast<float>(native_w_), a.L + a.W));
      const float B = std::max(0.0f, std::min(static_cast<float>(native_h_), a.T + a.H));
      const float W = R - L, H = B - T;
      if (W <= 1.0f || H <= 1.0f) continue;

      // Legacy bbox topic.
      vision_msgs::msg::Detection2D det;
      det.header.stamp = stamp;
      det.header.frame_id = frame_id_;
      det.bbox.center.position.x = L + W * 0.5;
      det.bbox.center.position.y = T + H * 0.5;
      det.bbox.size_x = W;
      det.bbox.size_y = H;
      vision_msgs::msg::ObjectHypothesisWithPose hyp;
      hyp.hypothesis.class_id = std::to_string(a.cls);
      hyp.hypothesis.score = a.conf;
      det.results.push_back(hyp);
      det_arr.detections.push_back(det);

      // Typed keypoint topic (tracking input).
      autoaim::msg::ArmorKeypoint k;
      k.header.stamp = stamp;
      k.header.frame_id = frame_id_;
      k.class_id = a.cls;
      k.class_name = class_name(a.cls);
      k.confidence = a.conf;
      k.bbox_cx = L + W * 0.5f;
      k.bbox_cy = T + H * 0.5f;
      k.bbox_w = W;
      k.bbox_h = H;
      for (int j = 0; j < kNumKeypoints; ++j) {
        const float kx = std::min(std::max(a.kp[j * 3 + 0], 0.0f), static_cast<float>(native_w_));
        const float ky = std::min(std::max(a.kp[j * 3 + 1], 0.0f), static_cast<float>(native_h_));
        k.keypoints_xy[j * 2 + 0] = kx;
        k.keypoints_xy[j * 2 + 1] = ky;
        k.keypoint_scores[j] = a.kp[j * 3 + 2];
        k.keypoint_valid[j] = a.kp[j * 3 + 2] >= kKeypointScoreThreshold;
      }
      kpt_arr.detections.push_back(k);

      if (publish_json_) {
        if (!first) *json << ",";
        first = false;
        *json << "{\"class_id\":" << a.cls << ",\"class_name\":\"" << class_name(a.cls)
              << "\",\"confidence\":" << a.conf << ",\"bbox\":{\"cx\":" << (L + W * 0.5f)
              << ",\"cy\":" << (T + H * 0.5f) << ",\"w\":" << W << ",\"h\":" << H << "}}";
      }
    }

    det_pub_->publish(det_arr);
    kpt_pub_->publish(kpt_arr);
    if (publish_json_ && json_pub_) {
      *json << "]}";
      std_msgs::msg::String js;
      js.data = json->str();
      json_pub_->publish(js);
    }
  }

  // Clean camera frame (no overlays) as JPEG, for the dataset-recording rosbag
  // started by the autostart script. Per-frame JPEG beats bag-level zstd on
  // camera images by an order of magnitude, so the bag records only this topic.
  void publishRecordImage(const uint8_t* data, const rclcpp::Time& stamp) {
    cv::Mat bgr(native_h_, native_w_, CV_8UC3, const_cast<uint8_t*>(data));
    if (!cv::imencode(".jpg", bgr, jpeg_buf_,
                      {cv::IMWRITE_JPEG_QUALITY, record_jpeg_quality_})) {
      AA_RCLCPP_WARN_THROTTLE(enable_ros_logs_, get_logger(), *get_clock(), 2000,
                           "JPEG encode failed for recording image");
      return;
    }
    sensor_msgs::msg::CompressedImage msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    msg.format = "jpeg";
    msg.data.assign(jpeg_buf_.begin(), jpeg_buf_.end());
    rec_pub_->publish(msg);
  }

  void publishDebugImage(const uint8_t* data, const rclcpp::Time& stamp) {
    sensor_msgs::msg::Image img;
    img.header.stamp = stamp;
    img.header.frame_id = frame_id_;
    img.height = static_cast<uint32_t>(native_h_);
    img.width = static_cast<uint32_t>(native_w_);
    img.encoding = "bgr8";
    img.is_bigendian = 0;
    img.step = static_cast<uint32_t>(native_w_ * 3);
    const size_t n = static_cast<size_t>(img.step) * native_h_;
    img.data.assign(data, data + n);
    img_pub_->publish(img);
  }

  // ── Parameters ──
  std::string engine_path_, serial_no_, frame_id_;
  float threshold_{0.15f}, nms_iou_{0.2f};
  int req_width_{640}, req_height_{480}, req_fps_{60};
  bool auto_exposure_{false};
  int exposure_{6000}, gain_{64};
  bool enable_depth_{false}, enable_infrared_{false}, enable_imu_{false};
  bool flip_180_{false};
  std::string pixel_format_{"bgr8"};
  bool use_yuyv_{false};
  int color_bpp_{3};  // bytes/px of the color stream: 3 (BGR8) or 2 (YUYV)
  cv::Mat flip_buf_;   // reused 180-rotated color buffer (BGR8 + flip_180_ only)
  cv::Mat debug_bgr_;  // reused BGR debug image built from YUYV (yuyv only)
  int publish_debug_every_{0}, camera_info_every_{1};
  int publish_record_every_{0}, record_jpeg_quality_{85};
  std::vector<uint8_t> jpeg_buf_;  // reused JPEG encode buffer (recording only)
  bool publish_json_{false};
  bool enable_ros_logs_{false};

  // ── RealSense ──
  rs2::pipeline pipe_;
  int native_w_{0}, native_h_{0};
  sensor_msgs::msg::CameraInfo cam_info_msg_;

  // ── TensorRT ──
  TrtLogger logger_;
  TrtPtr<nvinfer1::IRuntime> runtime_;
  TrtPtr<nvinfer1::ICudaEngine> engine_;
  TrtPtr<nvinfer1::IExecutionContext> context_;
  std::string in_name_, out_name_;
  int img_size_{0}, input_channels_{0}, channel_stride_{0};
  bool output_mode_post_nms_{false}, channels_first_{false};
  int output_channels_{0}, num_anchors_{0}, num_classes_{0};
  size_t out_count_{0};

  // ── Letterbox geometry ──
  float scale_net_to_native_{1.0f};
  int unscaled_h_net_{0}, pad_y_net_{0};

  // ── CUDA buffers ──
  float* d_in_{nullptr};
  float* d_out_{nullptr};
  unsigned char* d_color_{nullptr};
  float* h_out_{nullptr};
  cudaStream_t stream_{nullptr};

  // ── Decode scratch (reused) ──
  std::vector<Armor> armors_;
  std::vector<cv::Rect2d> nms_rects_;
  std::vector<float> nms_scores_;
  std::vector<Armor> nms_armors_;
  std::vector<int> nms_keep_;

  // ── Publishers ──
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr det_pub_;
  rclcpp::Publisher<autoaim::msg::ArmorKeypointArray>::SharedPtr kpt_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr json_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr rec_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_pub_;

  std::atomic<bool> running_{false};
  std::thread worker_;
  long frame_count_{0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<RealSenseDetector>());
  } catch (const std::exception& e) {
    AA_RCLCPP_FATAL(g_realsense_ros_logs_enabled.load(std::memory_order_relaxed), rclcpp::get_logger("realsense_detector"), "fatal: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
