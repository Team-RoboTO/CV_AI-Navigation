#ifndef AUTOAIM_V2__DETECTOR_TRT_HPP_
#define AUTOAIM_V2__DETECTOR_TRT_HPP_

#include <cuda_runtime_api.h>

#include <memory>
#include <string>
#include <vector>

#include "autoaim_v2/types.hpp"

namespace nvinfer1
{
class IRuntime;
class ICudaEngine;
class IExecutionContext;
}  // namespace nvinfer1

namespace aim
{

// CUDA letterbox preprocess: BGRA (pitched, device) -> normalized RGB NCHW
// fp32 (device). Implemented in preprocess.cu.
void cuda_preprocess_bgra(const uint8_t * d_bgra, size_t pitch_bytes, int in_w,
                          int in_h, float * d_out, int net_size, int pad_y,
                          float scale_net_to_native, cudaStream_t stream);

struct DetectorParams
{
  std::string engine_path;
  float conf_threshold = 0.15f;
  float nms_iou = 0.2f;
  int max_detections = 30;
  int native_w = 960, native_h = 600;
};

// TensorRT YOLO-pose armor detector. Consumes the SAME engine file as the old
// Python detector (yolov26_keypoints.engine): auto-detects raw
// [1,C,A]/[1,A,C] and post-NMS/end2end [1,max_det,6+12] output layouts.
// Keypoint order TL, TR, BR, BL in native (960x600) pixels.
class DetectorTRT
{
public:
  explicit DetectorTRT(const DetectorParams & p);
  ~DetectorTRT();

  DetectorTRT(const DetectorTRT &) = delete;
  DetectorTRT & operator=(const DetectorTRT &) = delete;

  bool ok() const { return ok_; }
  const std::string & error() const { return error_; }
  int net_size() const { return net_size_; }

  // d_bgra: device pointer to the ZED left image (BGRA8, pitched).
  // Runs preprocess + inference + decode. Synchronous on `stream`.
  std::vector<ArmorPx> infer(const uint8_t * d_bgra, size_t pitch_bytes,
                             cudaStream_t stream);

private:
  void decode_raw(std::vector<ArmorPx> & out);
  void decode_post_nms(std::vector<ArmorPx> & out);
  float net_x_to_native(float x) const { return x * scale_; }
  float net_y_to_native(float y) const { return (y - pad_y_) * scale_; }

  DetectorParams p_;
  bool ok_ = false;
  std::string error_;

  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> context_;
  std::string in_name_, out_name_;

  int net_size_ = 640;
  int pad_y_ = 0;
  float scale_ = 1.f;

  bool post_nms_ = false;
  bool channels_first_ = false;  // raw layout [1, C, A]
  int out_channels_ = 0;
  int out_rows_ = 0;
  int num_classes_ = 3;

  float * d_input_ = nullptr;
  float * d_output_ = nullptr;
  std::vector<float> h_output_;
};

}  // namespace aim

#endif  // AUTOAIM_V2__DETECTOR_TRT_HPP_
