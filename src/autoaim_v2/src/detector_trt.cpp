#include "autoaim_v2/detector_trt.hpp"

#include <NvInfer.h>

#include <algorithm>
#include <cstring>
#include <fstream>
#include <numeric>
#include <opencv2/dnn.hpp>

namespace aim
{
namespace
{

class TrtLogger : public nvinfer1::ILogger
{
  void log(Severity severity, const char * msg) noexcept override
  {
    if (severity <= Severity::kERROR) fprintf(stderr, "[TRT] %s\n", msg);
  }
};
TrtLogger g_logger;

constexpr int NUM_KPTS = 4;
constexpr int MAX_CANDIDATES = 80;

}  // namespace

DetectorTRT::DetectorTRT(const DetectorParams & p) : p_(p)
{
  std::ifstream f(p_.engine_path, std::ios::binary);
  if (!f) {
    error_ = "cannot open engine file: " + p_.engine_path;
    return;
  }
  std::vector<char> blob((std::istreambuf_iterator<char>(f)),
                         std::istreambuf_iterator<char>());

  runtime_.reset(nvinfer1::createInferRuntime(g_logger));
  if (!runtime_) {
    error_ = "createInferRuntime failed";
    return;
  }
  engine_.reset(runtime_->deserializeCudaEngine(blob.data(), blob.size()));
  if (!engine_) {
    error_ = "deserializeCudaEngine failed (wrong TRT version for this .engine?)";
    return;
  }
  context_.reset(engine_->createExecutionContext());
  if (!context_) {
    error_ = "createExecutionContext failed";
    return;
  }

  // Identify the IO tensors (one input, one output — Ultralytics export).
  for (int i = 0; i < engine_->getNbIOTensors(); i++) {
    const char * name = engine_->getIOTensorName(i);
    if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) {
      in_name_ = name;
    } else {
      out_name_ = name;
    }
  }
  if (in_name_.empty() || out_name_.empty()) {
    error_ = "engine must have exactly one input and one output tensor";
    return;
  }
  if (engine_->getTensorDataType(in_name_.c_str()) != nvinfer1::DataType::kFLOAT ||
      engine_->getTensorDataType(out_name_.c_str()) != nvinfer1::DataType::kFLOAT) {
    error_ = "engine IO must be fp32 (export without --int8/--fp16 IO)";
    return;
  }

  const auto in_dims = engine_->getTensorShape(in_name_.c_str());
  if (in_dims.nbDims != 4 || in_dims.d[1] != 3 || in_dims.d[2] != in_dims.d[3] ||
      in_dims.d[2] <= 0) {
    error_ = "expected static NCHW [1,3,S,S] input";
    return;
  }
  net_size_ = static_cast<int>(in_dims.d[2]);
  scale_ = static_cast<float>(p_.native_w) / static_cast<float>(net_size_);
  const int content_h = static_cast<int>(std::lround(p_.native_h / scale_));
  pad_y_ = (net_size_ - content_h) / 2;

  const auto out_dims = engine_->getTensorShape(out_name_.c_str());
  // Squeeze leading 1s: expect two meaningful dims.
  std::vector<int> dims;
  for (int i = 0; i < out_dims.nbDims; i++) {
    if (out_dims.d[i] != 1) dims.push_back(static_cast<int>(out_dims.d[i]));
  }
  if (dims.size() != 2 || dims[0] <= 0 || dims[1] <= 0) {
    error_ = "unsupported output shape (need static [1,C,A] / [1,A,C] / [1,N,18])";
    return;
  }
  const int post_nms_c = 6 + NUM_KPTS * 3;
  if (dims[1] == post_nms_c) {
    post_nms_ = true;
    out_rows_ = dims[0];
    out_channels_ = dims[1];
  } else if (dims[0] == post_nms_c) {
    post_nms_ = true;
    out_rows_ = dims[1];
    out_channels_ = dims[0];
    channels_first_ = true;
  } else if (dims[0] <= 256 && dims[1] > dims[0]) {
    channels_first_ = true;
    out_channels_ = dims[0];
    out_rows_ = dims[1];
  } else if (dims[1] <= 256 && dims[0] > dims[1]) {
    out_channels_ = dims[1];
    out_rows_ = dims[0];
  } else {
    error_ = "could not infer output layout";
    return;
  }
  if (!post_nms_) {
    num_classes_ = out_channels_ - 4 - NUM_KPTS * 3;
    if (num_classes_ <= 0) {
      error_ = "raw output channels inconsistent with 4-keypoint pose model";
      return;
    }
  }

  size_t in_bytes = static_cast<size_t>(3) * net_size_ * net_size_ * sizeof(float);
  size_t out_count = static_cast<size_t>(out_rows_) * out_channels_;
  if (cudaMalloc(reinterpret_cast<void **>(&d_input_), in_bytes) != cudaSuccess ||
      cudaMalloc(reinterpret_cast<void **>(&d_output_), out_count * sizeof(float)) !=
        cudaSuccess) {
    error_ = "cudaMalloc failed";
    return;
  }
  h_output_.resize(out_count);
  ok_ = true;
}

DetectorTRT::~DetectorTRT()
{
  if (d_input_) cudaFree(d_input_);
  if (d_output_) cudaFree(d_output_);
}

std::vector<ArmorPx> DetectorTRT::infer_contiguous(const uint8_t * d_color,
                                                   bool is_yuyv, bool flip180,
                                                   cudaStream_t stream)
{
  std::vector<ArmorPx> out;
  if (!ok_) return out;

  if (is_yuyv) {
    cuda_preprocess_yuyv(d_color, p_.native_w, p_.native_h, d_input_,
                         net_size_, pad_y_, scale_, flip180, stream);
  } else {
    cuda_preprocess_bgr8(d_color, p_.native_w, p_.native_h, d_input_,
                         net_size_, pad_y_, scale_, flip180, stream);
  }

  context_->setTensorAddress(in_name_.c_str(), d_input_);
  context_->setTensorAddress(out_name_.c_str(), d_output_);
  if (!context_->enqueueV3(stream)) return out;

  cudaMemcpyAsync(h_output_.data(), d_output_, h_output_.size() * sizeof(float),
                  cudaMemcpyDeviceToHost, stream);
  cudaStreamSynchronize(stream);

  if (post_nms_) {
    decode_post_nms(out);
  } else {
    decode_raw(out);
  }
  return out;
}

std::vector<ArmorPx> DetectorTRT::infer(const uint8_t * d_bgra,
                                        size_t pitch_bytes, cudaStream_t stream)
{
  std::vector<ArmorPx> out;
  if (!ok_) return out;

  cuda_preprocess_bgra(d_bgra, pitch_bytes, p_.native_w, p_.native_h, d_input_,
                       net_size_, pad_y_, scale_, stream);

  context_->setTensorAddress(in_name_.c_str(), d_input_);
  context_->setTensorAddress(out_name_.c_str(), d_output_);
  if (!context_->enqueueV3(stream)) return out;

  cudaMemcpyAsync(h_output_.data(), d_output_, h_output_.size() * sizeof(float),
                  cudaMemcpyDeviceToHost, stream);
  cudaStreamSynchronize(stream);

  if (post_nms_) {
    decode_post_nms(out);
  } else {
    decode_raw(out);
  }
  return out;
}

void DetectorTRT::decode_post_nms(std::vector<ArmorPx> & out)
{
  // Row: x1,y1,x2,y2,conf,class,(kx,ky,ks)*4 in net letterbox pixels.
  // Robustness: some exports emit normalized coords — detect and upscale.
  float max_coord = 0.f;
  for (int r = 0; r < out_rows_; r++) {
    const float * row = &h_output_[static_cast<size_t>(r) * out_channels_];
    if (!(row[4] > p_.conf_threshold)) continue;
    for (int j = 0; j < 4; j++) max_coord = std::max(max_coord, std::fabs(row[j]));
  }
  const float coord_scale = (max_coord > 0.f && max_coord <= 2.f)
                              ? static_cast<float>(net_size_)
                              : 1.f;

  for (int r = 0; r < out_rows_ && static_cast<int>(out.size()) < p_.max_detections;
       r++) {
    const float * row = &h_output_[static_cast<size_t>(r) * out_channels_];
    const float conf = row[4];
    if (!std::isfinite(conf) || conf <= p_.conf_threshold) continue;

    ArmorPx a;
    a.confidence = conf;
    a.class_id = static_cast<int>(row[5]);
    bool kpts_ok = true;
    for (int k = 0; k < NUM_KPTS; k++) {
      float kx = net_x_to_native(row[6 + 3 * k + 0] * coord_scale);
      float ky = net_y_to_native(row[6 + 3 * k + 1] * coord_scale);
      float ks = row[6 + 3 * k + 2];
      // Border rejection instead of clamping: a corner at/over the edge means
      // the plate is partially out of frame — a clamped quad still PnP-solves
      // to a confident but WRONG pose. Skip the plate entirely.
      if (!std::isfinite(kx) || !std::isfinite(ky) || ks < 0.05f ||
          kx < 2.f || kx > p_.native_w - 2.f || ky < 2.f || ky > p_.native_h - 2.f) {
        kpts_ok = false;
        break;
      }
      a.corners[k] = cv::Point2f(kx, ky);
    }
    if (kpts_ok) out.push_back(a);
  }
}

void DetectorTRT::decode_raw(std::vector<ArmorPx> & out)
{
  // Raw pose head: per anchor [cx,cy,w,h, cls0..clsN, (kx,ky,ks)*4].
  auto at = [&](int row, int ch) -> float {
    return channels_first_
             ? h_output_[static_cast<size_t>(ch) * out_rows_ + row]
             : h_output_[static_cast<size_t>(row) * out_channels_ + ch];
  };

  struct Cand
  {
    float conf;
    int cls;
    int row;
  };
  std::vector<Cand> cands;
  cands.reserve(256);
  for (int r = 0; r < out_rows_; r++) {
    float best = 0.f;
    int cls = 0;
    for (int c = 0; c < num_classes_; c++) {
      float s = at(r, 4 + c);
      if (s > best) {
        best = s;
        cls = c;
      }
    }
    if (best > p_.conf_threshold) cands.push_back({best, cls, r});
  }
  if (cands.empty()) return;
  if (static_cast<int>(cands.size()) > MAX_CANDIDATES) {
    std::nth_element(cands.begin(), cands.begin() + MAX_CANDIDATES, cands.end(),
                     [](const Cand & a, const Cand & b) { return a.conf > b.conf; });
    cands.resize(MAX_CANDIDATES);
  }

  std::vector<cv::Rect2d> boxes;
  std::vector<float> scores;
  for (const auto & c : cands) {
    const float w = at(c.row, 2) * scale_;
    const float h = at(c.row, 3) * scale_;
    const float x = at(c.row, 0) * scale_ - w / 2;
    const float y = (at(c.row, 1) - pad_y_) * scale_ - h / 2;
    boxes.emplace_back(x, y, w, h);
    scores.push_back(c.conf);
  }
  std::vector<int> keep;
  cv::dnn::NMSBoxes(boxes, scores, p_.conf_threshold, p_.nms_iou, keep);

  const int kpt_start = 4 + num_classes_;
  for (int idx : keep) {
    if (static_cast<int>(out.size()) >= p_.max_detections) break;
    const auto & c = cands[idx];
    ArmorPx a;
    a.confidence = c.conf;
    a.class_id = c.cls;
    bool kpts_ok = true;
    for (int k = 0; k < NUM_KPTS; k++) {
      float kx = net_x_to_native(at(c.row, kpt_start + 3 * k + 0));
      float ky = net_y_to_native(at(c.row, kpt_start + 3 * k + 1));
      float ks = at(c.row, kpt_start + 3 * k + 2);
      // Border rejection: see decode_post_nms.
      if (!std::isfinite(kx) || !std::isfinite(ky) || ks < 0.05f ||
          kx < 2.f || kx > p_.native_w - 2.f || ky < 2.f || ky > p_.native_h - 2.f) {
        kpts_ok = false;
        break;
      }
      a.corners[k] = cv::Point2f(kx, ky);
    }
    if (kpts_ok) out.push_back(a);
  }
}

}  // namespace aim
