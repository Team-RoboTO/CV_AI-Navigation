// CUDA preprocess kernel for the RealSense detector path. See
// include/autoaim_realsense/realsense_preprocess.h for the contract. Kept identical in
// math to the zed_detector.py kernel (only the input stride differs: BGR8 vs
// BGRA), so both camera paths produce the same network input.

#include "autoaim_realsense/realsense_preprocess.h"

namespace {

__global__ void preprocess_bgr8_kernel(const unsigned char* __restrict__ in,
                                       float* __restrict__ out,
                                       int in_w, int in_h, int out_w, int out_h,
                                       int pad_y, int unscaled_h, float scale) {
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= out_w || y >= out_h) {
    return;
  }

  const int stride = out_w * out_h;  // channel stride
  const int o = y * out_w + x;

  if (y < pad_y || y >= (pad_y + unscaled_h)) {
    // Letterbox pad band.
    out[o] = 0.447f;
    out[o + stride] = 0.447f;
    out[o + 2 * stride] = 0.447f;
  } else {
    int sx = static_cast<int>(x * scale);
    int sy = static_cast<int>((y - pad_y) * scale);
    if (sx >= in_w) sx = in_w - 1;
    if (sy >= in_h) sy = in_h - 1;
    const int i = (sy * in_w + sx) * 3;  // BGR8 source
    out[o] = static_cast<float>(in[i + 2]) / 255.0f;               // R
    out[o + stride] = static_cast<float>(in[i + 1]) / 255.0f;      // G
    out[o + 2 * stride] = static_cast<float>(in[i + 0]) / 255.0f;  // B
  }
}

__global__ void preprocess_yuyv_kernel(const unsigned char* __restrict__ in,
                                       float* __restrict__ out,
                                       int in_w, int in_h, int out_w, int out_h,
                                       int pad_y, int unscaled_h, float scale,
                                       bool flip180) {
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= out_w || y >= out_h) {
    return;
  }

  const int stride = out_w * out_h;  // channel stride
  const int o = y * out_w + x;

  if (y < pad_y || y >= (pad_y + unscaled_h)) {
    // Letterbox pad band.
    out[o] = 0.447f;
    out[o + stride] = 0.447f;
    out[o + 2 * stride] = 0.447f;
    return;
  }

  int sx = static_cast<int>(x * scale);
  int sy = static_cast<int>((y - pad_y) * scale);
  if (sx >= in_w) sx = in_w - 1;
  if (sy >= in_h) sy = in_h - 1;
  if (flip180) {           // 180 deg = mirror both axes in source coords
    sx = in_w - 1 - sx;
    sy = in_h - 1 - sy;
  }

  // YUY2 macropixel: 4 bytes [Y0, U, Y1, V] cover the pixel pair (2k, 2k+1).
  const int row = sy * in_w * 2;
  const int base = row + (sx >> 1) * 4;
  const float Y = static_cast<float>(in[base + ((sx & 1) ? 2 : 0)]);
  const float U = static_cast<float>(in[base + 1]);
  const float V = static_cast<float>(in[base + 3]);

  // BT.601 video-range YUV->RGB (matches librealsense's YUY2 converter).
  const float c = 1.164f * (Y - 16.0f);
  const float d = U - 128.0f;
  const float e = V - 128.0f;
  float r = c + 1.596f * e;
  float g = c - 0.391f * d - 0.813f * e;
  float b = c + 2.018f * d;
  r = fminf(fmaxf(r, 0.0f), 255.0f);
  g = fminf(fmaxf(g, 0.0f), 255.0f);
  b = fminf(fmaxf(b, 0.0f), 255.0f);

  out[o] = r / 255.0f;               // R
  out[o + stride] = g / 255.0f;      // G
  out[o + 2 * stride] = b / 255.0f;  // B
}

}  // namespace

void launch_preprocess_bgr8(const unsigned char* in, float* out,
                            int in_w, int in_h, int out_w, int out_h,
                            int pad_y, int unscaled_h, float scale,
                            cudaStream_t stream) {
  const dim3 block(16, 16, 1);
  const dim3 grid((out_w + 15) / 16, (out_h + 15) / 16, 1);
  preprocess_bgr8_kernel<<<grid, block, 0, stream>>>(
      in, out, in_w, in_h, out_w, out_h, pad_y, unscaled_h, scale);
}

void launch_preprocess_yuyv(const unsigned char* in, float* out,
                            int in_w, int in_h, int out_w, int out_h,
                            int pad_y, int unscaled_h, float scale,
                            bool flip180, cudaStream_t stream) {
  const dim3 block(16, 16, 1);
  const dim3 grid((out_w + 15) / 16, (out_h + 15) / 16, 1);
  preprocess_yuyv_kernel<<<grid, block, 0, stream>>>(
      in, out, in_w, in_h, out_w, out_h, pad_y, unscaled_h, scale, flip180);
}
