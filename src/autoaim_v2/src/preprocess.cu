#include <cuda_runtime_api.h>

#include <cstdint>

namespace aim
{

// BGRA (pitched) -> letterboxed, normalized RGB NCHW fp32.
// Same geometry as the old Python/PyCUDA kernel: nearest-neighbor sample,
// gray (0.447) padding above/below the content band.
__global__ void k_preprocess(const uint8_t * __restrict__ in, size_t pitch,
                             int in_w, int in_h, float * __restrict__ out,
                             int net, int pad_y, float scale)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= net || y >= net) return;

  const int stride = net * net;
  const int o = y * net + x;

  if (y < pad_y || y >= net - pad_y) {
    out[o] = 0.447f;
    out[o + stride] = 0.447f;
    out[o + 2 * stride] = 0.447f;
    return;
  }

  int sx = static_cast<int>(x * scale);
  int sy = static_cast<int>((y - pad_y) * scale);
  if (sx >= in_w) sx = in_w - 1;
  if (sy >= in_h) sy = in_h - 1;

  const uint8_t * px = in + sy * pitch + sx * 4;  // B G R A
  out[o] = px[2] / 255.f;                          // R
  out[o + stride] = px[1] / 255.f;                 // G
  out[o + 2 * stride] = px[0] / 255.f;             // B
}

void cuda_preprocess_bgra(const uint8_t * d_bgra, size_t pitch_bytes, int in_w,
                          int in_h, float * d_out, int net_size, int pad_y,
                          float scale_net_to_native, cudaStream_t stream)
{
  dim3 block(16, 16);
  dim3 grid((net_size + 15) / 16, (net_size + 15) / 16);
  k_preprocess<<<grid, block, 0, stream>>>(d_bgra, pitch_bytes, in_w, in_h,
                                           d_out, net_size, pad_y,
                                           scale_net_to_native);
}

// BGR8 (contiguous, host-uploaded) -> letterboxed, normalized RGB NCHW fp32.
// For RealSense in BGR8 pixel format.
__global__ void k_preprocess_bgr8(const uint8_t * __restrict__ in,
                                  int in_w, int in_h, float * __restrict__ out,
                                  int net, int pad_y, float scale, bool flip180)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= net || y >= net) return;

  const int stride = net * net;
  const int o = y * net + x;

  if (y < pad_y || y >= net - pad_y) {
    out[o] = 0.447f;
    out[o + stride] = 0.447f;
    out[o + 2 * stride] = 0.447f;
    return;
  }

  int sx = static_cast<int>(x * scale);
  int sy = static_cast<int>((y - pad_y) * scale);
  if (sx >= in_w) sx = in_w - 1;
  if (sy >= in_h) sy = in_h - 1;
  if (flip180) { sx = in_w - 1 - sx; sy = in_h - 1 - sy; }

  const int i = (sy * in_w + sx) * 3;
  out[o] = in[i + 2] / 255.f;                // R
  out[o + stride] = in[i + 1] / 255.f;       // G
  out[o + 2 * stride] = in[i + 0] / 255.f;   // B
}

// YUYV (contiguous, host-uploaded) -> letterboxed, normalized RGB NCHW fp32.
// For RealSense in raw YUYV pixel format (saves CPU vs librealsense BGR
// conversion). BT.601 video-range coefficients match librealsense's converter.
__global__ void k_preprocess_yuyv(const uint8_t * __restrict__ in,
                                  int in_w, int in_h, float * __restrict__ out,
                                  int net, int pad_y, float scale, bool flip180)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= net || y >= net) return;

  const int stride = net * net;
  const int o = y * net + x;

  if (y < pad_y || y >= net - pad_y) {
    out[o] = 0.447f;
    out[o + stride] = 0.447f;
    out[o + 2 * stride] = 0.447f;
    return;
  }

  int sx = static_cast<int>(x * scale);
  int sy = static_cast<int>((y - pad_y) * scale);
  if (sx >= in_w) sx = in_w - 1;
  if (sy >= in_h) sy = in_h - 1;
  if (flip180) { sx = in_w - 1 - sx; sy = in_h - 1 - sy; }

  const int row = sy * in_w * 2;
  const int base = row + (sx >> 1) * 4;
  const float Y = static_cast<float>(in[base + ((sx & 1) ? 2 : 0)]);
  const float U = static_cast<float>(in[base + 1]);
  const float V = static_cast<float>(in[base + 3]);

  const float c = 1.164f * (Y - 16.0f);
  const float d = U - 128.0f;
  const float e = V - 128.0f;
  float r = c + 1.596f * e;
  float g = c - 0.391f * d - 0.813f * e;
  float b = c + 2.018f * d;
  r = fminf(fmaxf(r, 0.0f), 255.0f);
  g = fminf(fmaxf(g, 0.0f), 255.0f);
  b = fminf(fmaxf(b, 0.0f), 255.0f);

  out[o] = r / 255.f;
  out[o + stride] = g / 255.f;
  out[o + 2 * stride] = b / 255.f;
}

void cuda_preprocess_bgr8(const uint8_t * d_bgr8, int in_w, int in_h,
                          float * d_out, int net_size, int pad_y,
                          float scale_net_to_native, bool flip180,
                          cudaStream_t stream)
{
  dim3 block(16, 16);
  dim3 grid((net_size + 15) / 16, (net_size + 15) / 16);
  k_preprocess_bgr8<<<grid, block, 0, stream>>>(d_bgr8, in_w, in_h, d_out,
                                                 net_size, pad_y,
                                                 scale_net_to_native, flip180);
}

void cuda_preprocess_yuyv(const uint8_t * d_yuyv, int in_w, int in_h,
                          float * d_out, int net_size, int pad_y,
                          float scale_net_to_native, bool flip180,
                          cudaStream_t stream)
{
  dim3 block(16, 16);
  dim3 grid((net_size + 15) / 16, (net_size + 15) / 16);
  k_preprocess_yuyv<<<grid, block, 0, stream>>>(d_yuyv, in_w, in_h, d_out,
                                                 net_size, pad_y,
                                                 scale_net_to_native, flip180);
}

}  // namespace aim
