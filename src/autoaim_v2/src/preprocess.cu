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

}  // namespace aim
