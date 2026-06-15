// CUDA letterbox + normalize preprocess for the RealSense detector path.
//
// Mirrors the kernel baked into zed_detector.py so the C++ RealSense path and
// the Python ZED path feed byte-identical tensors to the same TensorRT engine:
//   - letterbox a landscape color image into a square engine input,
//   - pad the top/bottom bands with 0.447 (the gray Ultralytics pad value),
//   - write normalized RGB in NCHW layout (R,G,B planes), value = channel/255.
//
// The only difference from the ZED kernel is the source pixel stride: ZED
// retrieves BGRA (4 bytes/px); the RealSense color stream is requested as
// BGR8 (3 bytes/px).
#pragma once

#include <cuda_runtime.h>

// in:  device pointer to interleaved BGR8 image (in_w * in_h * 3 bytes)
// out: device pointer to float CHW tensor [3, out_h, out_w]
// pad_y / unscaled_h / scale: precomputed letterbox geometry (see the host node)
void launch_preprocess_bgr8(const unsigned char* in, float* out,
                            int in_w, int in_h, int out_w, int out_h,
                            int pad_y, int unscaled_h, float scale,
                            cudaStream_t stream);
