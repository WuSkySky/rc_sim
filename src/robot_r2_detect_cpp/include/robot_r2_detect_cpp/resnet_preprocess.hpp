#pragma once

#include <cuda_runtime_api.h>

#include <cstddef>
#include <cstdint>

namespace robot_r2_detect_cpp {

enum class TensorElementType {
  kFloat32,
  kFloat16,
};

// Launches Resize(short side) + CenterCrop + BGR/RGB/mono to RGB +
// ImageNet normalization directly on the GPU. The destination is one NCHW
// tensor slot and may be float32 or float16.
void launch_resnet_preprocess(const std::uint8_t *source, int source_width,
                              int source_height, std::size_t source_step,
                              int source_channels, bool source_is_rgb,
                              void *destination,
                              TensorElementType destination_type,
                              int image_size, const float mean[3],
                              const float std[3], cudaStream_t stream);

} // namespace robot_r2_detect_cpp
