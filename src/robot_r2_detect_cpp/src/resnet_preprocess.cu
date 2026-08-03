#include "robot_r2_detect_cpp/resnet_preprocess.hpp"

#include <cuda_fp16.h>
#include <cuda_runtime.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace robot_r2_detect_cpp {
namespace {

template <typename OutputT>
__device__ inline OutputT convert_output(float value);

template <> __device__ inline float convert_output<float>(float value) {
  return value;
}

template <> __device__ inline __half convert_output<__half>(float value) {
  return __float2half(value);
}

__device__ inline float read_channel(const std::uint8_t *source, int width,
                                     int height, std::size_t step, int channels,
                                     int channel, int x, int y) {
  x = max(0, min(width - 1, x));
  y = max(0, min(height - 1, y));
  const auto *row = source + static_cast<std::size_t>(y) * step;
  if (channels == 1) {
    return static_cast<float>(row[x]);
  }
  return static_cast<float>(row[x * channels + channel]);
}

template <typename OutputT>
__global__ void preprocess_kernel(const std::uint8_t *source, int source_width,
                                  int source_height, std::size_t source_step,
                                  int source_channels, bool source_is_rgb,
                                  OutputT *destination, int image_size,
                                  float mean_0, float mean_1, float mean_2,
                                  float std_0, float std_1, float std_2) {
  const int output_x = blockIdx.x * blockDim.x + threadIdx.x;
  const int output_y = blockIdx.y * blockDim.y + threadIdx.y;
  if (output_x >= image_size || output_y >= image_size) {
    return;
  }

  // Match torchvision Resize(round(image_size / 0.875)) + CenterCrop.
  const int resize_short = static_cast<int>(roundf(image_size / 0.875f));
  const float scale = static_cast<float>(resize_short) /
                      static_cast<float>(min(source_width, source_height));
  const int resized_width =
      max(image_size, static_cast<int>(roundf(source_width * scale)));
  const int resized_height =
      max(image_size, static_cast<int>(roundf(source_height * scale)));
  const int crop_left = (resized_width - image_size) / 2;
  const int crop_top = (resized_height - image_size) / 2;
  const float resize_scale_x = static_cast<float>(resized_width) / source_width;
  const float resize_scale_y =
      static_cast<float>(resized_height) / source_height;

  // OpenCV INTER_LINEAR uses half-pixel coordinates for resize.
  const float source_x =
      (static_cast<float>(output_x + crop_left) + 0.5f) / resize_scale_x - 0.5f;
  const float source_y =
      (static_cast<float>(output_y + crop_top) + 0.5f) / resize_scale_y - 0.5f;
  const int x0 = static_cast<int>(floorf(source_x));
  const int y0 = static_cast<int>(floorf(source_y));
  const int x1 = x0 + 1;
  const int y1 = y0 + 1;
  const float wx = source_x - floorf(source_x);
  const float wy = source_y - floorf(source_y);

  const float means[3] = {mean_0, mean_1, mean_2};
  const float stds[3] = {std_0, std_1, std_2};
  const int plane = image_size * image_size;
  const int pixel = output_y * image_size + output_x;

  for (int output_channel = 0; output_channel < 3; ++output_channel) {
    int source_channel = 0;
    if (source_channels == 3) {
      source_channel = source_is_rgb ? output_channel : 2 - output_channel;
    }
    const float top_left =
        read_channel(source, source_width, source_height, source_step,
                     source_channels, source_channel, x0, y0);
    const float top_right =
        read_channel(source, source_width, source_height, source_step,
                     source_channels, source_channel, x1, y0);
    const float bottom_left =
        read_channel(source, source_width, source_height, source_step,
                     source_channels, source_channel, x0, y1);
    const float bottom_right =
        read_channel(source, source_width, source_height, source_step,
                     source_channels, source_channel, x1, y1);
    const float top = top_left + wx * (top_right - top_left);
    const float bottom = bottom_left + wx * (bottom_right - bottom_left);
    const float value = (top + wy * (bottom - top)) / 255.0f;
    destination[output_channel * plane + pixel] = convert_output<OutputT>(
        (value - means[output_channel]) / stds[output_channel]);
  }
}

void check_launch(const char *operation) {
  const cudaError_t status = cudaGetLastError();
  if (status != cudaSuccess) {
    throw std::runtime_error(std::string(operation) + ": " +
                             cudaGetErrorString(status));
  }
}

} // namespace

void launch_resnet_preprocess(const std::uint8_t *source, int source_width,
                              int source_height, std::size_t source_step,
                              int source_channels, bool source_is_rgb,
                              void *destination,
                              TensorElementType destination_type,
                              int image_size, const float mean[3],
                              const float std[3], cudaStream_t stream) {
  if (source == nullptr || destination == nullptr || source_width <= 0 ||
      source_height <= 0 || image_size <= 0 ||
      (source_channels != 1 && source_channels != 3)) {
    throw std::invalid_argument("invalid CUDA preprocessing arguments");
  }

  const dim3 block(16, 16);
  const dim3 grid((image_size + block.x - 1) / block.x,
                  (image_size + block.y - 1) / block.y);
  if (destination_type == TensorElementType::kFloat32) {
    preprocess_kernel<float><<<grid, block, 0, stream>>>(
        source, source_width, source_height, source_step, source_channels,
        source_is_rgb, static_cast<float *>(destination), image_size, mean[0],
        mean[1], mean[2], std[0], std[1], std[2]);
  } else {
    preprocess_kernel<__half><<<grid, block, 0, stream>>>(
        source, source_width, source_height, source_step, source_channels,
        source_is_rgb, static_cast<__half *>(destination), image_size, mean[0],
        mean[1], mean[2], std[0], std[1], std[2]);
  }
  check_launch("ResNet preprocessing kernel launch failed");
}

} // namespace robot_r2_detect_cpp
