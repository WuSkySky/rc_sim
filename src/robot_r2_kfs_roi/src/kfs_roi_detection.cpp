#include "robot_r2_kfs_roi/kfs_roi_detection.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace robot_r2_kfs_roi {
namespace {

void validate_hsv_range(const HsvRange &range, const std::string &name) {
  if (range.lower[0] > 179 || range.upper[0] > 179) {
    throw std::invalid_argument(name + " hue must be in [0, 179]");
  }
  for (int channel = 0; channel < 3; ++channel) {
    if (range.lower[channel] > range.upper[channel]) {
      throw std::invalid_argument(name +
                                  " lower bounds must not exceed upper bounds");
    }
  }
}

cv::Scalar as_scalar(const cv::Vec3b &value) {
  return cv::Scalar(value[0], value[1], value[2]);
}

struct ColumnBounds {
  int first{-1};
  int last{-1};
  int max_length{0};
  double threshold{0.0};
};

ColumnBounds find_column_bounds(const cv::Mat &mask, double threshold_ratio,
                                int &mask_area) {
  ColumnBounds bounds;
  cv::Mat column_sums;
  cv::reduce(mask, column_sums, 0, cv::REDUCE_SUM, CV_32S);
  mask_area = 0;
  for (int column = 0; column < mask.cols; ++column) {
    const int length = column_sums.at<int>(0, column) / 255;
    mask_area += length;
    bounds.max_length = std::max(bounds.max_length, length);
  }
  if (bounds.max_length <= 0) {
    return bounds;
  }

  bounds.threshold = bounds.max_length * threshold_ratio;
  for (int column = 0; column < mask.cols; ++column) {
    const int length = column_sums.at<int>(0, column) / 255;
    if (length < bounds.threshold) {
      continue;
    }
    if (bounds.first < 0) {
      bounds.first = column;
    }
    bounds.last = column;
  }
  return bounds;
}

int find_bottom_y(const cv::Mat &mask, int column) {
  for (int row = mask.rows - 1; row >= 0; --row) {
    if (mask.at<std::uint8_t>(row, column) != 0) {
      return row;
    }
  }
  return -1;
}

}  // namespace

void validate_kfs_roi_config(const KfsRoiConfig &config) {
  validate_hsv_range(config.blue, "blue_hsv");
  validate_hsv_range(config.red_low, "red_low_hsv");
  validate_hsv_range(config.red_high, "red_high_hsv");
  if (!std::isfinite(config.column_threshold_ratio) ||
      config.column_threshold_ratio <= 0.0 ||
      config.column_threshold_ratio > 1.0) {
    throw std::invalid_argument(
        "column_threshold_ratio must be finite and in (0, 1]");
  }
  if (config.morphology_kernel_size <= 0 ||
      config.morphology_kernel_size % 2 == 0) {
    throw std::invalid_argument(
        "morphology_kernel_size must be a positive odd integer");
  }
  if (config.min_mask_area_px <= 0) {
    throw std::invalid_argument("min_mask_area_px must be positive");
  }
}

KfsRoiResult extract_kfs_roi(const cv::Mat &bgr_image,
                             const KfsRoiConfig &config) {
  if (bgr_image.empty()) {
    throw std::invalid_argument("ROI source image is empty");
  }
  if (bgr_image.type() != CV_8UC3) {
    throw std::invalid_argument("ROI source image must be 8-bit BGR");
  }
  validate_kfs_roi_config(config);

  cv::Mat hsv;
  cv::cvtColor(bgr_image, hsv, cv::COLOR_BGR2HSV);
  cv::Mat blue_mask;
  cv::Mat red_low_mask;
  cv::Mat red_high_mask;
  cv::inRange(hsv, as_scalar(config.blue.lower),
              as_scalar(config.blue.upper), blue_mask);
  cv::inRange(hsv, as_scalar(config.red_low.lower),
              as_scalar(config.red_low.upper), red_low_mask);
  cv::inRange(hsv, as_scalar(config.red_high.lower),
              as_scalar(config.red_high.upper), red_high_mask);

  cv::Mat combined_mask;
  cv::bitwise_or(red_low_mask, red_high_mask, combined_mask);
  cv::bitwise_or(combined_mask, blue_mask, combined_mask);

  KfsRoiResult result;
  result.raw_mask = combined_mask;
  const cv::Mat kernel = cv::getStructuringElement(
      cv::MORPH_RECT,
      cv::Size(config.morphology_kernel_size, config.morphology_kernel_size));
  cv::morphologyEx(combined_mask, result.opened_mask, cv::MORPH_OPEN, kernel);
  const ColumnBounds horizontal = find_column_bounds(
      result.opened_mask, config.column_threshold_ratio, result.mask_area);
  if (result.mask_area < config.min_mask_area_px) {
    return result;
  }
  if (horizontal.first < 0 || horizontal.last < horizontal.first) {
    return result;
  }

  result.max_column_length = horizontal.max_length;
  result.column_threshold = horizontal.threshold;
  result.x1 = horizontal.first;
  result.x2 = horizontal.last;
  result.left_bottom_y = find_bottom_y(result.opened_mask, result.x1);
  result.right_bottom_y = find_bottom_y(result.opened_mask, result.x2);
  if (result.left_bottom_y < 0 || result.right_bottom_y < 0) {
    return result;
  }

  result.valid = true;
  result.center_u = (result.x1 + result.x2) / 2;
  result.center_offset_x = result.center_u - bgr_image.cols / 2;
  return result;
}

}  // namespace robot_r2_kfs_roi
