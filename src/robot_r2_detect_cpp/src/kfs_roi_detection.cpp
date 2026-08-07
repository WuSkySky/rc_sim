#include "robot_r2_detect_cpp/kfs_roi_detection.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace robot_r2_detect_cpp {
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

struct AxisBounds {
  int first{-1};
  int last{-1};
  int max_length{0};
  double threshold{0.0};
};

AxisBounds find_axis_bounds(const cv::Mat &mask, bool use_columns,
                            double threshold_ratio) {
  AxisBounds bounds;
  const int axis_length = use_columns ? mask.cols : mask.rows;
  const auto projection_length = [&mask, use_columns](const int index) {
    return use_columns ? cv::countNonZero(mask.col(index))
                       : cv::countNonZero(mask.row(index));
  };

  for (int index = 0; index < axis_length; ++index) {
    bounds.max_length =
        std::max(bounds.max_length, projection_length(index));
  }
  if (bounds.max_length <= 0) {
    return bounds;
  }

  bounds.threshold = bounds.max_length * threshold_ratio;
  for (int index = 0; index < axis_length; ++index) {
    if (projection_length(index) < bounds.threshold) {
      continue;
    }
    if (bounds.first < 0) {
      bounds.first = index;
    }
    bounds.last = index;
  }
  return bounds;
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
  if (!std::isfinite(config.row_threshold_ratio) ||
      config.row_threshold_ratio <= 0.0 ||
      config.row_threshold_ratio > 1.0) {
    throw std::invalid_argument(
        "row_threshold_ratio must be finite and in (0, 1]");
  }
  if (config.morphology_kernel_size <= 0 ||
      config.morphology_kernel_size % 2 == 0) {
    throw std::invalid_argument(
        "morphology_kernel_size must be a positive odd integer");
  }
  if (config.min_component_area_px <= 0) {
    throw std::invalid_argument("min_component_area_px must be positive");
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
  result.mask = cv::Mat::zeros(combined_mask.size(), CV_8UC1);

  cv::Mat labels;
  cv::Mat stats;
  cv::Mat centroids;
  const int label_count = cv::connectedComponentsWithStats(
      result.opened_mask, labels, stats, centroids, 8, CV_32S);
  if (label_count <= 1) {
    return result;
  }

  int largest_label = 1;
  int largest_area = stats.at<int>(1, cv::CC_STAT_AREA);
  for (int label = 2; label < label_count; ++label) {
    const int area = stats.at<int>(label, cv::CC_STAT_AREA);
    if (area > largest_area) {
      largest_label = label;
      largest_area = area;
    }
  }

  cv::compare(labels, cv::Scalar(largest_label), result.mask, cv::CMP_EQ);
  result.component_area = largest_area;
  if (largest_area < config.min_component_area_px) {
    return result;
  }

  const AxisBounds horizontal = find_axis_bounds(
      result.mask, true, config.column_threshold_ratio);
  const AxisBounds vertical =
      find_axis_bounds(result.mask, false, config.row_threshold_ratio);
  result.max_column_length = horizontal.max_length;
  result.max_row_length = vertical.max_length;
  result.column_threshold = horizontal.threshold;
  result.row_threshold = vertical.threshold;
  if (horizontal.first < 0 || horizontal.last < horizontal.first ||
      vertical.first < 0 || vertical.last < vertical.first) {
    return result;
  }

  result.valid = true;
  result.x1 = horizontal.first;
  result.y1 = vertical.first;
  result.x2 = horizontal.last;
  result.y2 = vertical.last;
  result.center_u = (result.x1 + result.x2) / 2;
  result.center_v = (result.y1 + result.y2) / 2;
  result.center_offset_x = result.center_u - bgr_image.cols / 2;
  result.center_offset_y = result.center_v - bgr_image.rows / 2;
  result.roi = bgr_image(cv::Rect(result.x1, result.y1,
                                  result.x2 - result.x1 + 1,
                                  result.y2 - result.y1 + 1))
                   .clone();
  return result;
}

}  // namespace robot_r2_detect_cpp
