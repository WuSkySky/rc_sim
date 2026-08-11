#pragma once

#include <opencv2/core.hpp>

namespace robot_r2_kfs_roi {

struct HsvRange {
  cv::Vec3b lower;
  cv::Vec3b upper;
};

struct KfsRoiConfig {
  HsvRange blue{cv::Vec3b(95, 60, 20), cv::Vec3b(135, 255, 255)};
  HsvRange red_low{cv::Vec3b(0, 60, 20), cv::Vec3b(15, 255, 255)};
  HsvRange red_high{cv::Vec3b(165, 60, 20),
                    cv::Vec3b(179, 255, 255)};
  double column_threshold_ratio{0.7};
  int morphology_kernel_size{5};
  int min_mask_area_px{100};
};

struct KfsRoiResult {
  bool valid{false};
  cv::Mat raw_mask;
  cv::Mat opened_mask;
  int mask_area{0};
  int max_column_length{0};
  double column_threshold{0.0};
  int x1{0};
  int x2{0};
  int left_bottom_y{0};
  int right_bottom_y{0};
  int center_u{0};
  int center_offset_x{0};
};

void validate_kfs_roi_config(const KfsRoiConfig &config);

KfsRoiResult extract_kfs_roi(const cv::Mat &bgr_image,
                             const KfsRoiConfig &config);

}  // namespace robot_r2_kfs_roi
