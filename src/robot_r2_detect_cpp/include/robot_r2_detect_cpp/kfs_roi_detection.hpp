#pragma once

#include <opencv2/core.hpp>

namespace robot_r2_detect_cpp {

struct HsvRange {
  cv::Vec3b lower;
  cv::Vec3b upper;
};

struct KfsRoiConfig {
  HsvRange blue{cv::Vec3b(105, 100, 80), cv::Vec3b(125, 255, 255)};
  HsvRange red_low{cv::Vec3b(0, 100, 80), cv::Vec3b(6, 255, 255)};
  HsvRange red_high{cv::Vec3b(174, 100, 80),
                    cv::Vec3b(179, 255, 255)};
  double column_threshold_ratio{0.8};
  double row_threshold_ratio{0.8};
  int morphology_kernel_size{3};
  int min_component_area_px{100};
};

struct KfsRoiResult {
  bool valid{false};
  cv::Mat raw_mask;
  cv::Mat opened_mask;
  cv::Mat mask;
  cv::Mat roi;
  int component_area{0};
  int max_column_length{0};
  int max_row_length{0};
  double column_threshold{0.0};
  double row_threshold{0.0};
  int x1{0};
  int y1{0};
  int x2{0};
  int y2{0};
  int center_u{0};
  int center_v{0};
  int center_offset_x{0};
  int center_offset_y{0};
};

void validate_kfs_roi_config(const KfsRoiConfig &config);

KfsRoiResult extract_kfs_roi(const cv::Mat &bgr_image,
                             const KfsRoiConfig &config);

}  // namespace robot_r2_detect_cpp
