#include "robot_r2_kfs_roi/kfs_roi_detection.hpp"

#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>

#include <cstdint>
#include <stdexcept>

namespace robot_r2_kfs_roi {
namespace {

KfsRoiConfig default_config() {
  KfsRoiConfig config;
  config.blue = {cv::Vec3b(105, 100, 80),
                 cv::Vec3b(125, 255, 255)};
  config.red_low = {cv::Vec3b(0, 100, 80),
                    cv::Vec3b(6, 255, 255)};
  config.red_high = {cv::Vec3b(174, 100, 80),
                     cv::Vec3b(179, 255, 255)};
  config.column_threshold_ratio = 0.8;
  config.row_threshold_ratio = 0.8;
  config.morphology_kernel_size = 3;
  config.min_mask_area_px = 100;
  return config;
}

cv::Vec3b bgr_from_hsv(std::uint8_t hue, std::uint8_t saturation,
                       std::uint8_t value) {
  cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(hue, saturation, value));
  cv::Mat bgr;
  cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
  return bgr.at<cv::Vec3b>(0, 0);
}

void fill(cv::Mat &image, const cv::Rect &rect, const cv::Vec3b &color) {
  image(rect).setTo(cv::Scalar(color[0], color[1], color[2]));
}

TEST(KfsRoiDetection, UsesIndependentColumnAndRowProjections) {
  cv::Mat image = cv::Mat::zeros(100, 120, CV_8UC3);
  const cv::Vec3b blue = bgr_from_hsv(115, 255, 255);
  fill(image, cv::Rect(20, 15, 60, 70), blue);
  fill(image, cv::Rect(48, 10, 3, 81), blue);

  const KfsRoiResult result = extract_kfs_roi(image, default_config());

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.x1, 20);
  EXPECT_EQ(result.y1, 15);
  EXPECT_EQ(result.x2, 79);
  EXPECT_EQ(result.y2, 84);
  EXPECT_EQ(result.center_u, 49);
  EXPECT_EQ(result.center_v, 49);
  EXPECT_EQ(result.center_offset_x, -11);
  EXPECT_EQ(result.center_offset_y, -1);
  EXPECT_EQ(result.roi.cols, 60);
  EXPECT_EQ(result.roi.rows, 70);
  EXPECT_FALSE(result.raw_mask.empty());
  EXPECT_FALSE(result.opened_mask.empty());
  EXPECT_EQ(result.mask_area, 4233);
  EXPECT_EQ(result.max_column_length, 81);
  EXPECT_EQ(result.max_row_length, 60);
  EXPECT_DOUBLE_EQ(result.column_threshold, 64.8);
  EXPECT_DOUBLE_EQ(result.row_threshold, 48.0);
}

TEST(KfsRoiDetection, RowThresholdChangesOnlyVerticalBounds) {
  cv::Mat image = cv::Mat::zeros(100, 120, CV_8UC3);
  const cv::Vec3b blue = bgr_from_hsv(115, 255, 255);
  fill(image, cv::Rect(20, 20, 60, 60), blue);
  fill(image, cv::Rect(30, 10, 40, 10), blue);
  fill(image, cv::Rect(30, 80, 40, 10), blue);

  KfsRoiConfig strict_rows = default_config();
  strict_rows.column_threshold_ratio = 0.7;
  strict_rows.row_threshold_ratio = 0.8;
  const KfsRoiResult strict_result =
      extract_kfs_roi(image, strict_rows);

  KfsRoiConfig relaxed_rows = strict_rows;
  relaxed_rows.row_threshold_ratio = 0.6;
  const KfsRoiResult relaxed_result =
      extract_kfs_roi(image, relaxed_rows);

  ASSERT_TRUE(strict_result.valid);
  ASSERT_TRUE(relaxed_result.valid);
  EXPECT_EQ(strict_result.x1, 20);
  EXPECT_EQ(strict_result.x2, 79);
  EXPECT_EQ(relaxed_result.x1, strict_result.x1);
  EXPECT_EQ(relaxed_result.x2, strict_result.x2);
  EXPECT_EQ(strict_result.y1, 20);
  EXPECT_EQ(strict_result.y2, 79);
  EXPECT_EQ(relaxed_result.y1, 10);
  EXPECT_EQ(relaxed_result.y2, 89);
}

TEST(KfsRoiDetection, AcceptsBothRedHueRanges) {
  for (const std::uint8_t hue : {std::uint8_t{3}, std::uint8_t{176}}) {
    cv::Mat image = cv::Mat::zeros(40, 50, CV_8UC3);
    fill(image, cv::Rect(10, 5, 30, 30),
         bgr_from_hsv(hue, 255, 255));

    const KfsRoiResult result = extract_kfs_roi(image, default_config());

    ASSERT_TRUE(result.valid);
    EXPECT_EQ(result.x1, 10);
    EXPECT_EQ(result.y1, 5);
    EXPECT_EQ(result.x2, 39);
    EXPECT_EQ(result.y2, 34);
  }
}

TEST(KfsRoiDetection, AcceptsInclusiveStrictThresholds) {
  const cv::Vec3b colors[] = {
      bgr_from_hsv(105, 255, 255),
      bgr_from_hsv(125, 255, 255),
      bgr_from_hsv(0, 255, 255),
      bgr_from_hsv(6, 255, 255),
      bgr_from_hsv(174, 255, 255),
      bgr_from_hsv(179, 255, 255),
      bgr_from_hsv(115, 100, 255),
      bgr_from_hsv(115, 255, 80),
  };
  for (const cv::Vec3b &color : colors) {
    cv::Mat image(30, 30, CV_8UC3,
                  cv::Scalar(color[0], color[1], color[2]));
    EXPECT_TRUE(extract_kfs_roi(image, default_config()).valid);
  }
}

TEST(KfsRoiDetection, RejectsColorsOutsideStrictThresholds) {
  const cv::Vec3b colors[] = {
      bgr_from_hsv(104, 255, 255),
      bgr_from_hsv(126, 255, 255),
      bgr_from_hsv(115, 99, 255),
      bgr_from_hsv(115, 255, 79),
      bgr_from_hsv(173, 255, 255),
  };
  for (const cv::Vec3b &color : colors) {
    cv::Mat image(30, 30, CV_8UC3,
                  cv::Scalar(color[0], color[1], color[2]));
    EXPECT_FALSE(extract_kfs_roi(image, default_config()).valid);
  }
}

TEST(KfsRoiDetection, RemovesIsolatedNoiseBeforeProjection) {
  cv::Mat image = cv::Mat::zeros(100, 120, CV_8UC3);
  const cv::Vec3b blue = bgr_from_hsv(115, 255, 255);
  const cv::Vec3b red = bgr_from_hsv(3, 255, 255);
  fill(image, cv::Rect(20, 20, 40, 40), blue);
  fill(image, cv::Rect(90, 10, 15, 15), red);
  image.at<cv::Vec3b>(5, 5) = blue;
  image.at<cv::Vec3b>(80, 110) = red;

  const KfsRoiResult result = extract_kfs_roi(image, default_config());

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.x1, 20);
  EXPECT_EQ(result.y1, 20);
  EXPECT_EQ(result.x2, 59);
  EXPECT_EQ(result.y2, 59);
  EXPECT_EQ(cv::countNonZero(result.raw_mask), 1827);
  EXPECT_EQ(cv::countNonZero(result.opened_mask), 1825);
}

TEST(KfsRoiDetection, RejectsMaskBelowMinimumArea) {
  cv::Mat image = cv::Mat::zeros(30, 30, CV_8UC3);
  fill(image, cv::Rect(5, 5, 9, 9), bgr_from_hsv(115, 255, 255));

  const KfsRoiResult result = extract_kfs_roi(image, default_config());

  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(result.roi.empty());
  EXPECT_EQ(result.mask_area, 81);
  EXPECT_EQ(cv::countNonZero(result.opened_mask), 81);
}

TEST(KfsRoiDetection, RejectsEmptyImageAndInvalidConfiguration) {
  EXPECT_THROW(extract_kfs_roi(cv::Mat(), default_config()),
               std::invalid_argument);

  KfsRoiConfig config = default_config();
  config.morphology_kernel_size = 2;
  EXPECT_THROW(validate_kfs_roi_config(config), std::invalid_argument);
  config = default_config();
  config.min_mask_area_px = 0;
  EXPECT_THROW(validate_kfs_roi_config(config), std::invalid_argument);
  config = default_config();
  config.column_threshold_ratio = 1.1;
  EXPECT_THROW(validate_kfs_roi_config(config), std::invalid_argument);
  config = default_config();
  config.row_threshold_ratio = 0.0;
  EXPECT_THROW(validate_kfs_roi_config(config), std::invalid_argument);
}

}  // namespace
}  // namespace robot_r2_kfs_roi
