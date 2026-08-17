#include "robot_r2_kfs_roi/kfs_roi_detection.hpp"

#include <opencv2/imgproc.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_r2_interfaces/msg/camera_frame.hpp>
#include <robot_r2_interfaces/msg/alignment_detection.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace robot_r2_kfs_roi {
namespace {

using CameraFrame = robot_r2_interfaces::msg::CameraFrame;
using AlignmentDetection = robot_r2_interfaces::msg::AlignmentDetection;

constexpr char kInputTopic[] = "/r2/front_camera/image_raw";
constexpr char kRoiTopic[] = "/r2/kfs/roi";
constexpr char kDebugTopic[] = "/r2/kfs/roi/debug";

struct NodeConfig {
  KfsRoiConfig roi;
  double target_processing_rate{30.0};
  bool visualization_enabled{false};
  int center_offset_px{0};
};

struct FrameView {
  const std::uint8_t *data{};
  int width{};
  int height{};
  std::size_t step{};
  int channels{};
  bool is_rgb{};
};

cv::Vec3b hsv_value(const std::vector<std::int64_t> &values,
                    const std::string &name) {
  if (values.size() != 3) {
    throw std::invalid_argument(name + " must contain [h, s, v]");
  }
  if (std::any_of(values.begin(), values.end(),
                  [](const std::int64_t value) {
                    return value < 0 || value > 255;
                  })) {
    throw std::invalid_argument(name + " values must be in [0, 255]");
  }
  return cv::Vec3b(static_cast<std::uint8_t>(values[0]),
                   static_cast<std::uint8_t>(values[1]),
                   static_cast<std::uint8_t>(values[2]));
}

constexpr int kMinCenterOffsetPx = -1000000;
constexpr int kMaxCenterOffsetPx = 1000000;

int positive_int(std::int64_t value, const std::string &name) {
  if (value <= 0 || value > std::numeric_limits<int>::max()) {
    throw std::invalid_argument(name + " must be a positive integer");
  }
  return static_cast<int>(value);
}

void validate_node_config(const NodeConfig &config) {
  validate_kfs_roi_config(config.roi);
  if (!std::isfinite(config.target_processing_rate) ||
      config.target_processing_rate <= 0.0) {
    throw std::invalid_argument(
        "target_processing_rate must be finite and positive");
  }
  if (config.center_offset_px < kMinCenterOffsetPx ||
      config.center_offset_px > kMaxCenterOffsetPx) {
    throw std::invalid_argument(
        "center_offset_px must be within +/-1000000 pixels");
  }
}

FrameView validate_frame(const CameraFrame &message) {
  if (message.layout_version != CameraFrame::LAYOUT_VERSION) {
    throw std::invalid_argument("unsupported CameraFrame layout_version");
  }
  if (message.width == 0 || message.height == 0) {
    throw std::invalid_argument("CameraFrame width/height must be positive");
  }
  if (message.width > static_cast<std::uint32_t>(
                          std::numeric_limits<int>::max()) ||
      message.height > static_cast<std::uint32_t>(
                           std::numeric_limits<int>::max())) {
    throw std::invalid_argument("CameraFrame dimensions exceed OpenCV limits");
  }
  if (message.is_bigendian > 1) {
    throw std::invalid_argument("CameraFrame is_bigendian must be 0 or 1");
  }

  int channels = 0;
  bool is_rgb = false;
  if (message.encoding == CameraFrame::ENCODING_BGR8) {
    channels = 3;
  } else if (message.encoding == CameraFrame::ENCODING_RGB8) {
    channels = 3;
    is_rgb = true;
  } else if (message.encoding == CameraFrame::ENCODING_MONO8) {
    channels = 1;
  } else {
    throw std::invalid_argument("unsupported CameraFrame encoding");
  }

  const std::size_t row_bytes =
      static_cast<std::size_t>(message.width) * channels;
  const std::size_t expected_size =
      static_cast<std::size_t>(message.height) * message.step;
  if (message.step < row_bytes || message.data_size != expected_size ||
      message.data.size() != expected_size ||
      expected_size > CameraFrame::DATA_CAPACITY) {
    throw std::invalid_argument("invalid CameraFrame step or data_size");
  }
  if (message.stamp_nanosec >= 1000000000U) {
    throw std::invalid_argument("invalid CameraFrame nanosecond timestamp");
  }
  if (message.frame_id_size > CameraFrame::FRAME_ID_CAPACITY) {
    throw std::invalid_argument("invalid CameraFrame frame_id_size");
  }

  return FrameView{std::addressof(message.data.front()),
                   static_cast<int>(message.width),
                   static_cast<int>(message.height), message.step, channels,
                   is_rgb};
}

cv::Mat to_bgr(const CameraFrame &message) {
  const FrameView view = validate_frame(message);
  cv::Mat source(view.height, view.width,
                 view.channels == 3 ? CV_8UC3 : CV_8UC1,
                 const_cast<std::uint8_t *>(view.data), view.step);
  if (view.channels == 1) {
    cv::Mat bgr;
    cv::cvtColor(source, bgr, cv::COLOR_GRAY2BGR);
    return bgr;
  }
  if (view.is_rgb) {
    cv::Mat bgr;
    cv::cvtColor(source, bgr, cv::COLOR_RGB2BGR);
    return bgr;
  }
  return source;
}

std_msgs::msg::Header make_header(const CameraFrame &frame) {
  std_msgs::msg::Header header;
  header.stamp.sec = frame.stamp_sec;
  header.stamp.nanosec = frame.stamp_nanosec;
  header.frame_id.assign(
      reinterpret_cast<const char *>(frame.frame_id.data()),
      frame.frame_id_size);
  return header;
}

sensor_msgs::msg::Image make_image(const cv::Mat &bgr,
                                   const std_msgs::msg::Header &header) {
  if (bgr.empty() || bgr.type() != CV_8UC3) {
    throw std::invalid_argument("debug image must be 8-bit BGR");
  }
  const cv::Mat packed = bgr.isContinuous() ? bgr : bgr.clone();
  sensor_msgs::msg::Image output;
  output.header = header;
  output.height = static_cast<std::uint32_t>(packed.rows);
  output.width = static_cast<std::uint32_t>(packed.cols);
  output.encoding = "bgr8";
  output.is_bigendian = 0;
  output.step = static_cast<std::uint32_t>(packed.cols * 3);
  const std::size_t data_size = packed.total() * packed.elemSize();
  output.data.assign(packed.data, packed.data + data_size);
  return output;
}

cv::Mat mask_to_bgr(const cv::Mat &mask) {
  cv::Mat view;
  cv::cvtColor(mask, view, cv::COLOR_GRAY2BGR);
  return view;
}

void draw_legend_item(cv::Mat &tile, int x, int y,
                      const cv::Scalar &color, const std::string &label,
                      bool marker = false) {
  constexpr int kSymbolWidth = 18;
  if (marker) {
    cv::drawMarker(tile, cv::Point(x + kSymbolWidth / 2, y - 4), color,
                   cv::MARKER_CROSS, 13, 2);
  } else {
    cv::line(tile, cv::Point(x, y - 4),
             cv::Point(x + kSymbolWidth, y - 4), color, 3);
  }
  cv::putText(tile, label, cv::Point(x + kSymbolWidth + 5, y),
              cv::FONT_HERSHEY_SIMPLEX, 0.38, cv::Scalar(255, 255, 255), 1,
              cv::LINE_AA);
}

void draw_stage_legend(cv::Mat &tile, const std::string &title,
                       const std::string &metrics) {
  const int label_height = std::min(96, tile.rows);
  cv::rectangle(tile, cv::Rect(0, 0, tile.cols, label_height),
                cv::Scalar(0, 0, 0), cv::FILLED);
  if (tile.rows < 84 || tile.cols < 280) {
    return;
  }
  cv::putText(tile, title, cv::Point(10, 22), cv::FONT_HERSHEY_SIMPLEX,
              0.58, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
  cv::putText(tile, metrics, cv::Point(10, 43), cv::FONT_HERSHEY_SIMPLEX,
              0.42, cv::Scalar(210, 210, 210), 1, cv::LINE_AA);

  const int third = tile.cols / 3;
  draw_legend_item(tile, 10, 66, cv::Scalar(255, 255, 0), "image ctr");
  draw_legend_item(tile, third, 66, cv::Scalar(0, 0, 255),
                   "adjusted ctr");
  draw_legend_item(tile, third * 2, 66, cv::Scalar(0, 255, 0), "ROI bounds");
  draw_legend_item(tile, 10, 88, cv::Scalar(255, 0, 255), "left bottom",
                   true);
  draw_legend_item(tile, third, 88, cv::Scalar(0, 255, 255), "right bottom",
                   true);
}

cv::Mat make_visualization(const cv::Mat &image,
                           const KfsRoiResult &result,
                           int center_offset_px) {
  cv::Mat source_view = image.clone();
  cv::Mat columns_view = mask_to_bgr(result.opened_mask);
  if (result.valid) {
    const int adjusted_center = result.center_u + center_offset_px;
    const auto draw_geometry = [&](cv::Mat &view) {
      // Image center (cyan), offset-adjusted KFS center (red), and qualified
      // column bounds (green).
      cv::line(view, cv::Point(image.cols / 2, 0),
               cv::Point(image.cols / 2, image.rows - 1),
               cv::Scalar(255, 255, 0), 2);
      cv::line(view, cv::Point(adjusted_center, 0),
               cv::Point(adjusted_center, image.rows - 1),
               cv::Scalar(0, 0, 255), 2);
      cv::line(view, cv::Point(result.x1, 0),
               cv::Point(result.x1, image.rows - 1),
               cv::Scalar(0, 255, 0), 2);
      cv::line(view, cv::Point(result.x2, 0),
               cv::Point(result.x2, image.rows - 1),
               cv::Scalar(0, 255, 0), 2);
      cv::drawMarker(view, cv::Point(result.x1, result.left_bottom_y),
                     cv::Scalar(255, 0, 255), cv::MARKER_CROSS, 18, 2);
      cv::drawMarker(view, cv::Point(result.x2, result.right_bottom_y),
                     cv::Scalar(0, 255, 255), cv::MARKER_CROSS, 18, 2);
    };
    draw_geometry(source_view);
    draw_geometry(columns_view);
  }

  std::string source_metrics = "ROI not found";
  std::string columns_metrics = "ROI not found";
  if (result.valid) {
    std::ostringstream source_stream;
    source_stream << "detected=" << std::showpos << result.center_offset_x
                  << " px  shift=" << center_offset_px << " px  output="
                  << (result.center_offset_x + center_offset_px)
                  << std::noshowpos << " px";
    source_metrics = source_stream.str();
    columns_metrics =
        cv::format("area=%d  threshold=%.1f/%d", result.mask_area,
                   result.column_threshold, result.max_column_length);
  }

  const std::vector<std::pair<cv::Mat, std::string>> stages{
      {source_view, source_metrics},
      {columns_view, columns_metrics},
  };

  const int tile_width = std::max(1, image.cols / 2);
  cv::Mat canvas(image.rows, tile_width * 2, CV_8UC3, cv::Scalar(0, 0, 0));
  for (std::size_t index = 0; index < stages.size(); ++index) {
    cv::Mat tile;
    cv::resize(stages[index].first, tile,
               cv::Size(tile_width, image.rows), 0.0, 0.0,
               index == 0 ? cv::INTER_AREA : cv::INTER_NEAREST);
    draw_stage_legend(tile, index == 0 ? "1 Source" : "4 Columns",
                      stages[index].second);
    tile.copyTo(canvas(cv::Rect(static_cast<int>(index) * tile_width, 0,
                                tile_width, image.rows)));
  }
  return canvas;
}

class KfsRoiNode final : public rclcpp::Node {
 public:
  KfsRoiNode() : Node("kfs_roi") {
    declare_parameter<double>("target_processing_rate", 30.0);
    declare_parameter<bool>("visualization_enabled", false);
    declare_parameter<std::vector<std::int64_t>>(
        "blue_hsv_lower", {95, 60, 20});
    declare_parameter<std::vector<std::int64_t>>(
        "blue_hsv_upper", {135, 255, 255});
    declare_parameter<std::vector<std::int64_t>>(
        "red_low_hsv_lower", {0, 60, 20});
    declare_parameter<std::vector<std::int64_t>>(
        "red_low_hsv_upper", {15, 255, 255});
    declare_parameter<std::vector<std::int64_t>>(
        "red_high_hsv_lower", {165, 60, 20});
    declare_parameter<std::vector<std::int64_t>>(
        "red_high_hsv_upper", {179, 255, 255});
    declare_parameter<double>("column_threshold_ratio", 0.7);
    declare_parameter<std::int64_t>("morphology_kernel_size", 5);
    declare_parameter<std::int64_t>("min_mask_area_px", 100);
    declare_parameter<std::int64_t>("center_offset_px", 0);

    config_ = read_config();
    validate_node_config(config_);

    const auto image_qos = rclcpp::SensorDataQoS().keep_last(1);
    roi_publisher_ = create_publisher<AlignmentDetection>(kRoiTopic, image_qos);
    debug_publisher_ =
        create_publisher<sensor_msgs::msg::Image>(kDebugTopic, image_qos);
    subscription_ = create_subscription<CameraFrame>(
        kInputTopic, image_qos,
        std::bind(&KfsRoiNode::on_image, this, std::placeholders::_1));
    parameter_callback_ = add_on_set_parameters_callback(
        std::bind(&KfsRoiNode::on_parameters_changed, this,
                  std::placeholders::_1));
  }

 private:
  NodeConfig read_config() const {
    NodeConfig config;
    config.target_processing_rate =
        get_parameter("target_processing_rate").as_double();
    config.visualization_enabled =
        get_parameter("visualization_enabled").as_bool();
    config.roi.blue = {
        hsv_value(get_parameter("blue_hsv_lower").as_integer_array(),
                  "blue_hsv_lower"),
        hsv_value(get_parameter("blue_hsv_upper").as_integer_array(),
                  "blue_hsv_upper")};
    config.roi.red_low = {
        hsv_value(get_parameter("red_low_hsv_lower").as_integer_array(),
                  "red_low_hsv_lower"),
        hsv_value(get_parameter("red_low_hsv_upper").as_integer_array(),
                  "red_low_hsv_upper")};
    config.roi.red_high = {
        hsv_value(get_parameter("red_high_hsv_lower").as_integer_array(),
                  "red_high_hsv_lower"),
        hsv_value(get_parameter("red_high_hsv_upper").as_integer_array(),
                  "red_high_hsv_upper")};
    config.roi.column_threshold_ratio =
        get_parameter("column_threshold_ratio").as_double();
    config.roi.morphology_kernel_size = positive_int(
        get_parameter("morphology_kernel_size").as_int(),
        "morphology_kernel_size");
    config.roi.min_mask_area_px = positive_int(
        get_parameter("min_mask_area_px").as_int(), "min_mask_area_px");
    config.center_offset_px =
        static_cast<int>(get_parameter("center_offset_px").as_int());
    return config;
  }

  rcl_interfaces::msg::SetParametersResult on_parameters_changed(
      const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    try {
      std::lock_guard<std::mutex> lock(config_mutex_);
      NodeConfig candidate = config_;
      for (const auto &parameter : parameters) {
        apply_parameter(candidate, parameter);
      }
      validate_node_config(candidate);
      config_ = candidate;
      result.successful = true;
    } catch (const std::exception &error) {
      result.reason = error.what();
    }
    return result;
  }

  static void apply_parameter(NodeConfig &config,
                              const rclcpp::Parameter &parameter) {
    const std::string &name = parameter.get_name();
    if (name == "target_processing_rate") {
      config.target_processing_rate = parameter.as_double();
    } else if (name == "visualization_enabled") {
      config.visualization_enabled = parameter.as_bool();
    } else if (name == "blue_hsv_lower") {
      config.roi.blue.lower = hsv_value(parameter.as_integer_array(), name);
    } else if (name == "blue_hsv_upper") {
      config.roi.blue.upper = hsv_value(parameter.as_integer_array(), name);
    } else if (name == "red_low_hsv_lower") {
      config.roi.red_low.lower = hsv_value(parameter.as_integer_array(), name);
    } else if (name == "red_low_hsv_upper") {
      config.roi.red_low.upper = hsv_value(parameter.as_integer_array(), name);
    } else if (name == "red_high_hsv_lower") {
      config.roi.red_high.lower =
          hsv_value(parameter.as_integer_array(), name);
    } else if (name == "red_high_hsv_upper") {
      config.roi.red_high.upper =
          hsv_value(parameter.as_integer_array(), name);
    } else if (name == "column_threshold_ratio") {
      config.roi.column_threshold_ratio = parameter.as_double();
    } else if (name == "morphology_kernel_size") {
      config.roi.morphology_kernel_size =
          positive_int(parameter.as_int(), name);
    } else if (name == "min_mask_area_px") {
      config.roi.min_mask_area_px =
          positive_int(parameter.as_int(), name);
    } else if (name == "center_offset_px") {
      config.center_offset_px = static_cast<int>(parameter.as_int());
    }
  }

  void on_image(const CameraFrame::ConstSharedPtr message) {
    const auto started_at = std::chrono::steady_clock::now();
    NodeConfig config;
    {
      std::lock_guard<std::mutex> lock(config_mutex_);
      config = config_;
    }

    try {
      const cv::Mat image = to_bgr(*message);
      const KfsRoiResult roi = extract_kfs_roi(image, config.roi);
      roi_publisher_->publish(make_roi_message(
          *message, image, roi, config.center_offset_px));
      if (config.visualization_enabled) {
        const auto header = make_header(*message);
        debug_publisher_->publish(make_image(
            make_visualization(image, roi, config.center_offset_px),
            header));
      }
    } catch (const cv::Exception &error) {
      RCLCPP_ERROR(get_logger(), "Failed to extract KFS ROI: %s",
                   error.what());
      return;
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "Failed to extract KFS ROI: %s",
                   error.what());
      return;
    }

    const double processing_time = std::chrono::duration<double>(
                                       std::chrono::steady_clock::now() -
                                       started_at)
                                       .count();
    const double deadline = 1.0 / config.target_processing_rate;
    if (processing_time > deadline) {
      RCLCPP_WARN(get_logger(),
                  "KFS ROI processing overrun: %.2f ms > %.2f ms "
                  "(target %g Hz)",
                  processing_time * 1000.0, deadline * 1000.0,
                  config.target_processing_rate);
    }
  }

  static AlignmentDetection make_roi_message(const CameraFrame &source,
                                               const cv::Mat &image,
                                               const KfsRoiResult &result,
                                               int center_offset_px) {
    AlignmentDetection message;
    message.header = make_header(source);
    message.sequence = source.sequence;
    message.valid = result.valid;
    message.image_width = image.cols;
    message.image_height = image.rows;
    if (!result.valid) {
      return message;
    }

    message.x1 = result.x1;
    message.x2 = result.x2;
    message.left_bottom_y = result.left_bottom_y;
    message.right_bottom_y = result.right_bottom_y;
    // The dynamic center_offset_px shifts the final KFS center before it
    // reaches the alignment consumers.
    message.center_u = result.center_u + center_offset_px;
    message.center_offset_x = result.center_offset_x + center_offset_px;
    return message;
  }

  std::mutex config_mutex_;
  NodeConfig config_;
  rclcpp::Publisher<AlignmentDetection>::SharedPtr roi_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_publisher_;
  rclcpp::Subscription<CameraFrame>::SharedPtr subscription_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameter_callback_;
};

}  // namespace

std::shared_ptr<rclcpp::Node> make_kfs_roi_node() {
  return std::make_shared<KfsRoiNode>();
}

}  // namespace robot_r2_kfs_roi

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(robot_r2_kfs_roi::make_kfs_roi_node());
  rclcpp::shutdown();
  return 0;
}
