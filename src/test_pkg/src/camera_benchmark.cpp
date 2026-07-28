#include <sys/resource.h>
#include <sys/statvfs.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

#include "rclcpp/rclcpp.hpp"
#include "robot_r2_interfaces/msg/camera_frame.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace
{

using CameraFrame = robot_r2_interfaces::msg::CameraFrame;
using SteadyClock = std::chrono::steady_clock;

struct Distribution
{
  double average{0.0};
  double p95{0.0};
  double maximum{0.0};
};

Distribution summarize(const std::vector<double> & values)
{
  if (values.empty()) {
    return {};
  }

  double sum = 0.0;
  double maximum = values.front();
  for (const auto value : values) {
    sum += value;
    maximum = std::max(maximum, value);
  }

  auto sorted = values;
  std::sort(sorted.begin(), sorted.end());
  const auto p95_index = static_cast<std::size_t>(
    std::ceil(0.95 * static_cast<double>(sorted.size()))) - 1U;
  return {
    sum / static_cast<double>(values.size()),
    sorted[p95_index],
    maximum,
  };
}

double process_cpu_seconds()
{
  rusage usage{};
  if (getrusage(RUSAGE_SELF, &usage) != 0) {
    return 0.0;
  }
  const auto user =
    static_cast<double>(usage.ru_utime.tv_sec) +
    static_cast<double>(usage.ru_utime.tv_usec) / 1.0e6;
  const auto system =
    static_cast<double>(usage.ru_stime.tv_sec) +
    static_cast<double>(usage.ru_stime.tv_usec) / 1.0e6;
  return user + system;
}

double maximum_rss_mib()
{
  rusage usage{};
  if (getrusage(RUSAGE_SELF, &usage) != 0) {
    return 0.0;
  }
  return static_cast<double>(usage.ru_maxrss) / 1024.0;
}

double current_rss_mib()
{
  FILE * statm = std::fopen("/proc/self/statm", "r");
  if (statm == nullptr) {
    return 0.0;
  }
  unsigned long total_pages = 0;
  unsigned long resident_pages = 0;
  const auto scanned =
    std::fscanf(statm, "%lu %lu", &total_pages, &resident_pages);
  std::fclose(statm);
  (void)total_pages;
  if (scanned != 2) {
    return 0.0;
  }
  const auto page_size = sysconf(_SC_PAGESIZE);
  return static_cast<double>(resident_pages) *
         static_cast<double>(page_size) / (1024.0 * 1024.0);
}

double shared_memory_used_mib()
{
  struct statvfs stats {};
  if (statvfs("/dev/shm", &stats) != 0) {
    return 0.0;
  }
  const auto used_blocks = stats.f_blocks - stats.f_bfree;
  return static_cast<double>(used_blocks) *
         static_cast<double>(stats.f_frsize) / (1024.0 * 1024.0);
}

std::optional<int64_t> timestamp_to_nanoseconds(
  int32_t seconds, uint32_t nanoseconds)
{
  if (seconds < 0 || nanoseconds >= 1000000000U) {
    return std::nullopt;
  }
  constexpr auto billion = int64_t{1000000000};
  return static_cast<int64_t>(seconds) * billion +
         static_cast<int64_t>(nanoseconds);
}

}  // namespace

class CameraBenchmark : public rclcpp::Node
{
public:
  CameraBenchmark()
  : Node("camera_benchmark"),
    node_started_(SteadyClock::now())
  {
    topic_ = declare_parameter<std::string>(
      "topic", "/r2/left_camera/image_raw");
    message_type_ = declare_parameter<std::string>("message_type", "bounded");
    processing_mode_ = declare_parameter<std::string>(
      "processing_mode", "transport");
    warmup_seconds_ = declare_parameter<double>("warmup_sec", 3.0);
    duration_seconds_ = declare_parameter<double>("duration_sec", 20.0);

    if (topic_.empty()) {
      throw std::invalid_argument("topic must not be empty");
    }
    if (message_type_ != "standard" && message_type_ != "bounded") {
      throw std::invalid_argument(
              "message_type must be 'standard' or 'bounded'");
    }
    if (message_type_ == "standard" &&
      (topic_.size() < 6U ||
      topic_.compare(topic_.size() - 6U, 6U, "/debug") != 0))
    {
      topic_ += "/debug";
    }
    if (processing_mode_ != "transport" &&
      processing_mode_ != "opencv_mean")
    {
      throw std::invalid_argument(
              "processing_mode must be 'transport' or 'opencv_mean'");
    }
    if (warmup_seconds_ < 0.0 || duration_seconds_ <= 0.0) {
      throw std::invalid_argument(
              "warmup_sec must be non-negative and duration_sec positive");
    }

    measurement_begin_ =
      node_started_ + std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(warmup_seconds_));
    measurement_end_ =
      measurement_begin_ + std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(duration_seconds_));

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
      .best_effort()
      .durability_volatile();
    if (message_type_ == "bounded") {
      bounded_subscription_ = create_subscription<CameraFrame>(
        topic_, qos,
        std::bind(
          &CameraBenchmark::on_bounded_frame, this,
          std::placeholders::_1));
      subscription_loan_supported_ =
        bounded_subscription_->can_loan_messages();
      sequence_enabled_ = true;
    } else {
      standard_subscription_ = create_subscription<sensor_msgs::msg::Image>(
        topic_, qos,
        std::bind(
          &CameraBenchmark::on_standard_frame, this,
          std::placeholders::_1));
    }

    finish_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&CameraBenchmark::on_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "Warming up for %.1f s, then measuring %s/%s on %s for %.1f s "
      "(loan support: %s; loan is not required)",
      warmup_seconds_, message_type_.c_str(), processing_mode_.c_str(),
      topic_.c_str(), duration_seconds_,
      message_type_ == "bounded" ?
      (subscription_loan_supported_ ? "yes" : "no") : "not applicable");
  }

private:
  void on_bounded_frame(const CameraFrame & message)
  {
    const auto callback_started = SteadyClock::now();
    if (callback_started >= measurement_end_) {
      return;
    }
    if (callback_started >= measurement_begin_) {
      start_measurement_if_needed();
    }
    const auto arrival_ns = now().nanoseconds();

    bool metadata_valid = true;
    if (message.layout_version != CameraFrame::LAYOUT_VERSION) {
      ++invalid_layout_;
      metadata_valid = false;
    }
    if (message.frame_id_size > message.frame_id.size()) {
      ++invalid_frame_id_;
      metadata_valid = false;
    }

    const auto cv_type = bounded_cv_type(message.encoding);
    const auto channels = channels_for_cv_type(cv_type);
    if (message.data.size() != message.data_size) {
      ++invalid_data_size_;
      metadata_valid = false;
    }
    metadata_valid = validate_layout(
      message.width, message.height, message.step, message.data_size,
      message.data.size(), channels, cv_type, true) && metadata_valid;

    std::optional<int64_t> stamp_ns =
      timestamp_to_nanoseconds(message.stamp_sec, message.stamp_nanosec);
    if (!stamp_ns.has_value()) {
      ++invalid_timestamp_;
      metadata_valid = false;
    }

    process_frame(
      message.width, message.height, message.step, message.data_size,
      message.data.empty() ? nullptr : &message.data[0],
      cv_type, metadata_valid, stamp_ns, arrival_ns,
      message.sequence, callback_started);
  }

  void on_standard_frame(const sensor_msgs::msg::Image & message)
  {
    const auto callback_started = SteadyClock::now();
    if (callback_started >= measurement_end_) {
      return;
    }
    if (callback_started >= measurement_begin_) {
      start_measurement_if_needed();
    }
    const auto arrival_ns = now().nanoseconds();
    const auto cv_type = standard_cv_type(message.encoding);
    const auto channels = channels_for_cv_type(cv_type);
    const auto data_size = message.data.size();
    const auto data_size_u32 =
      data_size <= std::numeric_limits<uint32_t>::max() ?
      static_cast<uint32_t>(data_size) : std::numeric_limits<uint32_t>::max();
    bool metadata_valid = validate_layout(
      message.width, message.height, message.step, data_size_u32,
      data_size, channels, cv_type, false);

    std::optional<int64_t> stamp_ns = timestamp_to_nanoseconds(
      message.header.stamp.sec, message.header.stamp.nanosec);
    if (!stamp_ns.has_value()) {
      ++invalid_timestamp_;
      metadata_valid = false;
    }

    process_frame(
      message.width, message.height, message.step, data_size_u32,
      message.data.data(), cv_type, metadata_valid, stamp_ns, arrival_ns,
      std::nullopt, callback_started);
  }

  int bounded_cv_type(uint8_t encoding)
  {
    if (encoding == CameraFrame::ENCODING_BGR8 ||
      encoding == CameraFrame::ENCODING_RGB8)
    {
      return CV_8UC3;
    }
    if (encoding == CameraFrame::ENCODING_MONO8) {
      return CV_8UC1;
    }
    return -1;
  }

  static int standard_cv_type(const std::string & encoding)
  {
    if (encoding == "bgr8" || encoding == "rgb8") {
      return CV_8UC3;
    }
    if (encoding == "mono8") {
      return CV_8UC1;
    }
    return -1;
  }

  static uint32_t channels_for_cv_type(int cv_type)
  {
    if (cv_type == CV_8UC3) {
      return 3U;
    }
    if (cv_type == CV_8UC1) {
      return 1U;
    }
    return 0U;
  }

  bool validate_layout(
    uint32_t width, uint32_t height, uint32_t step, uint32_t data_size,
    std::size_t buffer_capacity, uint32_t channels, int cv_type,
    bool require_exact_size)
  {
    bool valid = true;
    if (width == 0U || height == 0U) {
      ++invalid_dimensions_;
      valid = false;
    }
    if (cv_type < 0 || channels == 0U) {
      ++invalid_encoding_;
      valid = false;
    }

    const auto minimum_step =
      static_cast<uint64_t>(width) * static_cast<uint64_t>(channels);
    if (step < minimum_step) {
      ++invalid_step_;
      valid = false;
    }

    const auto required_size =
      static_cast<uint64_t>(step) * static_cast<uint64_t>(height);
    const bool size_mismatch = require_exact_size ?
      static_cast<uint64_t>(data_size) != required_size :
      static_cast<uint64_t>(data_size) < required_size;
    if (size_mismatch || data_size > buffer_capacity) {
      ++invalid_data_size_;
      valid = false;
    }
    return valid;
  }

  void process_frame(
    uint32_t width, uint32_t height, uint32_t step, uint32_t data_size,
    const uint8_t * data, int cv_type, bool metadata_valid,
    const std::optional<int64_t> & stamp_ns, int64_t arrival_ns,
    const std::optional<uint64_t> & sequence,
    const SteadyClock::time_point & callback_started)
  {
    if (metadata_valid) {
      cv::Mat image(
        static_cast<int>(height), static_cast<int>(width), cv_type,
        const_cast<uint8_t *>(data), static_cast<std::size_t>(step));
      if (processing_mode_ == "opencv_mean") {
        const auto mean = cv::mean(image);
        mean_checksum_ += mean[0] + mean[1] + mean[2] + mean[3];
      } else {
        view_checksum_ +=
          static_cast<uint64_t>(image.rows) +
          static_cast<uint64_t>(image.cols) +
          static_cast<uint64_t>(image.step);
      }
    }

    const auto callback_finished = SteadyClock::now();
    if (callback_finished < measurement_begin_ ||
      callback_finished >= measurement_end_)
    {
      return;
    }
    start_measurement_if_needed();

    ++received_frames_;
    last_width_ = width;
    last_height_ = height;
    last_step_ = step;
    last_data_size_ = data_size;
    if (!metadata_valid) {
      ++frames_with_metadata_errors_;
    }

    if (stamp_ns.has_value() && arrival_ns >= *stamp_ns) {
      latency_ms_.push_back(
        static_cast<double>(arrival_ns - *stamp_ns) / 1.0e6);
    } else if (stamp_ns.has_value()) {
      ++invalid_timestamp_;
      if (metadata_valid) {
        ++frames_with_metadata_errors_;
      }
    }

    callback_ms_.push_back(
      std::chrono::duration<double, std::milli>(
        callback_finished - callback_started).count());

    if (sequence.has_value()) {
      if (last_sequence_.has_value()) {
        if (*sequence > *last_sequence_ + 1U) {
          dropped_frames_ += *sequence - *last_sequence_ - 1U;
        } else if (*sequence <= *last_sequence_) {
          ++out_of_order_frames_;
        }
      }
      last_sequence_ = sequence;
    }
  }

  void start_measurement_if_needed()
  {
    if (measurement_started_) {
      return;
    }
    measurement_started_ = true;
    frames_with_metadata_errors_ = 0;
    invalid_dimensions_ = 0;
    invalid_encoding_ = 0;
    invalid_step_ = 0;
    invalid_data_size_ = 0;
    invalid_timestamp_ = 0;
    invalid_layout_ = 0;
    invalid_frame_id_ = 0;
    cpu_seconds_at_start_ = process_cpu_seconds();
    rss_at_start_mib_ = current_rss_mib();
    max_sampled_rss_mib_ = rss_at_start_mib_;
    shm_at_start_mib_ = shared_memory_used_mib();
    max_sampled_shm_mib_ = shm_at_start_mib_;
  }

  void on_timer()
  {
    const auto current = SteadyClock::now();
    if (current >= measurement_begin_ && current < measurement_end_) {
      start_measurement_if_needed();
      max_sampled_rss_mib_ =
        std::max(max_sampled_rss_mib_, current_rss_mib());
      max_sampled_shm_mib_ =
        std::max(max_sampled_shm_mib_, shared_memory_used_mib());
      return;
    }
    if (current < measurement_end_ || result_printed_) {
      return;
    }

    result_printed_ = true;
    print_result();
    rclcpp::shutdown();
  }

  void print_result()
  {
    const auto latency = summarize(latency_ms_);
    const auto callback = summarize(callback_ms_);
    const auto expected_frames = received_frames_ + dropped_frames_;
    const auto drop_rate = expected_frames == 0U ? 0.0 :
      100.0 * static_cast<double>(dropped_frames_) /
      static_cast<double>(expected_frames);
    const auto fps =
      static_cast<double>(received_frames_) / duration_seconds_;
    const auto cpu_percent =
      100.0 * (process_cpu_seconds() - cpu_seconds_at_start_) /
      duration_seconds_;
    const auto shm_final = shared_memory_used_mib();

    RCLCPP_INFO(
      get_logger(),
      "RESULT type=%s mode=%s topic=%s duration_s=%.3f frames=%lu "
      "fps=%.3f resolution=%ux%u step=%u data_size=%u",
      message_type_.c_str(), processing_mode_.c_str(), topic_.c_str(),
      duration_seconds_, static_cast<unsigned long>(received_frames_), fps,
      last_width_, last_height_, last_step_, last_data_size_);
    if (sequence_enabled_) {
      RCLCPP_INFO(
        get_logger(),
        "RESULT sequence_drops=%lu drop_rate_pct=%.6f out_of_order=%lu",
        static_cast<unsigned long>(dropped_frames_), drop_rate,
        static_cast<unsigned long>(out_of_order_frames_));
    } else {
      RCLCPP_INFO(
        get_logger(),
        "RESULT sequence_drops=unavailable drop_rate_pct=unavailable "
        "(sensor_msgs/Image has no sequence field)");
    }
    RCLCPP_INFO(
      get_logger(),
      "RESULT latency_ms avg=%.3f p95=%.3f max=%.3f samples=%lu; "
      "callback_ms avg=%.3f p95=%.3f max=%.3f",
      latency.average, latency.p95, latency.maximum,
      static_cast<unsigned long>(latency_ms_.size()),
      callback.average, callback.p95, callback.maximum);
    RCLCPP_INFO(
      get_logger(),
      "RESULT metadata_error_frames=%lu invalid_dimensions=%lu "
      "invalid_encoding=%lu invalid_step=%lu invalid_data_size=%lu "
      "invalid_timestamp=%lu invalid_layout=%lu invalid_frame_id=%lu",
      static_cast<unsigned long>(frames_with_metadata_errors_),
      static_cast<unsigned long>(invalid_dimensions_),
      static_cast<unsigned long>(invalid_encoding_),
      static_cast<unsigned long>(invalid_step_),
      static_cast<unsigned long>(invalid_data_size_),
      static_cast<unsigned long>(invalid_timestamp_),
      static_cast<unsigned long>(invalid_layout_),
      static_cast<unsigned long>(invalid_frame_id_));
    RCLCPP_INFO(
      get_logger(),
      "RESULT subscriber_cpu_pct=%.3f rss_start_mib=%.3f "
      "rss_max_sampled_mib=%.3f rss_max_process_mib=%.3f "
      "shm_start_mib=%.3f shm_max_sampled_mib=%.3f shm_final_mib=%.3f "
      "checksum=%.3f/%lu",
      cpu_percent, rss_at_start_mib_, max_sampled_rss_mib_,
      maximum_rss_mib(), shm_at_start_mib_, max_sampled_shm_mib_, shm_final,
      mean_checksum_, static_cast<unsigned long>(view_checksum_));
  }

  std::string topic_;
  std::string message_type_;
  std::string processing_mode_;
  double warmup_seconds_{0.0};
  double duration_seconds_{0.0};

  SteadyClock::time_point node_started_;
  SteadyClock::time_point measurement_begin_;
  SteadyClock::time_point measurement_end_;
  bool measurement_started_{false};
  bool result_printed_{false};
  bool sequence_enabled_{false};
  bool subscription_loan_supported_{false};

  rclcpp::Subscription<CameraFrame>::SharedPtr bounded_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
  standard_subscription_;
  rclcpp::TimerBase::SharedPtr finish_timer_;

  uint64_t received_frames_{0};
  uint64_t dropped_frames_{0};
  uint64_t out_of_order_frames_{0};
  std::optional<uint64_t> last_sequence_;
  uint32_t last_width_{0};
  uint32_t last_height_{0};
  uint32_t last_step_{0};
  uint32_t last_data_size_{0};

  uint64_t frames_with_metadata_errors_{0};
  uint64_t invalid_dimensions_{0};
  uint64_t invalid_encoding_{0};
  uint64_t invalid_step_{0};
  uint64_t invalid_data_size_{0};
  uint64_t invalid_timestamp_{0};
  uint64_t invalid_layout_{0};
  uint64_t invalid_frame_id_{0};

  std::vector<double> latency_ms_;
  std::vector<double> callback_ms_;
  double mean_checksum_{0.0};
  uint64_t view_checksum_{0};

  double cpu_seconds_at_start_{0.0};
  double rss_at_start_mib_{0.0};
  double max_sampled_rss_mib_{0.0};
  double shm_at_start_mib_{0.0};
  double max_sampled_shm_mib_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<CameraBenchmark>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("camera_benchmark"),
      "Camera benchmark failed: %s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
