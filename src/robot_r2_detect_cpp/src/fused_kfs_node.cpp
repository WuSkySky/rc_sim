#include "robot_r2_detect_cpp/resnet_preprocess.hpp"

#include <NvInfer.h>
#include <cuda_fp16.h>
#include <cuda_runtime_api.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_r2_interfaces/msg/camera_frame.hpp>
#include <robot_r2_interfaces/msg/kfs_fused_debug_images.hpp>
#include <robot_r2_interfaces/msg/kfs_fused_processed_detections.hpp>
#include <robot_r2_interfaces/msg/kfs_fused_raw_detections.hpp>
#include <robot_r2_interfaces/msg/kfs_processed_detection.hpp>
#include <robot_r2_interfaces/msg/kfs_raw_box.hpp>
#include <robot_r2_interfaces/msg/kfs_raw_detections.hpp>
#include <robot_r2_interfaces/msg/kfs_type_result.hpp>
#include <robot_r2_interfaces/srv/get_kfs_type.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <numeric>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

namespace robot_r2_detect_cpp {
namespace {

using CameraFrame = robot_r2_interfaces::msg::CameraFrame;
using GetKfsType = robot_r2_interfaces::srv::GetKfsType;

constexpr char kRawTopic[] = "/r2/detection/raw";
constexpr char kProcessedTopic[] = "/r2/detection/processed";
constexpr char kDebugTopic[] = "/r2/detection/debug";
constexpr char kServiceName[] = "/r2/detection/get_type";

void check_cuda(cudaError_t status, const char *operation) {
  if (status != cudaSuccess) {
    throw std::runtime_error(std::string(operation) + ": " +
                             cudaGetErrorString(status));
  }
}

std::size_t data_type_size(nvinfer1::DataType type) {
  switch (type) {
  case nvinfer1::DataType::kFLOAT:
    return sizeof(float);
  case nvinfer1::DataType::kHALF:
    return sizeof(__half);
  default:
    throw std::runtime_error(
        "Only float32 and float16 TensorRT tensors are supported");
  }
}

std::size_t tensor_volume(const nvinfer1::Dims &dimensions) {
  if (dimensions.nbDims <= 0) {
    throw std::runtime_error("TensorRT returned an empty tensor shape");
  }
  std::size_t volume = 1;
  for (int index = 0; index < dimensions.nbDims; ++index) {
    if (dimensions.d[index] <= 0) {
      throw std::runtime_error("TensorRT returned an unresolved tensor shape");
    }
    volume *= static_cast<std::size_t>(dimensions.d[index]);
  }
  return volume;
}

class TensorRtLogger final : public nvinfer1::ILogger {
public:
  void log(Severity severity, const char *message) noexcept override {
    if (severity <= Severity::kWARNING) {
      std::fprintf(stderr, "[TensorRT] %s\n", message);
    }
  }
};

template <typename T> struct TensorRtDeleter {
  void operator()(T *object) const noexcept { delete object; }
};

struct FrameView {
  const std::uint8_t *data{};
  int width{};
  int height{};
  std::size_t step{};
  int channels{};
  bool is_rgb{};
};

void validate_horizontal_crop_ratio(double ratio) {
  if (!std::isfinite(ratio) || ratio < 0.0 || ratio >= 0.5) {
    throw std::invalid_argument(
        "horizontal_crop_ratio must be finite and in [0.0, 0.5)");
  }
}

FrameView crop_frame_horizontally(FrameView view, double ratio) {
  validate_horizontal_crop_ratio(ratio);
  const int crop_each_side =
      static_cast<int>(std::floor(static_cast<double>(view.width) * ratio));
  const int cropped_width = view.width - 2 * crop_each_side;
  if (cropped_width <= 0) {
    throw std::invalid_argument("horizontal crop leaves an empty image");
  }
  view.data += static_cast<std::size_t>(crop_each_side) * view.channels;
  view.width = cropped_width;
  return view;
}

struct Classification {
  int class_id{};
  float confidence{};
};

class TensorRtResNet {
public:
  TensorRtResNet(const std::string &engine_path, int image_size,
                 int class_count, const std::array<float, 3> &mean,
                 const std::array<float, 3> &std)
      : image_size_(image_size), class_count_(class_count), mean_(mean),
        std_(std) {
    std::ifstream stream(engine_path, std::ios::binary | std::ios::ate);
    if (!stream) {
      throw std::runtime_error("Unable to open TensorRT engine: " +
                               engine_path);
    }
    const auto file_size = stream.tellg();
    if (file_size <= 0) {
      throw std::runtime_error("TensorRT engine is empty: " + engine_path);
    }
    std::vector<char> bytes(static_cast<std::size_t>(file_size));
    stream.seekg(0, std::ios::beg);
    if (!stream.read(bytes.data(), file_size)) {
      throw std::runtime_error("Unable to read TensorRT engine: " +
                               engine_path);
    }

    runtime_.reset(nvinfer1::createInferRuntime(logger_));
    if (!runtime_) {
      throw std::runtime_error("TensorRT runtime creation failed");
    }
    engine_.reset(runtime_->deserializeCudaEngine(bytes.data(), bytes.size()));
    if (!engine_) {
      throw std::runtime_error(
          "TensorRT engine deserialization failed; rebuild it on this Jetson");
    }
    context_.reset(engine_->createExecutionContext());
    if (!context_) {
      throw std::runtime_error("TensorRT execution context creation failed");
    }

    for (int index = 0; index < engine_->getNbIOTensors(); ++index) {
      const char *name = engine_->getIOTensorName(index);
      const auto mode = engine_->getTensorIOMode(name);
      if (mode == nvinfer1::TensorIOMode::kINPUT) {
        if (!input_name_.empty()) {
          throw std::runtime_error("Expected exactly one TensorRT input");
        }
        input_name_ = name;
      } else if (mode == nvinfer1::TensorIOMode::kOUTPUT) {
        if (!output_name_.empty()) {
          throw std::runtime_error("Expected exactly one TensorRT output");
        }
        output_name_ = name;
      }
    }
    if (input_name_.empty() || output_name_.empty()) {
      throw std::runtime_error(
          "TensorRT engine must have one input and one output");
    }

    input_dimensions_ = engine_->getTensorShape(input_name_.c_str());
    if (input_dimensions_.nbDims != 4 || input_dimensions_.d[1] != 3 ||
        input_dimensions_.d[2] != image_size_ ||
        input_dimensions_.d[3] != image_size_) {
      throw std::runtime_error(
          "TensorRT input does not match NCHW model_input_size");
    }
    fixed_batch_ = input_dimensions_.d[0] > 0 ? input_dimensions_.d[0] : 0;
    input_type_ = engine_->getTensorDataType(input_name_.c_str());
    output_type_ = engine_->getTensorDataType(output_name_.c_str());
    input_element_size_ = data_type_size(input_type_);
    output_element_size_ = data_type_size(output_type_);
    preprocess_type_ = input_type_ == nvinfer1::DataType::kFLOAT
                           ? TensorElementType::kFloat32
                           : TensorElementType::kFloat16;
    check_cuda(cudaStreamCreate(&cuda_stream_), "cudaStreamCreate");
  }

  ~TensorRtResNet() {
    for (void *buffer : source_buffers_) {
      if (buffer != nullptr) {
        cudaFree(buffer);
      }
    }
    if (device_input_ != nullptr) {
      cudaFree(device_input_);
    }
    if (device_output_ != nullptr) {
      cudaFree(device_output_);
    }
    if (cuda_stream_ != nullptr) {
      cudaStreamDestroy(cuda_stream_);
    }
  }

  TensorRtResNet(const TensorRtResNet &) = delete;
  TensorRtResNet &operator=(const TensorRtResNet &) = delete;

  int preferred_chunk_size() const {
    return fixed_batch_ > 0 ? fixed_batch_ : 3;
  }

  std::vector<Classification> infer(const std::vector<FrameView> &frames) {
    if (frames.empty()) {
      return {};
    }
    const int requested_batch = static_cast<int>(frames.size());
    const int execution_batch =
        fixed_batch_ > 0 ? fixed_batch_ : requested_batch;
    if (requested_batch > execution_batch) {
      throw std::runtime_error(
          "Input batch exceeds the fixed TensorRT batch size");
    }

    auto dimensions = input_dimensions_;
    dimensions.d[0] = execution_batch;
    if (!context_->setInputShape(input_name_.c_str(), dimensions)) {
      throw std::runtime_error(
          "TensorRT optimization profile rejected batch size " +
          std::to_string(execution_batch));
    }

    const auto output_dimensions =
        context_->getTensorShape(output_name_.c_str());
    const std::size_t output_elements = tensor_volume(output_dimensions);
    const std::size_t expected_output =
        static_cast<std::size_t>(execution_batch) * class_count_;
    if (output_elements != expected_output) {
      throw std::runtime_error(
          "TensorRT output size does not match batch * configured classes");
    }

    const std::size_t slot_elements =
        static_cast<std::size_t>(3) * image_size_ * image_size_;
    const std::size_t slot_bytes = slot_elements * input_element_size_;
    ensure_device_buffer(device_input_, input_capacity_,
                         slot_bytes * execution_batch, "TensorRT input");
    ensure_device_buffer(device_output_, output_capacity_,
                         output_elements * output_element_size_,
                         "TensorRT output");
    if (source_buffers_.size() < frames.size()) {
      source_buffers_.resize(frames.size(), nullptr);
      source_capacities_.resize(frames.size(), 0);
    }

    for (std::size_t slot = 0; slot < frames.size(); ++slot) {
      const FrameView &frame = frames[slot];
      const std::size_t packed_step =
          static_cast<std::size_t>(frame.width) * frame.channels;
      const std::size_t source_bytes = packed_step * frame.height;
      ensure_device_buffer(source_buffers_[slot], source_capacities_[slot],
                           source_bytes, "camera staging image");
      check_cuda(cudaMemcpy2DAsync(source_buffers_[slot], packed_step,
                                   frame.data, frame.step, packed_step,
                                   frame.height, cudaMemcpyHostToDevice,
                                   cuda_stream_),
                 "cudaMemcpy2DAsync camera image");
      auto *destination =
          static_cast<std::uint8_t *>(device_input_) + slot * slot_bytes;
      launch_resnet_preprocess(
          static_cast<const std::uint8_t *>(source_buffers_[slot]), frame.width,
          frame.height, packed_step, frame.channels, frame.is_rgb, destination,
          preprocess_type_, image_size_, mean_.data(), std_.data(),
          cuda_stream_);
    }

    // Fixed batch engines still work when only one or two cameras are alive:
    // duplicate tensor slot zero for padding and discard those outputs.
    for (int slot = requested_batch; slot < execution_batch; ++slot) {
      auto *destination = static_cast<std::uint8_t *>(device_input_) +
                          static_cast<std::size_t>(slot) * slot_bytes;
      check_cuda(cudaMemcpyAsync(destination, device_input_, slot_bytes,
                                 cudaMemcpyDeviceToDevice, cuda_stream_),
                 "cudaMemcpyAsync batch padding");
    }

    if (!context_->setTensorAddress(input_name_.c_str(), device_input_) ||
        !context_->setTensorAddress(output_name_.c_str(), device_output_)) {
      throw std::runtime_error("TensorRT tensor address setup failed");
    }
    if (!context_->enqueueV3(cuda_stream_)) {
      throw std::runtime_error("TensorRT enqueueV3 failed");
    }

    std::vector<std::uint8_t> host_output(output_elements *
                                          output_element_size_);
    check_cuda(cudaMemcpyAsync(host_output.data(), device_output_,
                               host_output.size(), cudaMemcpyDeviceToHost,
                               cuda_stream_),
               "cudaMemcpyAsync TensorRT output");
    check_cuda(cudaStreamSynchronize(cuda_stream_), "cudaStreamSynchronize");

    std::vector<Classification> classifications;
    classifications.reserve(frames.size());
    for (int batch = 0; batch < requested_batch; ++batch) {
      std::vector<float> logits(class_count_);
      for (int class_id = 0; class_id < class_count_; ++class_id) {
        const std::size_t index =
            static_cast<std::size_t>(batch) * class_count_ + class_id;
        if (output_type_ == nvinfer1::DataType::kFLOAT) {
          logits[class_id] =
              reinterpret_cast<const float *>(host_output.data())[index];
        } else {
          logits[class_id] = __half2float(
              reinterpret_cast<const __half *>(host_output.data())[index]);
        }
      }
      const float maximum = *std::max_element(logits.begin(), logits.end());
      float denominator = 0.0F;
      for (float &value : logits) {
        value = std::exp(value - maximum);
        denominator += value;
      }
      const auto best = std::max_element(logits.begin(), logits.end());
      classifications.push_back(
          Classification{static_cast<int>(std::distance(logits.begin(), best)),
                         denominator > 0.0F ? *best / denominator : 0.0F});
    }
    return classifications;
  }

private:
  static void ensure_device_buffer(void *&buffer, std::size_t &capacity,
                                   std::size_t required,
                                   const char *description) {
    if (required <= capacity) {
      return;
    }
    if (buffer != nullptr) {
      check_cuda(cudaFree(buffer), "cudaFree");
      buffer = nullptr;
      capacity = 0;
    }
    check_cuda(cudaMalloc(&buffer, required), description);
    capacity = required;
  }

  TensorRtLogger logger_;
  std::unique_ptr<nvinfer1::IRuntime, TensorRtDeleter<nvinfer1::IRuntime>>
      runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine, TensorRtDeleter<nvinfer1::ICudaEngine>>
      engine_;
  std::unique_ptr<nvinfer1::IExecutionContext,
                  TensorRtDeleter<nvinfer1::IExecutionContext>>
      context_;
  std::string input_name_;
  std::string output_name_;
  nvinfer1::Dims input_dimensions_{};
  nvinfer1::DataType input_type_{};
  nvinfer1::DataType output_type_{};
  TensorElementType preprocess_type_{};
  int image_size_{};
  int class_count_{};
  int fixed_batch_{};
  std::array<float, 3> mean_{};
  std::array<float, 3> std_{};
  std::size_t input_element_size_{};
  std::size_t output_element_size_{};
  cudaStream_t cuda_stream_{};
  void *device_input_{};
  void *device_output_{};
  std::size_t input_capacity_{};
  std::size_t output_capacity_{};
  std::vector<void *> source_buffers_;
  std::vector<std::size_t> source_capacities_;
};

struct CameraState {
  std::string name;
  std::string image_topic;
  CameraFrame::ConstSharedPtr latest_frame;
  std::chrono::steady_clock::time_point received_at{};
  std::uint64_t generation{};
  std::uint64_t processed_generation{};
  std::mutex vote_mutex;
  std::condition_variable vote_condition;
  bool vote_active{};
  std::size_t vote_target{};
  std::vector<std::string> vote_samples;
};

struct PendingFrame {
  std::size_t camera_index{};
  CameraFrame::ConstSharedPtr message;
};

struct KfsConfiguration {
  std::string model_path;
  int image_size{};
  std::vector<std::string> class_names;
  std::array<float, 3> mean{};
  std::array<float, 3> std{};
  double horizontal_crop_ratio{};
  double confidence_threshold{};
  bool visualization_enabled{};
  double inference_rate{};
  double stale_timeout_sec{};
  double default_vote_timeout_sec{};
};

class FusedKfsNode final : public rclcpp::Node {
public:
  FusedKfsNode() : Node("kfs_detect_fused") {
    declare_parameter<std::string>(
        "model_path", "",
        descriptor("TensorRT engine path; empty selects the installed default"));
    declare_parameter<int>(
        "model_input_size", 224,
        integer_descriptor("Square TensorRT model input size in pixels", 1,
                           16384));
    declare_parameter<std::vector<std::string>>(
        "model_class_names", {"R1", "Unlabeled", "fake", "true"},
        descriptor("Class names in TensorRT output-logit order"));
    declare_parameter<std::vector<double>>(
        "model_mean", {0.485, 0.456, 0.406},
        descriptor("Three RGB normalization mean values"));
    declare_parameter<std::vector<double>>(
        "model_std", {0.229, 0.224, 0.225},
        descriptor("Three positive RGB normalization standard deviations"));
    declare_parameter<double>(
        "horizontal_crop_ratio", 0.2,
        floating_descriptor(
            "Fraction cropped from both horizontal sides before inference",
            0.0, 0.499999999));
    declare_parameter<double>(
        "conf", 0.5,
        floating_descriptor("Minimum confidence for processed detections", 0.0,
                            1.0));
    declare_parameter<bool>(
        "visualization_enabled", false,
        descriptor("Publish annotated debug images when subscribers exist"));
    declare_parameter<double>(
        "inference_rate", 30.0,
        floating_descriptor("Maximum fused inference batch rate in Hz", 0.001,
                            1000.0));
    declare_parameter<double>(
        "frame_stale_timeout_sec", 0.5,
        floating_descriptor("Maximum accepted cached-frame age in seconds",
                            0.001, 1000000.0));
    declare_parameter<double>(
        "default_vote_timeout_sec", 10.0,
        floating_descriptor(
            "Default detection-vote timeout when a request uses zero", 0.001,
            1000000.0));

    auto configuration = std::make_shared<KfsConfiguration>();
    configuration->model_path = get_parameter("model_path").as_string();
    configuration->image_size = get_parameter("model_input_size").as_int();
    configuration->class_names =
        get_parameter("model_class_names").as_string_array();
    const auto mean = get_parameter("model_mean").as_double_array();
    const auto std = get_parameter("model_std").as_double_array();
    if (mean.size() == 3 && std.size() == 3) {
      for (std::size_t index = 0; index < 3; ++index) {
        configuration->mean[index] = static_cast<float>(mean[index]);
        configuration->std[index] = static_cast<float>(std[index]);
      }
    }
    configuration->horizontal_crop_ratio =
        get_parameter("horizontal_crop_ratio").as_double();
    configuration->confidence_threshold = get_parameter("conf").as_double();
    configuration->visualization_enabled =
        get_parameter("visualization_enabled").as_bool();
    configuration->inference_rate =
        get_parameter("inference_rate").as_double();
    configuration->stale_timeout_sec =
        get_parameter("frame_stale_timeout_sec").as_double();
    configuration->default_vote_timeout_sec =
        get_parameter("default_vote_timeout_sec").as_double();
    validate_configuration(*configuration, mean.size(), std.size());

    const std::string model_path = resolve_model_path(configuration->model_path);
    engine_ = std::make_unique<TensorRtResNet>(
        model_path, configuration->image_size,
        static_cast<int>(configuration->class_names.size()),
        configuration->mean, configuration->std);
    configuration_ = configuration;
    parameter_callback_handle_ = add_on_set_parameters_callback(std::bind(
        &FusedKfsNode::on_parameters_changed, this, std::placeholders::_1));

    const std::array<std::string, 3> names{"front", "left", "right"};
    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
    image_callback_group_ =
        create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    inference_callback_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_callback_group_ =
        create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    raw_publisher_ =
        create_publisher<robot_r2_interfaces::msg::KfsFusedRawDetections>(
            kRawTopic, 10);
    processed_publisher_ =
        create_publisher<
            robot_r2_interfaces::msg::KfsFusedProcessedDetections>(
            kProcessedTopic, 10);
    debug_publisher_ =
        create_publisher<robot_r2_interfaces::msg::KfsFusedDebugImages>(
            kDebugTopic, sensor_qos);

    for (std::size_t index = 0; index < names.size(); ++index) {
      auto state = std::make_unique<CameraState>();
      state->name = names[index];
      state->image_topic = "/r2/" + names[index] + "_camera/image_raw";
      cameras_.push_back(std::move(state));

      rclcpp::SubscriptionOptions options;
      options.callback_group = image_callback_group_;
      subscriptions_.push_back(create_subscription<CameraFrame>(
          cameras_[index]->image_topic, sensor_qos,
          [this, index](CameraFrame::ConstSharedPtr message) {
            on_image(index, std::move(message));
          },
          options));
    }

    service_ = create_service<GetKfsType>(
        kServiceName,
        [this](const std::shared_ptr<GetKfsType::Request> request,
               std::shared_ptr<GetKfsType::Response> response) {
          handle_vote_service(request, response);
        },
        rmw_qos_profile_services_default, service_callback_group_);

    replace_inference_timer(configuration->inference_rate);
    RCLCPP_INFO(get_logger(),
                "Loaded fused TensorRT classifier '%s' at %.1f Hz with %.1f%% "
                "cropped from each horizontal side; missing cameras are "
                "optional",
                model_path.c_str(), configuration->inference_rate,
                configuration->horizontal_crop_ratio * 100.0);
  }

private:
  static rcl_interfaces::msg::ParameterDescriptor
  descriptor(const std::string &description) {
    rcl_interfaces::msg::ParameterDescriptor result;
    result.description = description;
    return result;
  }

  static rcl_interfaces::msg::ParameterDescriptor
  floating_descriptor(const std::string &description, double minimum,
                      double maximum) {
    auto result = descriptor(description);
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = minimum;
    range.to_value = maximum;
    range.step = 0.0;
    result.floating_point_range.push_back(range);
    return result;
  }

  static rcl_interfaces::msg::ParameterDescriptor
  integer_descriptor(const std::string &description, std::int64_t minimum,
                     std::int64_t maximum) {
    auto result = descriptor(description);
    rcl_interfaces::msg::IntegerRange range;
    range.from_value = minimum;
    range.to_value = maximum;
    range.step = 1;
    result.integer_range.push_back(range);
    return result;
  }

  static std::string resolve_model_path(const std::string &configured_path) {
    if (!configured_path.empty()) {
      return configured_path;
    }
    return ament_index_cpp::get_package_share_directory("robot_r2_detect") +
           "/model/resnet18_batch3_fp16.engine";
  }

  rcl_interfaces::msg::SetParametersResult on_parameters_changed(
      const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    const auto current = std::atomic_load(&configuration_);
    auto next = std::make_shared<KfsConfiguration>(*current);
    bool reload_model = false;
    bool replace_timer = false;
    bool relevant_parameter = false;
    try {
      for (const auto &parameter : parameters) {
        const std::string &name = parameter.get_name();
        if (name == "model_path") {
          relevant_parameter = true;
          next->model_path = parameter.as_string();
          reload_model = true;
        } else if (name == "model_input_size") {
          relevant_parameter = true;
          next->image_size = parameter.as_int();
          reload_model = true;
        } else if (name == "model_class_names") {
          relevant_parameter = true;
          next->class_names = parameter.as_string_array();
          reload_model = true;
        } else if (name == "model_mean" || name == "model_std") {
          relevant_parameter = true;
          const auto values = parameter.as_double_array();
          if (values.size() != 3) {
            throw std::invalid_argument(name + " must contain 3 values");
          }
          auto &destination = name == "model_mean" ? next->mean : next->std;
          for (std::size_t index = 0; index < 3; ++index) {
            destination[index] = static_cast<float>(values[index]);
          }
          reload_model = true;
        } else if (name == "horizontal_crop_ratio") {
          relevant_parameter = true;
          next->horizontal_crop_ratio = parameter.as_double();
        } else if (name == "conf") {
          relevant_parameter = true;
          next->confidence_threshold = parameter.as_double();
        } else if (name == "visualization_enabled") {
          relevant_parameter = true;
          next->visualization_enabled = parameter.as_bool();
        } else if (name == "inference_rate") {
          relevant_parameter = true;
          next->inference_rate = parameter.as_double();
          replace_timer = next->inference_rate != current->inference_rate;
        } else if (name == "frame_stale_timeout_sec") {
          relevant_parameter = true;
          next->stale_timeout_sec = parameter.as_double();
        } else if (name == "default_vote_timeout_sec") {
          relevant_parameter = true;
          next->default_vote_timeout_sec = parameter.as_double();
        }
      }
      if (!relevant_parameter) {
        return result;
      }
      validate_configuration(*next, 3, 3);
    } catch (const std::exception &error) {
      result.successful = false;
      result.reason = error.what();
      return result;
    }

    std::unique_ptr<TensorRtResNet> replacement_engine;
    std::string resolved_model_path;
    if (reload_model) {
      try {
        resolved_model_path = resolve_model_path(next->model_path);
        replacement_engine = std::make_unique<TensorRtResNet>(
            resolved_model_path, next->image_size,
            static_cast<int>(next->class_names.size()), next->mean, next->std);
      } catch (const std::exception &error) {
        result.successful = false;
        result.reason = std::string("TensorRT model reload failed: ") +
                        error.what();
        return result;
      }
    }

    std::unique_ptr<TensorRtResNet> previous_engine;
    {
      std::lock_guard<std::mutex> lock(engine_mutex_);
      if (replacement_engine) {
        previous_engine = std::move(engine_);
        engine_ = std::move(replacement_engine);
      }
      std::shared_ptr<const KfsConfiguration> immutable_next = next;
      std::atomic_store(&configuration_, std::move(immutable_next));
    }

    if (replace_timer) {
      try {
        replace_inference_timer(next->inference_rate);
      } catch (const std::exception &error) {
        std::unique_ptr<TensorRtResNet> failed_engine;
        {
          std::lock_guard<std::mutex> lock(engine_mutex_);
          if (previous_engine) {
            failed_engine = std::move(engine_);
            engine_ = std::move(previous_engine);
          }
          std::atomic_store(&configuration_, current);
        }
        result.successful = false;
        result.reason =
            std::string("inference timer update failed: ") + error.what();
        return result;
      }
    }
    previous_engine.reset();
    if (reload_model) {
      RCLCPP_INFO(get_logger(), "Reloaded TensorRT classifier '%s'",
                  resolved_model_path.c_str());
    }
    RCLCPP_INFO(get_logger(),
                "Updated KFS parameters: conf=%.3f, rate=%.3f Hz, crop=%.1f%% "
                "per side, visualization=%s",
                next->confidence_threshold, next->inference_rate,
                next->horizontal_crop_ratio * 100.0,
                next->visualization_enabled ? "enabled" : "disabled");
    return result;
  }

  static void validate_configuration(const KfsConfiguration &configuration,
                                     std::size_t mean_size,
                                     std::size_t std_size) {
    if (configuration.image_size <= 0 || configuration.image_size > 16384 ||
        configuration.class_names.empty()) {
      throw std::invalid_argument(
          "model_input_size and model_class_names are required");
    }
    if (std::any_of(configuration.class_names.begin(),
                    configuration.class_names.end(),
                    [](const std::string &name) { return name.empty(); })) {
      throw std::invalid_argument("model_class_names must not contain blanks");
    }
    for (std::size_t left = 0; left < configuration.class_names.size();
         ++left) {
      for (std::size_t right = left + 1;
           right < configuration.class_names.size(); ++right) {
        if (configuration.class_names[left] ==
            configuration.class_names[right]) {
          throw std::invalid_argument("model_class_names must be unique");
        }
      }
    }
    if (mean_size != 3 || std_size != 3) {
      throw std::invalid_argument(
          "model_mean and model_std must contain 3 values");
    }
    validate_horizontal_crop_ratio(configuration.horizontal_crop_ratio);
    if (!std::isfinite(configuration.confidence_threshold) ||
        configuration.confidence_threshold < 0.0 ||
        configuration.confidence_threshold > 1.0) {
      throw std::invalid_argument("conf must be finite and in [0, 1]");
    }
    if (!std::isfinite(configuration.inference_rate) ||
        configuration.inference_rate < 0.001 ||
        configuration.inference_rate > 1000.0) {
      throw std::invalid_argument("inference_rate must be finite and positive");
    }
    if (!std::isfinite(configuration.stale_timeout_sec) ||
        configuration.stale_timeout_sec < 0.001 ||
        configuration.stale_timeout_sec > 1000000.0) {
      throw std::invalid_argument(
          "frame_stale_timeout_sec must be finite and positive");
    }
    if (!std::isfinite(configuration.default_vote_timeout_sec) ||
        configuration.default_vote_timeout_sec < 0.001 ||
        configuration.default_vote_timeout_sec > 1000000.0) {
      throw std::invalid_argument(
          "default_vote_timeout_sec must be finite and positive");
    }
    for (std::size_t index = 0; index < 3; ++index) {
      if (!std::isfinite(configuration.mean[index]) ||
          !std::isfinite(configuration.std[index]) ||
          configuration.std[index] <= 0.0F) {
        throw std::invalid_argument(
            "model_mean/std must be finite and std positive");
      }
    }
  }

  void replace_inference_timer(double rate) {
    const auto period = std::chrono::duration<double>(1.0 / rate);
    auto replacement = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&FusedKfsNode::run_inference, this),
        inference_callback_group_);
    if (inference_timer_) {
      inference_timer_->cancel();
    }
    inference_timer_ = std::move(replacement);
  }

  void on_image(std::size_t index, CameraFrame::ConstSharedPtr message) {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    CameraState &state = *cameras_.at(index);
    state.latest_frame = std::move(message);
    state.received_at = std::chrono::steady_clock::now();
    ++state.generation;
  }

  static FrameView validate_frame(const CameraFrame &message) {
    if (message.layout_version != CameraFrame::LAYOUT_VERSION) {
      throw std::invalid_argument("unsupported CameraFrame layout_version");
    }
    if (message.width == 0 || message.height == 0) {
      throw std::invalid_argument("CameraFrame width/height must be positive");
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
    const std::size_t expected =
        static_cast<std::size_t>(message.height) * message.step;
    if (message.step < row_bytes || message.data_size != expected ||
        message.data.size() != expected) {
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
                     static_cast<int>(message.height),
                     message.step,
                     channels,
                     is_rgb};
  }

  void run_inference() {
    const auto started = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> engine_lock(engine_mutex_);
    const auto configuration = std::atomic_load(&configuration_);
    std::vector<PendingFrame> pending;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      const auto now = std::chrono::steady_clock::now();
      for (std::size_t index = 0; index < cameras_.size(); ++index) {
        CameraState &state = *cameras_[index];
        if (!state.latest_frame ||
            state.generation == state.processed_generation) {
          continue;
        }
        state.processed_generation = state.generation;
        const double age =
            std::chrono::duration<double>(now - state.received_at).count();
        if (age <= configuration->stale_timeout_sec) {
          pending.push_back(PendingFrame{index, state.latest_frame});
        }
      }
    }
    if (pending.empty()) {
      return;
    }

    std::vector<PendingFrame> valid;
    std::vector<FrameView> views;
    for (const auto &item : pending) {
      try {
        views.push_back(crop_frame_horizontally(
            validate_frame(*item.message),
            configuration->horizontal_crop_ratio));
        valid.push_back(item);
      } catch (const std::exception &error) {
        RCLCPP_ERROR(get_logger(), "Ignored invalid %s camera frame: %s",
                     cameras_[item.camera_index]->name.c_str(), error.what());
      }
    }
    if (valid.empty()) {
      return;
    }

    robot_r2_interfaces::msg::KfsFusedRawDetections raw_batch;
    robot_r2_interfaces::msg::KfsFusedProcessedDetections processed_batch;
    const bool publish_debug =
        configuration->visualization_enabled &&
        debug_publisher_->get_subscription_count() > 0;
    robot_r2_interfaces::msg::KfsFusedDebugImages debug_batch;

    try {
      const int chunk_size = std::max(1, engine_->preferred_chunk_size());
      for (std::size_t begin = 0; begin < valid.size(); begin += chunk_size) {
        const std::size_t end = std::min(
            valid.size(), begin + static_cast<std::size_t>(chunk_size));
        std::vector<FrameView> chunk_views(views.begin() + begin,
                                           views.begin() + end);
        const auto results = engine_->infer(chunk_views);
        for (std::size_t offset = 0; offset < results.size(); ++offset) {
          const PendingFrame &item = valid[begin + offset];
          fill_result(item.camera_index, *item.message, views[begin + offset],
                      results[offset], *configuration, raw_batch,
                      processed_batch);
          if (publish_debug) {
            fill_debug(item.camera_index, *item.message, views[begin + offset],
                       results[offset], *configuration, debug_batch);
          }
        }
      }
    } catch (const std::exception &error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "Fused TensorRT inference failed: %s",
                            error.what());
    }

    raw_publisher_->publish(raw_batch);
    processed_publisher_->publish(processed_batch);
    if (publish_debug) {
      debug_publisher_->publish(debug_batch);
    }

    const double elapsed = std::chrono::duration<double>(
                               std::chrono::steady_clock::now() - started)
                               .count();
    if (elapsed > 1.0 / configuration->inference_rate) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Fused KFS batch took %.2f ms, above %.2f ms target",
                           elapsed * 1000.0,
                           1000.0 / configuration->inference_rate);
    }
  }

  static std_msgs::msg::Header make_header(const CameraFrame &frame) {
    std_msgs::msg::Header header;
    header.stamp.sec = frame.stamp_sec;
    header.stamp.nanosec = frame.stamp_nanosec;
    header.frame_id.assign(
        reinterpret_cast<const char *>(frame.frame_id.data()),
        frame.frame_id_size);
    return header;
  }

  static robot_r2_interfaces::msg::KfsRawDetections &
  raw_slot(robot_r2_interfaces::msg::KfsFusedRawDetections &batch,
           std::size_t camera_index) {
    switch (camera_index) {
    case 0:
      return batch.front;
    case 1:
      return batch.left;
    default:
      return batch.right;
    }
  }

  static robot_r2_interfaces::msg::KfsProcessedDetection &
  processed_slot(robot_r2_interfaces::msg::KfsFusedProcessedDetections &batch,
                 std::size_t camera_index) {
    switch (camera_index) {
    case 0:
      return batch.front;
    case 1:
      return batch.left;
    default:
      return batch.right;
    }
  }

  static sensor_msgs::msg::Image &
  debug_slot(robot_r2_interfaces::msg::KfsFusedDebugImages &batch,
             std::size_t camera_index) {
    switch (camera_index) {
    case 0:
      return batch.front;
    case 1:
      return batch.left;
    default:
      return batch.right;
    }
  }

  void fill_result(std::size_t camera_index, const CameraFrame &frame,
                   const FrameView &view,
                   const Classification &classification,
                   const KfsConfiguration &configuration,
                   robot_r2_interfaces::msg::KfsFusedRawDetections &raw_batch,
                   robot_r2_interfaces::msg::KfsFusedProcessedDetections
                       &processed_batch) {
    CameraState &state = *cameras_.at(camera_index);
    const std::string &class_name =
        configuration.class_names.at(classification.class_id);
    const auto header = make_header(frame);

    robot_r2_interfaces::msg::KfsRawBox box;
    box.class_name = class_name;
    box.class_id = classification.class_id;
    box.confidence = classification.confidence;
    auto &raw = raw_slot(raw_batch, camera_index);
    raw.header = header;
    raw.boxes.push_back(std::move(box));

    auto &processed = processed_slot(processed_batch, camera_index);
    processed.header = header;
    processed.image_width = static_cast<std::int32_t>(view.width);
    processed.image_height = static_cast<std::int32_t>(view.height);
    if (classification.confidence >= configuration.confidence_threshold) {
      processed.class_name = class_name;
      processed.confidence = classification.confidence;
    }
    record_vote(state, processed.class_name);
  }

  void fill_debug(std::size_t camera_index, const CameraFrame &frame,
                  const FrameView &view,
                  const Classification &classification,
                  const KfsConfiguration &configuration,
                  robot_r2_interfaces::msg::KfsFusedDebugImages &debug_batch) {
    const std::string &class_name =
        configuration.class_names.at(classification.class_id);
    auto &output = debug_slot(debug_batch, camera_index);
    output = make_debug_image(frame, view, class_name,
                              classification.confidence,
                              configuration.confidence_threshold);
  }

  static sensor_msgs::msg::Image
  make_debug_image(const CameraFrame &frame, const FrameView &view,
                   const std::string &class_name, float confidence,
                   double confidence_threshold) {
    cv::Mat source(view.height, view.width,
                   view.channels == 3 ? CV_8UC3 : CV_8UC1,
                   const_cast<std::uint8_t *>(view.data), view.step);
    cv::Mat bgr;
    if (view.channels == 1) {
      cv::cvtColor(source, bgr, cv::COLOR_GRAY2BGR);
    } else if (view.is_rgb) {
      cv::cvtColor(source, bgr, cv::COLOR_RGB2BGR);
    } else {
      bgr = source.clone();
    }
    const bool accepted = confidence >= confidence_threshold;
    cv::putText(bgr, class_name + " " + std::to_string(confidence).substr(0, 4),
                cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                accepted ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);

    sensor_msgs::msg::Image output;
    output.header = make_header(frame);
    output.height = bgr.rows;
    output.width = bgr.cols;
    output.encoding = "bgr8";
    output.step =
        static_cast<sensor_msgs::msg::Image::_step_type>(bgr.cols * 3);
    output.data.assign(bgr.datastart, bgr.dataend);
    return output;
  }

  static void record_vote(CameraState &state, const std::string &class_name) {
    std::lock_guard<std::mutex> lock(state.vote_mutex);
    if (!state.vote_active || state.vote_samples.size() >= state.vote_target) {
      return;
    }
    state.vote_samples.push_back(class_name);
    if (state.vote_samples.size() >= state.vote_target) {
      state.vote_condition.notify_all();
    }
  }

  static std::string select_vote(const std::vector<std::string> &samples) {
    std::string selected;
    std::size_t selected_count = 0;
    // Walk newest to oldest so ties prefer the most recent, matching Python.
    for (auto candidate = samples.rbegin(); candidate != samples.rend();
         ++candidate) {
      const std::size_t count = static_cast<std::size_t>(
          std::count(samples.begin(), samples.end(), *candidate));
      if (count > selected_count) {
        selected = *candidate;
        selected_count = count;
      }
    }
    return selected;
  }

  static robot_r2_interfaces::msg::KfsTypeResult &
  type_result_slot(GetKfsType::Response &response, std::size_t camera_index) {
    switch (camera_index) {
    case 0:
      return response.front;
    case 1:
      return response.left;
    default:
      return response.right;
    }
  }

  void handle_vote_service(
      const std::shared_ptr<GetKfsType::Request> request,
      const std::shared_ptr<GetKfsType::Response> response) {
    if (request->sample_count == 0) {
      response->success = false;
      response->message = "sample_count must be positive";
      return;
    }
    if (!std::isfinite(request->timeout_sec)) {
      response->success = false;
      response->message = "timeout_sec must be finite";
      return;
    }
    const std::size_t sample_count = request->sample_count;
    const auto configuration = std::atomic_load(&configuration_);
    const double timeout = request->timeout_sec > 0.0
                               ? request->timeout_sec
                               : configuration->default_vote_timeout_sec;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(timeout);

    // Serialize concurrent GetKfsType requests so they never fight over the
    // shared per-camera vote state.
    std::unique_lock<std::mutex> service_lock(service_mutex_);
    for (auto &camera : cameras_) {
      std::lock_guard<std::mutex> vote_lock(camera->vote_mutex);
      camera->vote_samples.clear();
      camera->vote_target = sample_count;
      camera->vote_active = true;
    }

    // Poll all three cameras until every one collected enough samples or the
    // deadline expires. Per-camera conditions fire independently, so a short
    // poll is simpler than waiting on three condition variables at once.
    while (rclcpp::ok()) {
      bool all_complete = true;
      for (const auto &camera : cameras_) {
        std::lock_guard<std::mutex> vote_lock(camera->vote_mutex);
        if (camera->vote_samples.size() < sample_count) {
          all_complete = false;
        }
      }
      if (all_complete) {
        break;
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    bool all_succeeded = true;
    for (std::size_t index = 0; index < cameras_.size(); ++index) {
      CameraState &camera = *cameras_[index];
      std::vector<std::string> samples;
      {
        std::lock_guard<std::mutex> vote_lock(camera.vote_mutex);
        samples = camera.vote_samples;
        camera.vote_active = false;
        camera.vote_target = 0;
      }
      auto &result = type_result_slot(*response, index);
      if (samples.size() < sample_count) {
        all_succeeded = false;
        result.success = false;
        result.message =
            "Detection vote timed out after collecting " +
            std::to_string(samples.size()) + "/" +
            std::to_string(sample_count) + " samples";
        result.class_name = "";
        continue;
      }
      result.success = true;
      result.message = "Selected most frequent class from " +
                       std::to_string(samples.size()) + " samples";
      result.class_name = select_vote(samples);
    }

    response->success = all_succeeded;
    response->message =
        all_succeeded ? "Selected most frequent classes for all cameras"
                      : "One or more cameras timed out during voting";
  }

  std::mutex frame_mutex_;
  std::vector<std::unique_ptr<CameraState>> cameras_;
  std::vector<rclcpp::Subscription<CameraFrame>::SharedPtr> subscriptions_;
  rclcpp::Publisher<robot_r2_interfaces::msg::KfsFusedRawDetections>::SharedPtr
      raw_publisher_;
  rclcpp::Publisher<
      robot_r2_interfaces::msg::KfsFusedProcessedDetections>::SharedPtr
      processed_publisher_;
  rclcpp::Publisher<robot_r2_interfaces::msg::KfsFusedDebugImages>::SharedPtr
      debug_publisher_;
  rclcpp::Service<GetKfsType>::SharedPtr service_;
  std::mutex service_mutex_;
  rclcpp::CallbackGroup::SharedPtr image_callback_group_;
  rclcpp::CallbackGroup::SharedPtr inference_callback_group_;
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::TimerBase::SharedPtr inference_timer_;
  std::unique_ptr<TensorRtResNet> engine_;
  std::mutex engine_mutex_;
  std::shared_ptr<const KfsConfiguration> configuration_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameter_callback_handle_;
};

} // namespace
} // namespace robot_r2_detect_cpp

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_r2_detect_cpp::FusedKfsNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                    6);
  executor.add_node(node);
  try {
    executor.spin();
  } catch (const std::exception &error) {
    RCLCPP_FATAL(node->get_logger(), "Fused KFS node stopped: %s",
                 error.what());
  }
  executor.remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
