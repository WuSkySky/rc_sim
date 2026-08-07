cmake_minimum_required(VERSION 3.18)
project(robot_r2_detect_cpp LANGUAGES CXX CUDA)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

# Jetson Orin is compute capability 8.7. Override this cache value if the
# package is intentionally built for a different NVIDIA GPU.
set(ROBOT_R2_CUDA_ARCHITECTURES "87" CACHE STRING
  "CUDA architectures used by robot_r2_detect_cpp")
set(CMAKE_CUDA_ARCHITECTURES "${ROBOT_R2_CUDA_ARCHITECTURES}")

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_r2_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(CUDAToolkit REQUIRED)

find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  HINTS
    /usr/include/aarch64-linux-gnu
    /usr/include/x86_64-linux-gnu
    /usr/local/include)
find_library(TENSORRT_LIBRARY nvinfer
  HINTS
    /usr/lib/aarch64-linux-gnu
    /usr/lib/x86_64-linux-gnu
    /usr/local/lib)

if(NOT TENSORRT_INCLUDE_DIR OR NOT TENSORRT_LIBRARY)
  message(FATAL_ERROR
    "TensorRT development files were not found. Install the JetPack "
    "libnvinfer-dev package before building robot_r2_detect_cpp.")
endif()

add_library(kfs_roi_detection STATIC
  src/kfs_roi_detection.cpp)

target_include_directories(kfs_roi_detection PUBLIC
  include
  ${OpenCV_INCLUDE_DIRS})

target_link_libraries(kfs_roi_detection
  ${OpenCV_LIBRARIES})

target_compile_options(kfs_roi_detection PRIVATE
  -Wall -Wextra -Wpedantic)

add_executable(kfs_roi
  src/kfs_roi_node.cpp)

target_include_directories(kfs_roi PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS})

ament_target_dependencies(kfs_roi
  rcl_interfaces
  rclcpp
  robot_r2_interfaces
  sensor_msgs
  std_msgs)

target_link_libraries(kfs_roi
  kfs_roi_detection
  ${OpenCV_LIBRARIES})

target_compile_options(kfs_roi PRIVATE
  -Wall -Wextra -Wpedantic)

add_executable(kfs_detect_fused
  src/fused_kfs_node.cpp
  src/resnet_preprocess.cu)

target_include_directories(kfs_detect_fused PRIVATE
  include
  ${TENSORRT_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS})

ament_target_dependencies(kfs_detect_fused
  ament_index_cpp
  rclcpp
  robot_r2_interfaces
  sensor_msgs
  std_msgs)

target_link_libraries(kfs_detect_fused
  CUDA::cudart
  ${TENSORRT_LIBRARY}
  ${OpenCV_LIBRARIES})

target_compile_options(kfs_detect_fused PRIVATE
  $<$<COMPILE_LANGUAGE:CXX>:-Wall;-Wextra;-Wpedantic>
  $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr>)

set_target_properties(kfs_detect_fused PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON)

install(TARGETS kfs_detect_fused kfs_roi
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_kfs_roi_detection
    test/test_kfs_roi_detection.cpp)
  target_include_directories(test_kfs_roi_detection PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(test_kfs_roi_detection
    kfs_roi_detection
    ${OpenCV_LIBRARIES})
endif()

ament_package()
