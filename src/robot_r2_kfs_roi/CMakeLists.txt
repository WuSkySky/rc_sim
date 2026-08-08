cmake_minimum_required(VERSION 3.18)
project(robot_r2_kfs_roi LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_r2_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

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

install(TARGETS kfs_roi
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

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
