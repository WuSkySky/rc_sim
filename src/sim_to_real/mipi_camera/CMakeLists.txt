cmake_minimum_required(VERSION 3.8)
project(mipi_camera)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(robot_r2_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

pkg_check_modules(
  GSTREAMER
  REQUIRED
  IMPORTED_TARGET
  gstreamer-1.0
  gstreamer-app-1.0
)

add_executable(mipi_camera src/mipi_camera.cpp)
ament_target_dependencies(
  mipi_camera
  rclcpp
  rcl_interfaces
  robot_r2_interfaces
  sensor_msgs
)
target_link_libraries(mipi_camera PkgConfig::GSTREAMER)

install(
  TARGETS mipi_camera
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
