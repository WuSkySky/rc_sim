cmake_minimum_required(VERSION 3.8)
project(test_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core)
find_package(rclcpp REQUIRED)
find_package(robot_r2_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

ament_python_install_package(robot_r2_tests)

add_executable(camera_benchmark src/camera_benchmark.cpp)
ament_target_dependencies(
  camera_benchmark
  rclcpp
  robot_r2_interfaces
  sensor_msgs
)
target_include_directories(camera_benchmark PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(camera_benchmark ${OpenCV_LIBRARIES})

install(
  TARGETS camera_benchmark
  DESTINATION lib/${PROJECT_NAME}
)
install(
  PROGRAMS robot_r2_tests/move_to_center.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME move_to_center_test
)
install(
  PROGRAMS robot_r2_tests/step_traverse.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME step_traverse_test
)
install(
  PROGRAMS robot_r2_tests/rotate_90_servo.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME rotate_90_servo_test
)
install(
  PROGRAMS robot_r2_tests/odin_xy_calibration.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME odin_xy_calibration_test
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
