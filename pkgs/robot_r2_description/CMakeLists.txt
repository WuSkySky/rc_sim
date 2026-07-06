cmake_minimum_required(VERSION 3.8)
project(robot_r2_description)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_r2_interfaces REQUIRED)

add_library(robot_r2_planar_move SHARED
  src/robot_r2_planar_move.cpp
)

add_library(robot_r2_localizer SHARED
  src/robot_r2_localizer.cpp
)

add_library(robot_r2_lift_controller SHARED
  src/robot_r2_lift_controller.cpp
)

target_include_directories(robot_r2_planar_move PUBLIC
  ${GAZEBO_INCLUDE_DIRS}
)

target_include_directories(robot_r2_localizer PUBLIC
  ${GAZEBO_INCLUDE_DIRS}
)

target_include_directories(robot_r2_lift_controller PUBLIC
  ${GAZEBO_INCLUDE_DIRS}
)

ament_target_dependencies(robot_r2_planar_move
  gazebo_dev
  gazebo_ros
  geometry_msgs
  rclcpp
)

ament_target_dependencies(robot_r2_localizer
  gazebo_dev
  gazebo_ros
  geometry_msgs
  rclcpp
)

ament_target_dependencies(robot_r2_lift_controller
  gazebo_dev
  gazebo_ros
  rclcpp
  robot_r2_interfaces
)

target_link_libraries(robot_r2_planar_move
  ${GAZEBO_LIBRARIES}
)

target_link_libraries(robot_r2_localizer
  ${GAZEBO_LIBRARIES}
)

target_link_libraries(robot_r2_lift_controller
  ${GAZEBO_LIBRARIES}
)

install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS robot_r2_planar_move robot_r2_localizer robot_r2_lift_controller
  LIBRARY DESTINATION lib
)

ament_package()
