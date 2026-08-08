cmake_minimum_required(VERSION 3.8)
project(hik_camera)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(robot_r2_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
  set(MVS_ARCH amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  set(MVS_ARCH arm64)
else()
  message(FATAL_ERROR
    "hik_camera supports only x86_64 and aarch64, got ${CMAKE_SYSTEM_PROCESSOR}")
endif()

set(MVS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/mvs")
set(MVS_LIBRARY_DIR "${MVS_ROOT}/lib/${MVS_ARCH}")

add_executable(hik_camera src/hik_camera.cpp)
target_include_directories(hik_camera PRIVATE "${MVS_ROOT}/include")
ament_target_dependencies(
  hik_camera
  rclcpp
  rcl_interfaces
  robot_r2_interfaces
  sensor_msgs
)
target_link_libraries(
  hik_camera
  "${MVS_LIBRARY_DIR}/libFormatConversion.so"
  "${MVS_LIBRARY_DIR}/libMediaProcess.so"
  "${MVS_LIBRARY_DIR}/libMvCameraControl.so"
  "${MVS_LIBRARY_DIR}/libMVRender.so"
  "${MVS_LIBRARY_DIR}/libMvUsb3vTL.so"
)
set_target_properties(
  hik_camera
  PROPERTIES
    BUILD_RPATH "${MVS_LIBRARY_DIR}"
    INSTALL_RPATH "$ORIGIN/.."
)

install(
  TARGETS hik_camera
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY "${MVS_LIBRARY_DIR}/"
  DESTINATION lib
  FILES_MATCHING PATTERN "*.so"
)
install(
  DIRECTORY config launch udev
  DESTINATION share/${PROJECT_NAME}
)
install(
  FILES
    README.md
    LICENSE
    NOTICE
    third_party/mvs/CLIENT_MVS_Linux_license_notice.txt
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
