cmake_minimum_required(VERSION 3.8)
project(robot_r2_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LiftCommand.msg"
  "msg/LiftFeedback.msg"
  "msg/KfsRawBox.msg"
  "msg/KfsRawDetections.msg"
  "msg/KfsProcessedDetection.msg"
  "srv/MoveToPose.srv"
  "srv/SetLift.srv"
  "srv/SetGripperLift.srv"
  "srv/SetGripperRotate.srv"
  "srv/SetGripperGrip.srv"
  "srv/TraverseAdjacentStep.srv"
  "srv/AlignToKFS.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
