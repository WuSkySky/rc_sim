"""Launch YOLO11 target detection and lateral chassis alignment."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory(
        "robot_r2_target_alignment"
    )
    detector_config = os.path.join(
        package_share,
        "config",
        "yolo_target_detector.yaml",
    )
    controller_config = os.path.join(
        package_share,
        "config",
        "target_alignment_controller.yaml",
    )

    camera_topic = LaunchConfiguration("camera_topic")
    debug_image_topic = LaunchConfiguration("debug_image_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    model_path = LaunchConfiguration("model_path")
    test_mode = LaunchConfiguration("test_mode")
    visualization_enabled = LaunchConfiguration("visualization_enabled")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_topic",
                default_value="/r2/front_camera/image_raw",
                description="CameraFrame input topic.",
            ),
            DeclareLaunchArgument(
                "debug_image_topic",
                default_value="/r2/target_alignment/debug_image",
                description="Annotated detection image output topic.",
            ),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="/r2/cmd_vel",
                description="Chassis Twist output topic.",
            ),
            DeclareLaunchArgument(
                "model_path",
                default_value=(
                    "package://robot_r2_detect/model/duantou.pt"
                ),
                description=(
                    "YOLO11 weights as an absolute path or package URI."
                ),
            ),
            DeclareLaunchArgument(
                "test_mode",
                default_value="true",
                description="Log candidate commands without publishing cmd_vel.",
            ),
            DeclareLaunchArgument(
                "visualization_enabled",
                default_value="true",
                description="Publish annotated detection images.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use the ROS simulation clock.",
            ),
            Node(
                package="robot_r2_target_alignment",
                executable="yolo_target_detector",
                namespace="r2/target_alignment",
                name="yolo_target_detector",
                parameters=[
                    detector_config,
                    {
                        "model.path": ParameterValue(
                            model_path,
                            value_type=str,
                        ),
                        "visualization.enabled": ParameterValue(
                            visualization_enabled,
                            value_type=bool,
                        ),
                        "use_sim_time": ParameterValue(
                            use_sim_time,
                            value_type=bool,
                        ),
                    },
                ],
                remappings=[
                    ("camera/image_raw", camera_topic),
                    ("debug_image", debug_image_topic),
                ],
                output="screen",
            ),
            Node(
                package="robot_r2_target_alignment",
                executable="target_alignment_controller",
                namespace="r2/target_alignment",
                name="target_alignment_controller",
                parameters=[
                    controller_config,
                    {
                        "test_mode": ParameterValue(
                            test_mode,
                            value_type=bool,
                        ),
                        "use_sim_time": ParameterValue(
                            use_sim_time,
                            value_type=bool,
                        ),
                    },
                ],
                remappings=[("cmd_vel", cmd_vel_topic)],
                output="screen",
            ),
        ]
    )
