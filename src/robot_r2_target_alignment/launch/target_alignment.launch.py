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

    debug_image_topic = LaunchConfiguration("debug_image_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
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
                        "use_sim_time": ParameterValue(
                            use_sim_time,
                            value_type=bool,
                        ),
                    },
                ],
                remappings=[
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
