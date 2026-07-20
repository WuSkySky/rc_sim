"""Launch kfs_detect."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("robot_r2_detect"),
        "config",
        "kfs_detect.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("model_path", default_value="best.pt"),
            DeclareLaunchArgument(
                "color_topic", default_value="/r2/front_camera/image_raw"
            ),
            DeclareLaunchArgument("conf", default_value="0.75"),
            DeclareLaunchArgument(
                "simulation_state_detection", default_value="false"
            ),

            Node(
                package="robot_r2_detect",
                executable="kfs_detect",
                name="kfs_detect",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "model_path": LaunchConfiguration("model_path"),
                        "color_topic": LaunchConfiguration("color_topic"),
                        "conf": LaunchConfiguration("conf"),
                        "simulation_state_detection": ParameterValue(
                            LaunchConfiguration(
                                "simulation_state_detection"
                            ),
                            value_type=bool,
                        ),
                    }
                ],
            ),
        ]
    )
