"""Launch led_detect."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("robot_r2_detect")
    led_detect_config = os.path.join(
        package_share,
        "config",
        "led_detect.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="robot_r2_detect",
                executable="led_detect",
                name="led_detect",
                output="screen",
                parameters=[led_detect_config],
            ),
        ]
    )
