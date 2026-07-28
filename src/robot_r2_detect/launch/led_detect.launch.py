"""Launch led_detect."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("robot_r2_detect")
    interfaces_share = get_package_share_directory("robot_r2_interfaces")
    fastdds_profile = os.path.join(
        interfaces_share,
        "config",
        "fastdds_camera.xml",
    )
    led_detect_config = os.path.join(
        package_share,
        "config",
        "led_detect.yaml",
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                "RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"
            ),
            SetEnvironmentVariable(
                "RMW_FASTRTPS_USE_QOS_FROM_XML", "1"
            ),
            SetEnvironmentVariable(
                "FASTDDS_DEFAULT_PROFILES_FILE", fastdds_profile
            ),
            SetEnvironmentVariable(
                "FASTRTPS_DEFAULT_PROFILES_FILE", fastdds_profile
            ),
            Node(
                package="robot_r2_detect",
                executable="led_detect",
                name="led_detect",
                output="screen",
                parameters=[led_detect_config],
            ),
        ]
    )
