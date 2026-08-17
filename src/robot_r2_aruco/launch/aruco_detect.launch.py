"""Launch aruco_detect."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("robot_r2_aruco")
    interfaces_share = get_package_share_directory("robot_r2_interfaces")
    fastdds_profile = os.path.join(
        interfaces_share,
        "config",
        "fastdds_camera.xml",
    )
    aruco_config = os.path.join(
        package_share,
        "config",
        "aruco_detect.yaml",
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
                package="robot_r2_aruco",
                executable="aruco_detect",
                name="aruco_detect",
                output="screen",
                parameters=[aruco_config],
                remappings=[
                    ("image_raw", "/r2/front_camera/image_raw"),
                    ("camera_info", "/r2/front_camera/camera_info"),
                    ("detections", "/r2/aruco/detections"),
                    ("debug", "/r2/aruco/debug"),
                ],
            ),
        ]
    )
