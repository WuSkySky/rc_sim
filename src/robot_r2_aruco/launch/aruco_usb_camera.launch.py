"""Launch usb_camera_bridge + aruco_detect + debug_saver for testing."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("robot_r2_aruco")
    interfaces_share = get_package_share_directory("robot_r2_interfaces")
    fastdds_profile = os.path.join(
        interfaces_share, "config", "fastdds_camera.xml",
    )
    aruco_config = os.path.join(
        package_share, "config", "aruco_detect.yaml",
    )
    usb_camera_config = os.path.join(
        package_share, "config", "usb_camera_bridge.yaml",
    )

    return LaunchDescription([
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
        SetEnvironmentVariable("RMW_FASTRTPS_USE_QOS_FROM_XML", "1"),
        SetEnvironmentVariable("FASTDDS_DEFAULT_PROFILES_FILE", fastdds_profile),
        SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", fastdds_profile),

        Node(
            package="robot_r2_aruco",
            executable="usb_camera_bridge",
            name="usb_camera_bridge",
            output="screen",
            parameters=[usb_camera_config],
        ),
        Node(
            package="robot_r2_aruco",
            executable="aruco_detect",
            name="aruco_detect",
            output="screen",
            parameters=[aruco_config],
            remappings=[
                ("image_raw", "/image_raw"),
                ("camera_info", "/camera_info"),
                ("detections", "/r2/aruco/detections"),
                ("debug", "/r2/aruco/debug"),
            ],
        ),
        Node(
            package="robot_r2_aruco",
            executable="debug_saver",
            name="debug_saver",
            output="screen",
        ),
    ])
