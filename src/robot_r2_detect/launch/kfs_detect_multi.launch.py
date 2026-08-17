"""Launch one fused C++/CUDA classifier for front, left, and right cameras."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


CAMERAS = ("front", "left", "right")
DEFAULT_IMAGE_TOPICS = {
    "front": "/r2/front_camera/image_raw",
    "left": "/r2/left_camera/image_raw",
    "right": "/r2/right_camera/image_raw",
}


def generate_launch_description():
    interfaces_share = get_package_share_directory("robot_r2_interfaces")
    config_directory = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "config")
    )
    fastdds_profile = os.path.join(
        interfaces_share,
        "config",
        "fastdds_camera.xml",
    )
    detect_config = os.path.join(
        config_directory,
        "kfs_detect.yaml",
    )

    remappings = [
        (
            DEFAULT_IMAGE_TOPICS[camera],
            LaunchConfiguration(f"{camera}_image_topic"),
        )
        for camera in CAMERAS
    ]
    remappings.extend(
        [
            ("/r2/detection/raw", LaunchConfiguration("raw_topic")),
            ("/r2/detection/processed", LaunchConfiguration("processed_topic")),
            (
                "/r2/detection/front/debug",
                LaunchConfiguration("front_debug_topic"),
            ),
            (
                "/r2/detection/left/debug",
                LaunchConfiguration("left_debug_topic"),
            ),
            (
                "/r2/detection/right/debug",
                LaunchConfiguration("right_debug_topic"),
            ),
            ("/r2/detection/get_type", LaunchConfiguration("get_type_service")),
        ]
    )

    fused_detector = Node(
        package="robot_r2_detect_cpp",
        executable="kfs_detect_fused",
        name="kfs_detect_fused",
        output="screen",
        parameters=[detect_config],
        remappings=remappings,
    )

    actions = [
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
        SetEnvironmentVariable("RMW_FASTRTPS_USE_QOS_FROM_XML", "1"),
        SetEnvironmentVariable("FASTDDS_DEFAULT_PROFILES_FILE", fastdds_profile),
        SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", fastdds_profile),
    ]

    for camera in CAMERAS:
        actions.append(
            DeclareLaunchArgument(
                f"{camera}_image_topic",
                default_value=DEFAULT_IMAGE_TOPICS[camera],
            )
        )
    actions.extend(
        [
            DeclareLaunchArgument(
                "raw_topic", default_value="/r2/detection/raw"
            ),
            DeclareLaunchArgument(
                "processed_topic", default_value="/r2/detection/processed"
            ),
            DeclareLaunchArgument(
                "front_debug_topic", default_value="/r2/detection/front/debug"
            ),
            DeclareLaunchArgument(
                "left_debug_topic", default_value="/r2/detection/left/debug"
            ),
            DeclareLaunchArgument(
                "right_debug_topic", default_value="/r2/detection/right/debug"
            ),
            DeclareLaunchArgument(
                "get_type_service", default_value="/r2/detection/get_type"
            ),
        ]
    )
    actions.append(fused_detector)

    return LaunchDescription(actions)
