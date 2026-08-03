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

def _camera_arguments(camera):
    detection_prefix = f"/r2/detection/{camera}"
    return [
        DeclareLaunchArgument(
            f"{camera}_image_topic",
            default_value=DEFAULT_IMAGE_TOPICS[camera],
        ),
        DeclareLaunchArgument(
            f"{camera}_raw_topic",
            default_value=f"{detection_prefix}/raw",
        ),
        DeclareLaunchArgument(
            f"{camera}_processed_topic",
            default_value=f"{detection_prefix}/processed",
        ),
        DeclareLaunchArgument(
            f"{camera}_debug_topic",
            default_value=f"{detection_prefix}/debug",
        ),
        DeclareLaunchArgument(
            f"{camera}_get_type_service",
            default_value=f"{detection_prefix}/get_type",
        ),
    ]


def _fused_detector_node(detect_config):
    remappings = []
    for camera in CAMERAS:
        detection_prefix = f"/r2/detection/{camera}"
        remappings.extend(
            [
                (
                    DEFAULT_IMAGE_TOPICS[camera],
                    LaunchConfiguration(f"{camera}_image_topic"),
                ),
                (
                    f"{detection_prefix}/raw",
                    LaunchConfiguration(f"{camera}_raw_topic"),
                ),
                (
                    f"{detection_prefix}/processed",
                    LaunchConfiguration(f"{camera}_processed_topic"),
                ),
                (
                    f"{detection_prefix}/debug",
                    LaunchConfiguration(f"{camera}_debug_topic"),
                ),
                (
                    f"{detection_prefix}/get_type",
                    LaunchConfiguration(f"{camera}_get_type_service"),
                ),
            ]
        )

    return Node(
        package="robot_r2_detect_cpp",
        executable="kfs_detect_fused",
        name="kfs_detect_fused",
        output="screen",
        parameters=[detect_config],
        remappings=remappings,
    )


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

    actions = [
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
    ]

    for camera in CAMERAS:
        actions.extend(_camera_arguments(camera))
    actions.append(_fused_detector_node(detect_config))

    return LaunchDescription(actions)
