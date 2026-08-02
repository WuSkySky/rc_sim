"""Launch front, left, and right ResNet KFS detectors."""

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

INTERNAL_IMAGE_TOPIC = "/r2/left_camera/image_raw"
INTERNAL_RAW_TOPIC = "/r2/detection/raw"
INTERNAL_PROCESSED_TOPIC = "/r2/detection/processed"
INTERNAL_DEBUG_TOPIC = "/r2/detection/debug"
INTERNAL_GET_TYPE_SERVICE = "/r2/detection/get_type"
INTERNAL_SIMULATION_STATUS_TOPIC = "/simulation/status"
INTERNAL_ROBOT_POSE_TOPIC = "/r2/pose_feedback"


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


def _detector_node(camera, detect_config):
    return Node(
        package="robot_r2_detect",
        executable="kfs_detect",
        name=f"kfs_detect_{camera}",
        output="screen",
        parameters=[detect_config],
        remappings=[
            (
                INTERNAL_IMAGE_TOPIC,
                LaunchConfiguration(f"{camera}_image_topic"),
            ),
            (
                INTERNAL_RAW_TOPIC,
                LaunchConfiguration(f"{camera}_raw_topic"),
            ),
            (
                INTERNAL_PROCESSED_TOPIC,
                LaunchConfiguration(f"{camera}_processed_topic"),
            ),
            (
                INTERNAL_DEBUG_TOPIC,
                LaunchConfiguration(f"{camera}_debug_topic"),
            ),
            (
                INTERNAL_GET_TYPE_SERVICE,
                LaunchConfiguration(f"{camera}_get_type_service"),
            ),
            (
                INTERNAL_SIMULATION_STATUS_TOPIC,
                LaunchConfiguration("simulation_status_topic"),
            ),
            (
                INTERNAL_ROBOT_POSE_TOPIC,
                LaunchConfiguration("robot_pose_topic"),
            ),
        ],
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
        DeclareLaunchArgument(
            "simulation_status_topic",
            default_value="/simulation/status",
        ),
        DeclareLaunchArgument(
            "robot_pose_topic",
            default_value="/r2/pose_feedback",
        ),
    ]

    for camera in CAMERAS:
        actions.extend(_camera_arguments(camera))
    for camera in CAMERAS:
        actions.append(_detector_node(camera, detect_config))

    return LaunchDescription(actions)
