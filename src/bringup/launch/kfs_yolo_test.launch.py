"""Launch the left MIPI camera, KFS ROI, and YOLO classifier."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    interfaces_share = get_package_share_directory("robot_r2_interfaces")
    camera_share = get_package_share_directory("mipi_camera")
    detect_share = get_package_share_directory("robot_r2_detect")
    roi_share = get_package_share_directory("robot_r2_kfs_roi")

    fastdds_profile = os.path.join(
        interfaces_share,
        "config",
        "fastdds_camera.xml",
    )
    camera_config = os.path.join(
        camera_share,
        "config",
        "mipi_camera.yaml",
    )
    roi_config = os.path.join(
        roi_share,
        "config",
        "kfs_roi.yaml",
    )
    detect_config = os.path.join(
        detect_share,
        "config",
        "kfs_detect_yolo.yaml",
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
                package="mipi_camera",
                executable="mipi_camera",
                name="left_mipi_camera",
                parameters=[camera_config],
                remappings=[
                    (
                        "/r2/mipi_camera/image_raw",
                        "/r2/left_camera/image_raw",
                    ),
                    (
                        "/r2/mipi_camera/image_raw/debug",
                        "/r2/left_camera/image_raw/debug",
                    ),
                    (
                        "/r2/mipi_camera/camera_info",
                        "/r2/left_camera/camera_info",
                    ),
                ],
                output="screen",
            ),
            Node(
                package="robot_r2_kfs_roi",
                executable="kfs_roi",
                name="kfs_roi",
                parameters=[roi_config],
                remappings=[
                    (
                        "/r2/front_camera/image_raw",
                        "/r2/left_camera/image_raw",
                    ),
                ],
                output="screen",
            ),
            Node(
                package="robot_r2_detect",
                executable="kfs_detect",
                name="kfs_detect",
                parameters=[detect_config],
                output="screen",
            ),
        ]
    )
