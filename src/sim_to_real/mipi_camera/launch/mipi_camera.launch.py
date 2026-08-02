import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('mipi_camera')
    interfaces_share = get_package_share_directory('robot_r2_interfaces')
    config = os.path.join(
        package_share,
        'config',
        'mipi_camera.yaml',
    )
    fastdds_profile = os.path.join(
        interfaces_share,
        'config',
        'fastdds_camera.xml',
    )

    left_camera = Node(
        package='mipi_camera',
        executable='mipi_camera',
        name='left_mipi_camera',
        parameters=[config],
        remappings=[
            ('/r2/mipi_camera/image_raw', '/r2/left_camera/image_raw'),
            (
                '/r2/mipi_camera/image_raw/debug',
                '/r2/left_camera/image_raw/debug',
            ),
            (
                '/r2/mipi_camera/camera_info',
                '/r2/left_camera/camera_info',
            ),
        ],
        output='screen',
    )
    right_camera = Node(
        package='mipi_camera',
        executable='mipi_camera',
        name='right_mipi_camera',
        parameters=[config],
        remappings=[
            ('/r2/mipi_camera/image_raw', '/r2/right_camera/image_raw'),
            (
                '/r2/mipi_camera/image_raw/debug',
                '/r2/right_camera/image_raw/debug',
            ),
            (
                '/r2/mipi_camera/camera_info',
                '/r2/right_camera/camera_info',
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            'RMW_IMPLEMENTATION',
            'rmw_fastrtps_cpp',
        ),
        SetEnvironmentVariable(
            'RMW_FASTRTPS_USE_QOS_FROM_XML',
            '1',
        ),
        SetEnvironmentVariable(
            'FASTDDS_DEFAULT_PROFILES_FILE',
            fastdds_profile,
        ),
        # Fast DDS releases shipped with ROS 2 Humble use the legacy name.
        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE',
            fastdds_profile,
        ),
        left_camera,
        right_camera,
    ])
