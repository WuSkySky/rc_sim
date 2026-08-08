import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('hik_camera')
    config = os.path.join(package_share, 'config', 'hik_camera.yaml')

    return LaunchDescription([
        Node(
            package='hik_camera',
            executable='hik_camera',
            name='hik_camera',
            parameters=[config],
            output='screen',
        ),
    ])
