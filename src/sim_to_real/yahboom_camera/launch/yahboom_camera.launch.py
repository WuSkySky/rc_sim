import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('yahboom_camera')
    config = os.path.join(package_share, 'config', 'yahboom_camera.yaml')

    return LaunchDescription([
        Node(
            package='yahboom_camera',
            executable='yahboom_camera',
            name='yahboom_camera',
            parameters=[config],
            output='screen',
        ),
    ])
