import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('yahboom_camera')
    # HD config (new model, 05a3:9230, 1920x1080@30) for the standalone run.
    # For the old Sunplus SDYH-8P0P model (1bcf:0b09), switch to
    # yahboom_camera.yaml, which keeps the 1280x720@30 default.
    config = os.path.join(package_share, 'config', 'yahboom_camera_hd.yaml')

    return LaunchDescription([
        Node(
            package='yahboom_camera',
            executable='yahboom_camera',
            name='yahboom_camera',
            parameters=[config],
            output='screen',
        ),
    ])
