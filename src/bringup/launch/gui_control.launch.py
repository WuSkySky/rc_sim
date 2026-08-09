import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    control_pkg = get_package_share_directory('robot_r2_control')
    gui_config = os.path.join(
        control_pkg,
        'config',
        'gui_control.yaml',
    )

    gui_control = Node(
        package='robot_r2_control',
        executable='gui_control',
        parameters=[gui_config],
        output='screen',
    )

    return LaunchDescription([gui_control])
