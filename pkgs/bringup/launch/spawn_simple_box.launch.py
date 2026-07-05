import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    field_pkg = get_package_share_directory('rc2026_field')
    robot_pkg = get_package_share_directory('robot_r2_description')

    field_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                field_pkg,
                'launch',
                'rc2026_field_sim_with_controller.launch.py'
            )
        )
    )

    simple_box_sdf = os.path.join(
        robot_pkg, 'models', 'SimpleBox', 'model.sdf'
    )

    spawn_simple_box = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'simple_box',
            '-file', simple_box_sdf,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '2.0',
            '-Y', '0.0',
        ],
        output='screen',
    )

    return LaunchDescription([
        field_launch,
        spawn_simple_box,
    ])
