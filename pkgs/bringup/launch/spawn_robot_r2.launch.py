import os

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    field_pkg = get_package_share_directory('rc2026_field')
    robot_pkg = get_package_share_directory('robot_r2_description')
    robot_prefix = get_package_prefix('robot_r2_description')

    field_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                field_pkg,
                'launch',
                'rc2026_field_sim_with_controller.launch.py'
            )
        )
    )

    robot_r2_sdf = os.path.join(
        robot_pkg, 'models', 'robot_r2', 'model.sdf'
    )

    spawn_robot_r2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'robot_r2',
            '-file', robot_r2_sdf,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01',
            '-Y', '0.0',
        ],
        output='screen',
    )

    return LaunchDescription([
        AppendEnvironmentVariable(
            'GAZEBO_PLUGIN_PATH',
            os.path.join(robot_prefix, 'lib'),
        ),
        field_launch,
        spawn_robot_r2,
    ])
