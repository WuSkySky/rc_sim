import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg = get_package_share_directory('bringup')
    sim_to_real_pkg = get_package_share_directory('sim_to_real')

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bringup_pkg,
                'launch',
                'control.launch.py',
            )
        ),
        launch_arguments={
            'simulation_state_detection': 'false',
        }.items(),
    )

    serial_bridge_config = os.path.join(
        sim_to_real_pkg,
        'config',
        'serial_bridge.yaml',
    )
    serial_bridge = Node(
        package='sim_to_real',
        executable='serial_bridge',
        parameters=[
            serial_bridge_config,
            {'receive_feedback_enabled': True},
        ],
        output='screen',
    )

    return LaunchDescription([
        control_launch,
        serial_bridge,
    ])
