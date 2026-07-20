import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg = get_package_share_directory('bringup')
    odin_driver_pkg = get_package_share_directory('odin_ros_driver')
    odin_data_postprocess_pkg = get_package_share_directory(
        'odin_data_postprocess')
    serial_pkg = get_package_share_directory('serial_pkg')

    odin_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                odin_driver_pkg,
                'launch',
                'odin1_ros2_no_rviz.launch.py',
            )
        )
    )

    odometry_pose_config = os.path.join(
        odin_data_postprocess_pkg,
        'config',
        'odometry_pose_republisher.yaml',
    )
    odometry_pose_republisher = Node(
        package='odin_data_postprocess',
        executable='odometry_pose_republisher',
        parameters=[odometry_pose_config],
        output='screen',
    )

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
        serial_pkg,
        'config',
        'serial_bridge.yaml',
    )
    serial_bridge = Node(
        package='serial_pkg',
        executable='serial_bridge',
        parameters=[
            serial_bridge_config,
            {'receive_feedback_enabled': True},
        ],
        output='screen',
    )

    return LaunchDescription([
        odin_launch,
        odometry_pose_republisher,
        control_launch,
        serial_bridge,
    ])
