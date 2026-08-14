import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg = get_package_share_directory('bringup')
    interfaces_pkg = get_package_share_directory('robot_r2_interfaces')
    serial_pkg = get_package_share_directory('serial_pkg')
    control_pkg = get_package_share_directory('robot_r2_control')
    fastdds_profile = os.path.join(
        interfaces_pkg,
        'config',
        'fastdds_camera.xml',
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
            'kfs_get_type_service': LaunchConfiguration(
                'kfs_get_type_service'),
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

    odometry_tf_config = os.path.join(
        serial_pkg,
        'config',
        'odometry_tf.yaml',
    )
    odometry_tf = Node(
        package='serial_pkg',
        executable='odometry_tf',
        parameters=[odometry_tf_config],
        output='screen',
    )

    alignment_config = os.path.join(
        control_pkg,
        'config',
        'alignment.yaml',
    )
    kfs_alignment = Node(
        package='robot_r2_control',
        executable='alignment',
        name='kfs_alignment',
        parameters=[alignment_config],
        remappings=[
            ('/r2/alignment/detection', '/r2/kfs/roi'),
            ('/r2/alignment/cmd_vel', '/r2/cmd_vel'),
            ('/r2/alignment/align', '/r2/align_to_kfs'),
        ],
        output='screen',
    )
    tip_alignment = Node(
        package='robot_r2_control',
        executable='alignment',
        name='tip_alignment',
        parameters=[alignment_config, {'reverse_direction': True}],
        remappings=[
            ('/r2/alignment/detection', '/r2/tip/roi'),
            ('/r2/alignment/cmd_vel', '/r2/cmd_vel'),
            ('/r2/alignment/align', '/r2/align_to_tip'),
        ],
        output='screen',
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            'RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        SetEnvironmentVariable(
            'RMW_FASTRTPS_USE_QOS_FROM_XML', '1'),
        SetEnvironmentVariable(
            'FASTDDS_DEFAULT_PROFILES_FILE', fastdds_profile),
        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE', fastdds_profile),
        DeclareLaunchArgument(
            'kfs_get_type_service',
            default_value='/r2/detection/left/get_type',
            description='Remote KFS detection service used by control',
        ),
        control_launch,
        serial_bridge,
        odometry_tf,
        kfs_alignment,
        tip_alignment,
    ])
