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
    odin_driver_pkg = get_package_share_directory('odin_ros_driver')
    odin_data_postprocess_pkg = get_package_share_directory(
        'odin_data_postprocess')
    serial_pkg = get_package_share_directory('serial_pkg')
    control_pkg = get_package_share_directory('robot_r2_control')
    detect_pkg = get_package_share_directory('robot_r2_detect')
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

    odin_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                odin_driver_pkg,
                'launch',
                'odin1_ros2_no_rviz.launch.py',
            )
        )
    )

    camera_frame_config = os.path.join(
        odin_data_postprocess_pkg,
        'config',
        'camera_frame_postprocess.yaml',
    )
    camera_frame_postprocess = Node(
        package='odin_data_postprocess',
        executable='camera_frame_postprocess',
        parameters=[camera_frame_config],
        output='screen',
    )

    odometry_config = os.path.join(
        odin_data_postprocess_pkg,
        'config',
        'odometry_postprocess.yaml',
    )
    odometry_postprocess = Node(
        package='odin_data_postprocess',
        executable='odometry_postprocess',
        parameters=[odometry_config],
        output='screen',
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

    led_detect_config = os.path.join(
        detect_pkg,
        'config',
        'led_detect.yaml',
    )
    led_detect = Node(
        package='robot_r2_detect',
        executable='led_detect',
        name='led_detect',
        parameters=[led_detect_config],
        remappings=[
            (
                '/r2/left_camera/image_raw',
                '/r2/rear_camera/image_raw',
            ),
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
        odin_launch,
        camera_frame_postprocess,
        odometry_postprocess,
        control_launch,
        serial_bridge,
        kfs_alignment,
        tip_alignment,
        led_detect,
    ])
