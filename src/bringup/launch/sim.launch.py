import os

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    field_pkg = get_package_share_directory('rc2026_field')
    robot_pkg = get_package_share_directory('robot_r2_description')
    robot_prefix = get_package_prefix('robot_r2_description')
    interfaces_pkg = get_package_share_directory('robot_r2_interfaces')
    controller_pkg = get_package_share_directory('robot_r2_controller')
    detect_pkg = get_package_share_directory('robot_r2_detect')
    roi_pkg = get_package_share_directory('robot_r2_kfs_roi')
    fastdds_profile = os.path.join(
        interfaces_pkg,
        'config',
        'fastdds_camera.xml',
    )

    field_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                field_pkg,
                'launch',
                'rc2026_field_sim_with_controller.launch.py'
            )
        )
    )

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                os.path.dirname(__file__),
                'control.launch.py',
            )
        )
    )

    robot_r2_urdf = os.path.join(
        robot_pkg, 'urdf', 'robot_r2.urdf'
    )

    spawn_robot_r2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'robot_r2',
            '-file', robot_r2_urdf,
            '-x', '5.56',
            '-y', '-1.4',
            '-z', '0.3',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '3.14',
        ],
        output='screen',
    )

    kfs_alignment_config = os.path.join(
        controller_pkg,
        'config',
        'kfs_alignment.yaml',
    )
    kfs_alignment = Node(
        package='robot_r2_controller',
        executable='kfs_alignment',
        parameters=[kfs_alignment_config],
        output='screen',
    )

    kfs_roi_config = os.path.join(
        roi_pkg,
        'config',
        'kfs_roi.yaml',
    )
    kfs_roi = Node(
        package='robot_r2_kfs_roi',
        executable='kfs_roi',
        name='kfs_roi',
        parameters=[kfs_roi_config],
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
                '/r2/front_camera/image_raw',
            ),
        ],
        output='screen',
    )

    teleop_control = Node(
        package='robot_r2_control',
        executable='teleop_control',
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
        AppendEnvironmentVariable(
            'GAZEBO_PLUGIN_PATH',
            os.path.join(robot_prefix, 'lib'),
        ),
        field_launch,
        spawn_robot_r2,
        control_launch,
        kfs_roi,
        kfs_alignment,
        led_detect,
        teleop_control,
    ])
