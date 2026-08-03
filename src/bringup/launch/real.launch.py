import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg = get_package_share_directory('bringup')
    interfaces_pkg = get_package_share_directory('robot_r2_interfaces')
    mipi_camera_pkg = get_package_share_directory('mipi_camera')
    odin_driver_pkg = get_package_share_directory('odin_ros_driver')
    odin_data_postprocess_pkg = get_package_share_directory(
        'odin_data_postprocess')
    serial_pkg = get_package_share_directory('serial_pkg')
    detect_pkg = get_package_share_directory('robot_r2_detect')
    fastdds_profile = os.path.join(
        interfaces_pkg,
        'config',
        'fastdds_camera.xml',
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

    mipi_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                mipi_camera_pkg,
                'launch',
                'mipi_camera.launch.py',
            )
        )
    )

    kfs_detect_multi_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                detect_pkg,
                'launch',
                'kfs_detect_multi.launch.py',
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
            'start_kfs_detect': 'false',
            'kfs_get_type_service': '/r2/detection/front/get_type',
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
        SetEnvironmentVariable(
            'RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        SetEnvironmentVariable(
            'RMW_FASTRTPS_USE_QOS_FROM_XML', '1'),
        SetEnvironmentVariable(
            'FASTDDS_DEFAULT_PROFILES_FILE', fastdds_profile),
        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE', fastdds_profile),
        odin_launch,
        camera_frame_postprocess,
        mipi_camera_launch,
        kfs_detect_multi_launch,
        odometry_postprocess,
        control_launch,
        serial_bridge,
    ])
