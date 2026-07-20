# USAGE: ros2 launch odin_ros_driver odin1_ros2_no_rviz.launch.py
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('odin_ros_driver')
    control_config_path = os.path.join(
        package_dir, 'config', 'control_command.yaml')
    calibration_path = os.path.join(package_dir, 'config', 'calib.yaml')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=control_config_path,
        description='Path to the control config YAML file',
    )

    host_sdk_node = Node(
        package='odin_ros_driver',
        executable='host_sdk_sample',
        name='host_sdk_sample',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
        }],
        additional_env={
        'LD_LIBRARY_PATH':
            '/lib/aarch64-linux-gnu:'
            '/usr/lib/aarch64-linux-gnu'
        },
    )

    with open(control_config_path, 'r') as config_file:
        pcd2depth_params = yaml.safe_load(config_file)
    pcd2depth_params['calib_file_path'] = calibration_path
    pcd2depth_node = Node(
        package='odin_ros_driver',
        executable='pcd2depth_ros2_node',
        name='pcd2depth_ros2_node',
        output='screen',
        parameters=[pcd2depth_params],
    )

    with open(control_config_path, 'r') as config_file:
        reprojection_params = yaml.safe_load(config_file)
    reprojection_params['calib_file_path'] = calibration_path
    cloud_reprojection_node = Node(
        package='odin_ros_driver',
        executable='cloud_reprojection_ros2_node',
        name='cloud_reprojection_ros2_node',
        output='screen',
        parameters=[reprojection_params],
    )

    with open(control_config_path, 'r') as config_file:
        overlay_params = yaml.safe_load(config_file)
    image_overlay_node = Node(
        package='odin_ros_driver',
        executable='image_overlay_node',
        name='image_overlay_node',
        output='screen',
        parameters=[overlay_params],
    )

    return LaunchDescription([
        config_file_arg,
        host_sdk_node,
        pcd2depth_node,
        cloud_reprojection_node,
        image_overlay_node,
    ])
