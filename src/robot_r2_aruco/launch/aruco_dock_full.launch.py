#!/usr/bin/env python3
"""Full one-shot ArUco docking stack (chassis feedback + docking).

Combines the chassis feedback (``serial_bridge`` + ``odometry_tf``) with the
per-camera ArUco stack (included from ``<camera>_aruco.launch.py``) and a
dedicated docking servo (``aruco_chassis_pose_servo``).

This launch replaces ``real1.launch.py`` for docking — do NOT run both at the
same time, or two servos would drive ``/r2/cmd_vel``.

Usage::

    ros2 launch robot_r2_aruco aruco_dock_full.launch.py camera:=yahboom

Teach-in / docking flow
-----------------------
1. Push R2 until the tip meets the rod, then read the target pose from the
   ``aruco_pose_bridge`` log (x / y / yaw, printed every 2 s).
2. Command the dock with the recorded pose (angles in radians)::

       ros2 service call /r2/aruco/move_to_pose robot_r2_interfaces/srv/MoveToPose \
         "{pose_source: serial, x: <x>, y: <y>, yaw: <yaw>, \
           position_tolerance: 0.01, \
           yaw_tolerance: 0.02, timeout_sec: 20.0}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _camera_setup(context):
    camera = LaunchConfiguration('camera').perform(context)
    aruco_share = get_package_share_directory('robot_r2_aruco')
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(aruco_share, 'launch', f'{camera}_aruco.launch.py'),
            ),
        ),
    ]


def generate_launch_description():
    interfaces_share = get_package_share_directory('robot_r2_interfaces')
    serial_share = get_package_share_directory('serial_pkg')
    controller_share = get_package_share_directory('robot_r2_controller')

    fastdds_profile = os.path.join(
        interfaces_share, 'config', 'fastdds_camera.xml',
    )
    serial_bridge_config = os.path.join(
        serial_share, 'config', 'serial_bridge.yaml',
    )
    odometry_tf_config = os.path.join(
        serial_share, 'config', 'odometry_tf.yaml',
    )
    servo_config = os.path.join(
        controller_share, 'config', 'aruco_chassis_pose_servo.yaml',
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
    odometry_tf = Node(
        package='serial_pkg',
        executable='odometry_tf',
        parameters=[odometry_tf_config],
        output='screen',
    )
    servo = Node(
        package='robot_r2_controller',
        executable='chassis_pose_servo',
        name='aruco_chassis_pose_servo',
        parameters=[servo_config],
        remappings=[
            ('/r2/pose_feedback', '/r2/aruco/pose_feedback'),
            ('/r2/move_to_pose', '/r2/aruco/move_to_pose'),
            ('/r2/move_relative', '/r2/aruco/move_relative'),
        ],
        output='screen',
    )

    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        SetEnvironmentVariable('RMW_FASTRTPS_USE_QOS_FROM_XML', '1'),
        SetEnvironmentVariable('FASTDDS_DEFAULT_PROFILES_FILE', fastdds_profile),
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', fastdds_profile),
        DeclareLaunchArgument(
            'camera',
            default_value='yahboom',
            choices=['yahboom', 'hik'],
            description='Camera stack to use for docking',
        ),
        serial_bridge,
        odometry_tf,
        servo,
        OpaqueFunction(function=_camera_setup),
    ])
