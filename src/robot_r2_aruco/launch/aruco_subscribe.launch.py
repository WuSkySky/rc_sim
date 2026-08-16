#!/usr/bin/env python3
"""Run the ArUco pipeline against an already-running camera (subscribe-only).

This launch does NOT start any camera driver. It starts aruco_detect,
aruco_camera_tf, aruco_pose_bridge, and debug_saver, and remaps them onto an
externally published camera — for example the tip MIPI / IMX219 started by
``real2.launch.py`` as ``/r2/left_camera/image_raw``.

Usage (defaults to the tip MIPI left camera)::

    ros2 launch robot_r2_aruco aruco_subscribe.launch.py

Or point it at any other running camera::

    ros2 launch robot_r2_aruco aruco_subscribe.launch.py \
        image_topic:=/r2/left_camera/image_raw \
        camera_info_topic:=/r2/left_camera/camera_info
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    aruco_share = get_package_share_directory('robot_r2_aruco')
    interfaces_share = get_package_share_directory('robot_r2_interfaces')

    fastdds_profile = os.path.join(
        interfaces_share, 'config', 'fastdds_camera.xml',
    )
    aruco_config = os.path.join(
        aruco_share, 'config', 'aruco_detect.yaml',
    )
    pose_bridge_config = os.path.join(
        aruco_share, 'config', 'aruco_pose_bridge.yaml',
    )
    camera_tf_config = os.path.join(
        aruco_share, 'config', 'aruco_camera_tf.yaml',
    )

    image_topic = LaunchConfiguration('image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')

    aruco_detect = Node(
        package='robot_r2_aruco',
        executable='aruco_detect',
        name='aruco_detect',
        output='screen',
        parameters=[aruco_config],
        remappings=[
            ('image_raw', image_topic),
            ('camera_info', camera_info_topic),
            ('detections', '/aruco/detections'),
            ('debug', '/aruco/debug'),
        ],
    )
    camera_tf = Node(
        package='robot_r2_aruco',
        executable='aruco_camera_tf',
        name='aruco_camera_tf',
        output='screen',
        parameters=[camera_tf_config],
        remappings=[
            ('camera_info', camera_info_topic),
        ],
    )
    pose_bridge = Node(
        package='robot_r2_aruco',
        executable='aruco_pose_bridge',
        name='aruco_pose_bridge',
        output='screen',
        parameters=[pose_bridge_config],
    )
    debug_saver = Node(
        package='robot_r2_aruco',
        executable='debug_saver',
        name='debug_saver',
        output='screen',
    )

    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        SetEnvironmentVariable('RMW_FASTRTPS_USE_QOS_FROM_XML', '1'),
        SetEnvironmentVariable('FASTDDS_DEFAULT_PROFILES_FILE', fastdds_profile),
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', fastdds_profile),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/r2/left_camera/image_raw',
            description='CameraFrame topic published by the running camera',
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/r2/left_camera/camera_info',
            description='CameraInfo topic published by the running camera',
        ),
        aruco_detect,
        camera_tf,
        pose_bridge,
        debug_saver,
    ])
