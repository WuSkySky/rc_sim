#!/usr/bin/env python3
"""One-shot ArUco docking against an already-running camera (subscribe-only).

Like ``aruco_dock_full.launch.py`` but the camera is NOT started here — it
subscribes to an externally running camera (by default the tip MIPI / IMX219
published by ``real1.launch.py`` as ``/r2/tip_camera/image_raw``).

Started nodes: serial_bridge + odometry_tf (chassis feedback), aruco_detect,
aruco_camera_tf, aruco_pose_bridge, and the dedicated docking servo
``aruco_chassis_pose_servo`` (service ``/r2/aruco/move_to_pose``, publishes
``/r2/cmd_vel``).

Usage::

    ros2 launch robot_r2_aruco aruco_dock_subscribe.launch.py

Or point at another running camera::

    ros2 launch robot_r2_aruco aruco_dock_subscribe.launch.py \
        image_topic:=/r2/left_camera/image_raw \
        camera_info_topic:=/r2/left_camera/camera_info

Note: serial_bridge needs the lower-machine serial port (default
``/dev/ttyACM0``). Do not run this together with ``real1.launch.py`` (two
servos would both drive ``/r2/cmd_vel``).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    interfaces_share = get_package_share_directory('robot_r2_interfaces')
    serial_share = get_package_share_directory('serial_pkg')
    controller_share = get_package_share_directory('robot_r2_controller')
    aruco_share = get_package_share_directory('robot_r2_aruco')

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
    aruco_detect = Node(
        package='robot_r2_aruco',
        executable='aruco_detect',
        name='aruco_detect',
        output='screen',
        parameters=[aruco_config],
        remappings=[
            ('image_raw', image_topic),
            ('camera_info', camera_info_topic),
            ('detections', '/r2/aruco/detections'),
            ('debug', '/r2/aruco/debug'),
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

    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        SetEnvironmentVariable('RMW_FASTRTPS_USE_QOS_FROM_XML', '1'),
        SetEnvironmentVariable('FASTDDS_DEFAULT_PROFILES_FILE', fastdds_profile),
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', fastdds_profile),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/r2/tip_camera/image_raw',
            description='CameraFrame topic published by the running camera',
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/r2/tip_camera/camera_info',
            description='CameraInfo topic published by the running camera',
        ),
        serial_bridge,
        odometry_tf,
        servo,
        aruco_detect,
        camera_tf,
        pose_bridge,
    ])
