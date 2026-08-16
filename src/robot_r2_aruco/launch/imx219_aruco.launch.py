#!/usr/bin/env python3
"""Launch one IMX219 (mipi_camera) + aruco_detect + TF + pose bridge.

The IMX219 driver publishes generic topics internally
(``/r2/mipi_camera/image_raw`` and ``/r2/mipi_camera/camera_info``); this
launch remaps them onto the ArUco pipeline's ``/image_raw`` and ``/camera_info``
so the detection, camera TF, and pose bridge nodes work unchanged.

The camera side is selected with the ``side`` argument (left/right). The node
name ``<side>_mipi_camera`` also selects the optical frame id automatically, and
``aruco_camera_tf`` reads that frame from ``camera_info``, so no frame name has
to be hard-coded here.

Usage::

    ros2 launch robot_r2_aruco imx219_aruco.launch.py side:=left

Note: mipi_camera needs the Jetson (IMX219 CSI + NVIDIA Argus). The camera
orientation in ``aruco_camera_tf.yaml`` must match how the IMX219 is actually
mounted (see camera_rpy there).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _mipi_camera_setup(context, mipi_config):
    side = LaunchConfiguration('side').perform(context)
    return [
        Node(
            package='mipi_camera',
            executable='mipi_camera',
            name=f'{side}_mipi_camera',
            parameters=[mipi_config],
            remappings=[
                ('/r2/mipi_camera/image_raw', '/image_raw'),
                ('/r2/mipi_camera/image_raw/debug', '/image_raw/debug'),
                ('/r2/mipi_camera/camera_info', '/camera_info'),
            ],
            output='screen',
        ),
    ]


def generate_launch_description():
    aruco_share = get_package_share_directory('robot_r2_aruco')
    mipi_share = get_package_share_directory('mipi_camera')
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
    mipi_config = os.path.join(
        mipi_share, 'config', 'mipi_camera.yaml',
    )

    aruco_detect = Node(
        package='robot_r2_aruco',
        executable='aruco_detect',
        name='aruco_detect',
        output='screen',
        parameters=[aruco_config],
        remappings=[
            ('image_raw', '/image_raw'),
            ('camera_info', '/camera_info'),
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
            ('camera_info', '/camera_info'),
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
            'side',
            default_value='left',
            choices=['left', 'right'],
            description='Which IMX219 CSI camera to use',
        ),
        aruco_detect,
        camera_tf,
        pose_bridge,
        debug_saver,
        OpaqueFunction(
            function=lambda context: _mipi_camera_setup(context, mipi_config),
        ),
    ])
