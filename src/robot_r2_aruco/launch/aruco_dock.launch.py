#!/usr/bin/env python3
"""One-shot ArUco docking stack.

Combines a per-camera ArUco stack (camera + aruco_detect + base_link->camera
static TF + aruco_pose_bridge, included from ``<camera>_aruco.launch.py``)
with a dedicated ``chassis_pose_servo`` instance that closes the loop on the
ArUco pose feedback.

Usage::

    ros2 launch robot_r2_aruco aruco_dock.launch.py camera:=yahboom

Teach-in / docking flow
-----------------------
1. Keep the lower machine running (serial_bridge + odometry_tf), which
   provides the ``base_link`` TF and executes ``/r2/cmd_vel``. Do NOT also run
   ``real1.launch.py``: its own ``chassis_pose_servo`` would fight this one
   for ``/r2/cmd_vel``.
2. Manually push R2 until the tip meets the rod, then read the target pose::

       ros2 topic echo /r2/aruco/pose_feedback

   Record ``position.x/y`` and the yaw of ``orientation`` (or read the yaw
   the bridge logs every 2 seconds).
3. Command the dock with the recorded pose (angles in radians)::

       ros2 service call /r2/aruco/move_to_pose robot_r2_interfaces/srv/MoveToPose \
         "{x: <x>, y: <y>, yaw: <yaw>, position_tolerance: 0.01, \
           yaw_tolerance: 0.02, timeout_sec: 20.0}"

The static ``base_link -> camera`` transform only needs to be approximately
right: the teach-in absorbs the mount offset error.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    camera = LaunchConfiguration('camera').perform(context)
    aruco_share = get_package_share_directory('robot_r2_aruco')
    controller_share = get_package_share_directory('robot_r2_controller')

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(aruco_share, 'launch', f'{camera}_aruco.launch.py'),
        ),
    )

    servo_config = os.path.join(
        controller_share, 'config', 'aruco_chassis_pose_servo.yaml',
    )
    servo = Node(
        package='robot_r2_controller',
        executable='chassis_pose_servo',
        name='chassis_pose_servo',
        output='screen',
        parameters=[servo_config],
        remappings=[
            ('/r2/pose_feedback', '/r2/aruco/pose_feedback'),
            ('/r2/move_to_pose', '/r2/aruco/move_to_pose'),
        ],
    )

    return [camera_launch, servo]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera',
            default_value='yahboom',
            choices=['yahboom', 'hik'],
            description='Camera stack to use for docking',
        ),
        OpaqueFunction(function=_setup),
    ])
