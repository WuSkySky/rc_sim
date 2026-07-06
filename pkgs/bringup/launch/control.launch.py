import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    control_pkg = get_package_share_directory('robot_r2_control')
    pose_servo_config = os.path.join(
        control_pkg,
        'config',
        'pose_servo.yaml',
    )

    wasd_teleop = Node(
        package='robot_r2_control',
        executable='wasd_teleop',
        output='screen',
    )

    pose_servo = Node(
        package='robot_r2_control',
        executable='pose_servo',
        parameters=[pose_servo_config],
        output='screen',
    )

    lift_service_controller = Node(
        package='robot_r2_control',
        executable='lift_service_controller',
        output='screen',
    )

    step_traverse_service = Node(
        package='robot_r2_control',
        executable='step_traverse_service',
        output='screen',
    )

    return LaunchDescription([
        wasd_teleop,
        pose_servo,
        lift_service_controller,
        step_traverse_service,
    ])
