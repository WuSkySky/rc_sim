import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    control_pkg = get_package_share_directory('robot_r2_control')
    controller_pkg = get_package_share_directory('robot_r2_controller')

    step_traverse_config = os.path.join(
        control_pkg,
        'config',
        'step_traverse_service.yaml',
    )
    kfs_loader_config = os.path.join(
        control_pkg,
        'config',
        'kfs_loader.yaml',
    )

    chassis_pose_servo_config = os.path.join(
        controller_pkg,
        'config',
        'chassis_pose_servo.yaml',
    )
    chassis_lift_config = os.path.join(
        controller_pkg,
        'config',
        'chassis_lift.yaml',
    )
    kfs_alignment_config = os.path.join(
        controller_pkg,
        'config',
        'kfs_alignment.yaml',
    )
    kfs_gripper_lift_config = os.path.join(
        controller_pkg,
        'config',
        'kfs_gripper_lift.yaml',
    )
    kfs_gripper_rotate_config = os.path.join(
        controller_pkg,
        'config',
        'kfs_gripper_rotate.yaml',
    )
    kfs_gripper_grip_config = os.path.join(
        controller_pkg,
        'config',
        'kfs_gripper_grip.yaml',
    )

    teleop_controller = Node(
        package='robot_r2_control',
        executable='teleop_controller',
        output='screen',
    )

    step_traverse_controller = Node(
        package='robot_r2_control',
        executable='step_traverse_controller',
        parameters=[step_traverse_config],
        output='screen',
    )

    kfs_loader_controller = Node(
        package='robot_r2_control',
        executable='kfs_loader_controller',
        parameters=[kfs_loader_config],
        output='screen',
    )

    chassis_pose_servo = Node(
        package='robot_r2_controller',
        executable='chassis_pose_servo',
        parameters=[chassis_pose_servo_config],
        output='screen',
    )

    chassis_lift = Node(
        package='robot_r2_controller',
        executable='chassis_lift',
        parameters=[chassis_lift_config],
        output='screen',
    )

    kfs_alignment = Node(
        package='robot_r2_controller',
        executable='kfs_alignment',
        parameters=[kfs_alignment_config],
        output='screen',
    )

    kfs_gripper_lift = Node(
        package='robot_r2_controller',
        executable='kfs_gripper_lift',
        parameters=[kfs_gripper_lift_config],
        output='screen',
    )

    kfs_gripper_rotate = Node(
        package='robot_r2_controller',
        executable='kfs_gripper_rotate',
        parameters=[kfs_gripper_rotate_config],
        output='screen',
    )

    kfs_gripper_grip = Node(
        package='robot_r2_controller',
        executable='kfs_gripper_grip',
        parameters=[kfs_gripper_grip_config],
        output='screen',
    )

    kfs_detect = Node(
        package='robot_r2_detect',
        executable='kfs_detect',
        name='kfs_detect',
        output='screen',
        parameters=[
            {
                'model_path': 'best.pt',
                'color_topic': '/r2/front_camera/image_raw',
                'conf': 0.75,
            }
        ],
    )

    return LaunchDescription([
        teleop_controller,
        step_traverse_controller,
        kfs_loader_controller,
        chassis_pose_servo,
        chassis_lift,
        kfs_alignment,
        kfs_gripper_lift,
        kfs_gripper_rotate,
        kfs_gripper_grip,
        kfs_detect,
    ])
