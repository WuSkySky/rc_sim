import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    control_pkg = get_package_share_directory('robot_r2_control')
    controller_pkg = get_package_share_directory('robot_r2_controller')
    detect_pkg = get_package_share_directory('robot_r2_detect')

    stage_two_config = os.path.join(
        control_pkg,
        'config',
        'stage_two.yaml',
    )
    stage_two_point_one_config = os.path.join(
        control_pkg,
        'config',
        'stage_two_point_one.yaml',
    )
    stage_two_point_two_config = os.path.join(
        control_pkg,
        'config',
        'stage_two_point_two.yaml',
    )
    kfs_loader_config = os.path.join(
        control_pkg,
        'config',
        'kfs_loader.yaml',
    )
    step_traverse_config = os.path.join(
        control_pkg,
        'config',
        'step_traverse.yaml',
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
    kfs_lift_config = os.path.join(
        controller_pkg,
        'config',
        'kfs_lift.yaml',
    )
    kfs_gripper_rotate_config = os.path.join(
        controller_pkg,
        'config',
        'kfs_gripper_rotate.yaml',
    )
    kfs_gripper_tip_rotate_config = os.path.join(
        controller_pkg,
        'config',
        'kfs_gripper_tip_rotate.yaml',
    )
    kfs_gripper_grip_config = os.path.join(
        controller_pkg,
        'config',
        'kfs_gripper_grip.yaml',
    )
    kfs_detect_config = os.path.join(
        detect_pkg,
        'config',
        'kfs_detect.yaml',
    )

    stage_two_control = Node(
        package='robot_r2_control',
        executable='stage_two_control',
        parameters=[stage_two_config],
        output='screen',
    )

    stage_two_point_one = Node(
        package='robot_r2_control',
        executable='stage_two_point_one',
        parameters=[stage_two_point_one_config],
        output='screen',
    )

    stage_two_point_two = Node(
        package='robot_r2_control',
        executable='stage_two_point_two',
        parameters=[stage_two_point_two_config],
        output='screen',
    )

    kfs_loader_control = Node(
        package='robot_r2_control',
        executable='kfs_loader_control',
        parameters=[kfs_loader_config],
        output='screen',
    )

    step_traverse = Node(
        package='robot_r2_control',
        executable='step_traverse',
        parameters=[step_traverse_config],
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

    kfs_lift = Node(
        package='robot_r2_controller',
        executable='kfs_lift',
        parameters=[kfs_lift_config],
        output='screen',
    )

    kfs_gripper_rotate = Node(
        package='robot_r2_controller',
        executable='kfs_gripper_rotate',
        parameters=[kfs_gripper_rotate_config],
        output='screen',
    )

    kfs_gripper_tip_rotate = Node(
        package='robot_r2_controller',
        executable='kfs_gripper_tip_rotate',
        parameters=[kfs_gripper_tip_rotate_config],
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
            kfs_detect_config,
            {
                'model_path': 'best.pt',
                'color_topic': '/r2/front_camera/image_raw',
                'conf': 0.75,
                'simulation_state_detection': ParameterValue(
                    LaunchConfiguration('simulation_state_detection'),
                    value_type=bool,
                ),
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'simulation_state_detection',
            default_value='true',
            description=(
                'Infer KFS service results from the first simulation status '
                'and the live robot grid pose'
            ),
        ),
        stage_two_control,
        stage_two_point_one,
        stage_two_point_two,
        kfs_loader_control,
        step_traverse,
        chassis_pose_servo,
        chassis_lift,
        kfs_alignment,
        kfs_lift,
        kfs_gripper_rotate,
        kfs_gripper_tip_rotate,
        kfs_gripper_grip,
        kfs_detect,
    ])
