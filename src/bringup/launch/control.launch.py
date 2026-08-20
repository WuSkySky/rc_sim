import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    control_pkg = get_package_share_directory('robot_r2_control')
    controller_pkg = get_package_share_directory('robot_r2_controller')
    interfaces_pkg = get_package_share_directory('robot_r2_interfaces')
    fastdds_profile = os.path.join(
        interfaces_pkg,
        'config',
        'fastdds_camera.xml',
    )

    stage_two_config = os.path.join(
        control_pkg,
        'config',
        'stage_two.yaml',
    )
    stage_one_config = os.path.join(
        control_pkg,
        'config',
        'stage_one.yaml',
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
    stage_three_config = os.path.join(
        control_pkg,
        'config',
        'stage_three.yaml',
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
    weapon_rotate_config = os.path.join(
        controller_pkg,
        'config',
        'weapon_rotate.yaml',
    )
    weapon_grip_config = os.path.join(
        controller_pkg,
        'config',
        'weapon_grip.yaml',
    )

    stage_one = Node(
        package='robot_r2_control',
        executable='stage_one',
        parameters=[stage_one_config],
        condition=IfCondition(LaunchConfiguration('enable_stage_one')),
        output='screen',
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
        remappings=[
            (
                '/r2/detection/get_type',
                LaunchConfiguration('kfs_get_type_service'),
            ),
        ],
        output='screen',
    )

    stage_two_point_two = Node(
        package='robot_r2_control',
        executable='stage_two_point_two',
        parameters=[stage_two_point_two_config],
        remappings=[
            (
                '/r2/detection/get_type',
                LaunchConfiguration('kfs_get_type_service'),
            ),
        ],
        output='screen',
    )

    stage_three = Node(
        package='robot_r2_control',
        executable='stage_three',
        parameters=[stage_three_config],
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

    weapon_rotate = Node(
        package='robot_r2_controller',
        executable='weapon_rotate',
        parameters=[weapon_rotate_config],
        condition=IfCondition(LaunchConfiguration('enable_stage_one')),
        output='screen',
    )

    weapon_grip = Node(
        package='robot_r2_controller',
        executable='weapon_grip',
        parameters=[weapon_grip_config],
        condition=IfCondition(LaunchConfiguration('enable_stage_one')),
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
        DeclareLaunchArgument(
            'kfs_get_type_service',
            default_value='/r2/detection/get_type',
            description='KFS detection service used by stage controllers',
        ),
        DeclareLaunchArgument(
            'enable_stage_one',
            default_value='false',
            description='Start real-only Stage 1 and weapon controllers',
        ),
        stage_one,
        stage_two_control,
        stage_two_point_one,
        stage_two_point_two,
        stage_three,
        kfs_loader_control,
        step_traverse,
        chassis_pose_servo,
        chassis_lift,
        kfs_lift,
        kfs_gripper_rotate,
        kfs_gripper_tip_rotate,
        kfs_gripper_grip,
        weapon_rotate,
        weapon_grip,
    ])
