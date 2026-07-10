import os

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    field_pkg = get_package_share_directory('rc2026_field')
    robot_pkg = get_package_share_directory('robot_r2_description')
    robot_prefix = get_package_prefix('robot_r2_description')

    field_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                field_pkg,
                'launch',
                'rc2026_field_sim_with_controller.launch.py'
            )
        )
    )

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                os.path.dirname(__file__),
                'control.launch.py',
            )
        )
    )

    robot_r2_urdf = os.path.join(
        robot_pkg, 'urdf', 'robot_r2.urdf'
    )

    spawn_robot_r2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'robot_r2',
            '-file', robot_r2_urdf,
            '-x', '5.56',
            '-y', '-1.4',
            '-z', '0.3',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '3.14',
        ],
        output='screen',
    )

    # Load lift PID YAML after robot spawns (plugins need time to init)
    load_lift_pid = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'for i in $(seq 1 15); do '
            + f'ros2 param load /robot_r2_lift_controller {robot_pkg}/config/lift_pid.yaml 2>/dev/null && exit 0; '
            + 'sleep 2; '
            + 'done; '
            + 'echo "WARNING: lift PID load failed after 30s"'
        ],
        output='screen',
    )

    return LaunchDescription([
        AppendEnvironmentVariable(
            'GAZEBO_PLUGIN_PATH',
            os.path.join(robot_prefix, 'lib'),
        ),
        field_launch,
        spawn_robot_r2,
        control_launch,
        load_lift_pid,
    ])
