import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    interfaces_share = get_package_share_directory('robot_r2_interfaces')
    fastdds_profile = os.path.join(
        interfaces_share,
        'config',
        'fastdds_camera.xml',
    )

    message_type = LaunchConfiguration('message_type')
    processing_mode = LaunchConfiguration('processing_mode')
    warmup_sec = LaunchConfiguration('warmup_sec')
    duration_sec = LaunchConfiguration('duration_sec')
    left = LaunchConfiguration('left')
    right = LaunchConfiguration('right')

    common_parameters = {
        'message_type': message_type,
        'processing_mode': processing_mode,
        'warmup_sec': ParameterValue(warmup_sec, value_type=float),
        'duration_sec': ParameterValue(duration_sec, value_type=float),
    }

    left_benchmark = Node(
        package='test_pkg',
        executable='camera_benchmark',
        name='left_camera_benchmark',
        parameters=[
            common_parameters,
            {'topic': '/r2/left_camera/image_raw'},
        ],
        condition=IfCondition(left),
        output='screen',
    )
    right_benchmark = Node(
        package='test_pkg',
        executable='camera_benchmark',
        name='right_camera_benchmark',
        parameters=[
            common_parameters,
            {'topic': '/r2/right_camera/image_raw'},
        ],
        condition=IfCondition(right),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'message_type',
            default_value='bounded',
            description='standard (sensor_msgs/Image) or bounded (CameraFrame)',
        ),
        DeclareLaunchArgument(
            'processing_mode',
            default_value='transport',
            description='transport or opencv_mean',
        ),
        DeclareLaunchArgument('warmup_sec', default_value='3.0'),
        DeclareLaunchArgument('duration_sec', default_value='20.0'),
        DeclareLaunchArgument(
            'left',
            default_value='true',
            description='Benchmark the left camera topic',
        ),
        DeclareLaunchArgument(
            'right',
            default_value='false',
            description='Benchmark the right camera topic',
        ),
        SetEnvironmentVariable(
            'RMW_IMPLEMENTATION',
            'rmw_fastrtps_cpp',
        ),
        SetEnvironmentVariable(
            'RMW_FASTRTPS_USE_QOS_FROM_XML',
            '1',
        ),
        SetEnvironmentVariable(
            'FASTDDS_DEFAULT_PROFILES_FILE',
            fastdds_profile,
        ),
        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE',
            fastdds_profile,
        ),
        left_benchmark,
        right_benchmark,
    ])
