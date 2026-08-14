"""启动左摄像头与端头检测,配合 real1 使用端头对齐功能。

real1.launch.py 已启动 tip_alignment(订阅 /r2/tip/roi)等控制节点,但缺少
端头检测的上游数据源。本 launch 只负责补齐这两部分:

- left_mipi_camera:左 MIPI 摄像头,发布 CameraFrame 到 /r2/left_camera/image_raw
- yolo_target_detector:端头 YOLO 检测,订阅左摄像头,发布 AlignmentDetection
  到 /r2/tip/roi(喂给 real1 的 tip_alignment)

运行时与 real1 配合使用:

    ros2 launch bringup real1.launch.py
    ros2 launch bringup test_left_alignment.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# mipi_camera 节点内部使用的通用 topic,需 remap 到左摄像头专用名称。
MIPI_IMAGE_TOPIC = '/r2/mipi_camera/image_raw'
MIPI_DEBUG_TOPIC = '/r2/mipi_camera/image_raw/debug'
MIPI_CAMERA_INFO_TOPIC = '/r2/mipi_camera/camera_info'


def generate_launch_description():
    interfaces_pkg = get_package_share_directory('robot_r2_interfaces')
    mipi_camera_pkg = get_package_share_directory('mipi_camera')
    target_alignment_pkg = get_package_share_directory(
        'robot_r2_target_alignment')
    fastdds_profile = os.path.join(
        interfaces_pkg,
        'config',
        'fastdds_camera.xml',
    )

    mipi_camera_config = os.path.join(
        mipi_camera_pkg,
        'config',
        'mipi_camera.yaml',
    )
    detector_config = os.path.join(
        target_alignment_pkg,
        'config',
        'yolo_target_detector.yaml',
    )

    image_topic = LaunchConfiguration('image_topic')

    left_mipi_camera = Node(
        package='mipi_camera',
        executable='mipi_camera',
        name='left_mipi_camera',
        parameters=[mipi_camera_config],
        remappings=[
            (MIPI_IMAGE_TOPIC, '/r2/left_camera/image_raw'),
            (MIPI_DEBUG_TOPIC, '/r2/left_camera/image_raw/debug'),
            (MIPI_CAMERA_INFO_TOPIC, '/r2/left_camera/camera_info'),
        ],
        output='screen',
    )

    yolo_target_detector = Node(
        package='robot_r2_target_alignment',
        executable='yolo_target_detector',
        namespace='r2/target_alignment',
        name='yolo_target_detector',
        parameters=[
            detector_config,
            {'input_video_topic': image_topic},
        ],
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
            'image_topic',
            default_value='/r2/left_camera/image_raw',
            description='端头检测输入的 CameraFrame 话题',
        ),
        left_mipi_camera,
        yolo_target_detector,
    ])
