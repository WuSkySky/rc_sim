import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


MIPI_IMAGE_TOPIC = '/r2/mipi_camera/image_raw'
MIPI_DEBUG_TOPIC = '/r2/mipi_camera/image_raw/debug'
MIPI_CAMERA_INFO_TOPIC = '/r2/mipi_camera/camera_info'


def mipi_camera_node(side, config):
    camera_prefix = f'/r2/{side}_camera'
    return Node(
        package='mipi_camera',
        executable='mipi_camera',
        name=f'{side}_mipi_camera',
        parameters=[config],
        remappings=[
            (MIPI_IMAGE_TOPIC, f'{camera_prefix}/image_raw'),
            (MIPI_DEBUG_TOPIC, f'{camera_prefix}/image_raw/debug'),
            (MIPI_CAMERA_INFO_TOPIC, f'{camera_prefix}/camera_info'),
        ],
        output='screen',
    )


def fused_kfs_detect_node(config):
    return Node(
        package='robot_r2_detect_cpp',
        executable='kfs_detect_fused',
        name='kfs_detect_fused',
        parameters=[config],
        output='screen',
    )


def generate_launch_description():
    interfaces_pkg = get_package_share_directory('robot_r2_interfaces')
    mipi_camera_pkg = get_package_share_directory('mipi_camera')
    detect_pkg = get_package_share_directory('robot_r2_detect')
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
    left_mipi_camera = mipi_camera_node('left', mipi_camera_config)
    right_mipi_camera = mipi_camera_node('right', mipi_camera_config)

    kfs_detect_config = os.path.join(
        detect_pkg,
        'config',
        'kfs_detect.yaml',
    )
    fused_kfs_detect = fused_kfs_detect_node(kfs_detect_config)

    kfs_roi_config = os.path.join(
        detect_pkg,
        'config',
        'kfs_roi.yaml',
    )
    kfs_roi = Node(
        package='robot_r2_detect_cpp',
        executable='kfs_roi',
        name='kfs_roi',
        parameters=[kfs_roi_config],
        remappings=[
            (
                '/r2/front_camera/image_raw',
                LaunchConfiguration('roi_image_topic'),
            ),
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
            'roi_image_topic',
            default_value='/r2/left_camera/image_raw',
            description='MIPI image topic used by the single KFS ROI node',
        ),
        left_mipi_camera,
        right_mipi_camera,
        fused_kfs_detect,
        kfs_roi,
    ])
