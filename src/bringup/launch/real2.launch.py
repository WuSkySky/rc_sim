import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


MIPI_IMAGE_TOPIC = '/r2/mipi_camera/image_raw'
MIPI_DEBUG_TOPIC = '/r2/mipi_camera/image_raw/debug'
MIPI_CAMERA_INFO_TOPIC = '/r2/mipi_camera/camera_info'
YAHBOOM_IMAGE_TOPIC = '/r2/yahboom_camera/image_raw'
YAHBOOM_DEBUG_TOPIC = '/r2/yahboom_camera/image_raw/debug'
YAHBOOM_CAMERA_INFO_TOPIC = '/r2/yahboom_camera/camera_info'


def tip_mipi_camera_node(config):
    return Node(
        package='mipi_camera',
        executable='mipi_camera',
        name='tip_mipi_camera',
        parameters=[config],
        remappings=[
            (MIPI_IMAGE_TOPIC, '/r2/tip_camera/image_raw'),
            (MIPI_DEBUG_TOPIC, '/r2/tip_camera/image_raw/debug'),
            (MIPI_CAMERA_INFO_TOPIC, '/r2/tip_camera/camera_info'),
        ],
        output='screen',
    )


def front_yahboom_camera_node(config):
    return Node(
        package='yahboom_camera',
        executable='yahboom_camera',
        name='front_yahboom_camera',
        parameters=[config],
        remappings=[
            (YAHBOOM_IMAGE_TOPIC, '/r2/front_camera/image_raw'),
            (YAHBOOM_DEBUG_TOPIC, '/r2/front_camera/image_raw/debug'),
            (YAHBOOM_CAMERA_INFO_TOPIC, '/r2/front_camera/camera_info'),
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
    yahboom_camera_pkg = get_package_share_directory('yahboom_camera')
    mipi_camera_pkg = get_package_share_directory('mipi_camera')
    detect_pkg = get_package_share_directory('robot_r2_detect')
    roi_pkg = get_package_share_directory('robot_r2_kfs_roi')
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
    tip_mipi_camera = tip_mipi_camera_node(mipi_camera_config)

    front_camera_config = os.path.join(
        yahboom_camera_pkg,
        'config',
        'yahboom_camera_hd.yaml',
    )
    front_yahboom_camera = front_yahboom_camera_node(front_camera_config)

    kfs_detect_config = os.path.join(
        detect_pkg,
        'config',
        'kfs_detect.yaml',
    )
    fused_kfs_detect = fused_kfs_detect_node(kfs_detect_config)

    kfs_roi_config = os.path.join(
        roi_pkg,
        'config',
        'kfs_roi.yaml',
    )
    kfs_roi = Node(
        package='robot_r2_kfs_roi',
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

    # The tip camera and target detector run together on real2. The detector
    # publishes AlignmentDetection over DDS to real1's tip_alignment controller.
    target_detector_config = os.path.join(
        target_alignment_pkg,
        'config',
        'yolo_target_detector.yaml',
    )
    yolo_target_detector = Node(
        package='robot_r2_target_alignment',
        executable='yolo_target_detector',
        namespace='r2/target_alignment',
        name='yolo_target_detector',
        parameters=[
            target_detector_config,
            {'input_video_topic': '/r2/tip_camera/image_raw'},
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
            default_value='/r2/front_camera/image_raw',
            description='Image topic used by the single KFS ROI node',
        ),
        front_yahboom_camera,
        tip_mipi_camera,
        # fused_kfs_detect,
        kfs_roi,
        yolo_target_detector,
    ])
