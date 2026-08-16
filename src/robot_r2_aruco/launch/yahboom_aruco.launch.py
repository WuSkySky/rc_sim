"""Launch yahboom_camera + aruco_detect + debug_saver for Yahboom camera ArUco testing."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    aruco_share = get_package_share_directory('robot_r2_aruco')
    yahboom_share = get_package_share_directory('yahboom_camera')
    interfaces_share = get_package_share_directory('robot_r2_interfaces')
    fastdds_profile = os.path.join(
        interfaces_share, 'config', 'fastdds_camera.xml',
    )
    aruco_config = os.path.join(
        aruco_share, 'config', 'aruco_detect.yaml',
    )
    pose_bridge_config = os.path.join(
        aruco_share, 'config', 'aruco_pose_bridge.yaml',
    )
    camera_tf_config = os.path.join(
        aruco_share, 'config', 'aruco_camera_tf.yaml',
    )
    yahboom_config = os.path.join(
        yahboom_share, 'config', 'yahboom_camera.yaml',
    )

    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        SetEnvironmentVariable('RMW_FASTRTPS_USE_QOS_FROM_XML', '1'),
        SetEnvironmentVariable('FASTDDS_DEFAULT_PROFILES_FILE', fastdds_profile),
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', fastdds_profile),

        Node(
            package='yahboom_camera',
            executable='yahboom_camera',
            name='yahboom_camera',
            output='screen',
            parameters=[yahboom_config],
            remappings=[
                ('/r2/yahboom_camera/image_raw', '/image_raw'),
                ('/r2/yahboom_camera/image_raw/debug', '/image_raw/debug'),
                ('/r2/yahboom_camera/camera_info', '/camera_info'),
            ],
        ),
        Node(
            package='robot_r2_aruco',
            executable='aruco_detect',
            name='aruco_detect',
            output='screen',
            parameters=[aruco_config],
            remappings=[
                ('image_raw', '/image_raw'),
                ('camera_info', '/camera_info'),
                ('detections', '/aruco/detections'),
                ('debug', '/aruco/debug'),
            ],
        ),
        Node(
            package='robot_r2_aruco',
            executable='debug_saver',
            name='debug_saver',
            output='screen',
        ),
        # Static base_link -> camera transform, published from YAML parameters
        # (aruco_camera_tf.yaml). The camera frame is read from camera_info so
        # it always matches the camera node. Orientation must be roughly right
        # (a reversed camera cannot be recovered by teach-in); translation is
        # approximate.
        Node(
            package='robot_r2_aruco',
            executable='aruco_camera_tf',
            name='aruco_camera_tf',
            output='screen',
            parameters=[camera_tf_config],
            remappings=[
                ('camera_info', '/camera_info'),
            ],
        ),
        # Publish base_link pose in the marker frame for downstream docking.
        Node(
            package='robot_r2_aruco',
            executable='aruco_pose_bridge',
            name='aruco_pose_bridge',
            output='screen',
            parameters=[pose_bridge_config],
        ),
    ])
