"""Launch kfs_detect."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    interfaces_share = get_package_share_directory("robot_r2_interfaces")
    config_directory = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "config")
    )
    fastdds_profile = os.path.join(
        interfaces_share,
        "config",
        "fastdds_camera.xml",
    )
    detect_config = os.path.join(
        config_directory,
        "kfs_detect.yaml",
    )
    roi_config = os.path.join(
        config_directory,
        "kfs_roi.yaml",
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                "RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"
            ),
            SetEnvironmentVariable(
                "RMW_FASTRTPS_USE_QOS_FROM_XML", "1"
            ),
            SetEnvironmentVariable(
                "FASTDDS_DEFAULT_PROFILES_FILE", fastdds_profile
            ),
            SetEnvironmentVariable(
                "FASTRTPS_DEFAULT_PROFILES_FILE", fastdds_profile
            ),
            DeclareLaunchArgument(
                "color_topic", default_value="/r2/left_camera/image_raw"
            ),
            DeclareLaunchArgument("conf", default_value="0.5"),
            DeclareLaunchArgument(
                "simulation_state_detection", default_value="false"
            ),

            Node(
                package="robot_r2_detect",
                executable="kfs_roi",
                name="kfs_roi",
                output="screen",
                parameters=[
                    roi_config,
                    {
                        "color_topic": LaunchConfiguration(
                            "color_topic"
                        ),
                    },
                ],
            ),
            Node(
                package="robot_r2_detect",
                executable="kfs_detect",
                name="kfs_detect",
                output="screen",
                parameters=[
                    detect_config,
                    {
                        "conf": ParameterValue(
                            LaunchConfiguration("conf"),
                            value_type=float,
                        ),
                        "simulation_state_detection": ParameterValue(
                            LaunchConfiguration(
                                "simulation_state_detection"
                            ),
                            value_type=bool,
                        ),
                    }
                ],
            ),
        ]
    )
