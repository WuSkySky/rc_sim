"""Launch kfs_detect."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("model_path", default_value="best.pt"),
            DeclareLaunchArgument(
                "color_topic", default_value="/r2/front_camera/image_raw"
            ),
            DeclareLaunchArgument("conf", default_value="0.75"),

            Node(
                package="robot_r2_detect",
                executable="kfs_detect",
                name="kfs_detect",
                output="screen",
                parameters=[
                    {
                        "model_path": LaunchConfiguration("model_path"),
                        "color_topic": LaunchConfiguration("color_topic"),
                        "conf": LaunchConfiguration("conf"),
                    }
                ],
            ),
        ]
    )
