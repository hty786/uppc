import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory("fast_livo"), "config")
    rviz_config = os.path.join(get_package_share_directory("fast_livo"), "rviz_cfg", "fast_livo2.rviz")

    mid360_config = os.path.join(config_dir, "mid360.yaml")
    camera_config = os.path.join(config_dir, "camera_mid360.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="False"),

        Node(
            package="fast_livo",
            executable="fastlivo_mapping",
            name="laserMapping",
            parameters=[mid360_config, camera_config],
            output="screen",
            prefix=["stdbuf -oL -eL"],
        ),

        Node(
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])
