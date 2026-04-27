import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(get_package_share_directory("gap_follow"), "config", "gap_follow.yaml")

    gap_follow_node = Node(
        package="gap_follow",
        namespace="arcus",
        executable="gap_follow",
        name="gap_follow",
        parameters=[config_path],
    )

    return LaunchDescription([gap_follow_node])