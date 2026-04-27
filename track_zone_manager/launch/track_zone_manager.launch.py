import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("track_zone_manager"),
        "config",
        "track_zone_manager.yaml",
    )

    node = Node(
        package="track_zone_manager",
        executable="track_zone_manager_node",
        namespace="arcus",
        name="track_zone_manager",
        parameters=[params],
        output="screen",
    )

    return LaunchDescription([node])
