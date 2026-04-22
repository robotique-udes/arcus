import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(get_package_share_directory("safety_node"), "config", "safety_node.yaml")

    safety_node_node = Node(
        package="safety_node",
        namespace="arcus",
        executable="safety_node",
        name="safety_node",
        parameters=[config_path],
    )

    return LaunchDescription([safety_node_node])