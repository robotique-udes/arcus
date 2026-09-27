import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(get_package_share_directory("param_saver_node"), "config", "param_saver_node.yaml")

    param_saver_node = Node(
        package="param_saver_node",
        namespace="arcus",
        executable="param_saver_node",
        name="param_saver_node",
        parameters=[config_path],
    )

    return LaunchDescription([param_saver_node])