import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld: LaunchDescription = LaunchDescription()

    safety_node_node = Node(package="safety_node",
                    namespace="arcus",
                    executable="safety_node",
                    name="safety_node")

    ld.add_action(safety_node_node)

    return LaunchDescription([safety_node_node])