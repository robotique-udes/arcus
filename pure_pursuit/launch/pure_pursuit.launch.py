import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld: LaunchDescription = LaunchDescription()

    pure_pursuit_node = Node(package="pure_pursuit",
                    namespace="arcus",
                    executable="pure_pursuit",
                    name="pure_pursuit")

    ld.add_action(pure_pursuit_node)

    return LaunchDescription([pure_pursuit_node])