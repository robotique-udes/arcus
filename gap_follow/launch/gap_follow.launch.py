import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld: LaunchDescription = LaunchDescription()

    gap_follow_node = Node(package="gap_follow",
                    namespace="arcus",
                    executable="gap_follow",
                    name="gap_follow")

    ld.add_action(gap_follow_node)

    return LaunchDescription([gap_follow_node])