import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    param_saver_node = Node(
        package="param_saver_node",
        namespace="arcus",
        executable="param_saver_node",
        name="param_saver_node",
    )

    return LaunchDescription([param_saver_node])