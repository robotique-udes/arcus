import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("arcus_master"),
        "config",
        "master_node.yaml",
    )

    node = Node(
        package="arcus_master",
        executable="master_node",
        namespace="arcus",
        name="master_node",
        parameters=[params],
        output="screen",
    )

    return LaunchDescription([node])
