from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description() -> LaunchDescription:
    package_name = "costmap_maker"
    default_params = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "local_costmap.yaml",
    )

    costmap_node = Node(
        package=package_name,
        executable="costmap_node",
        name="costmap_maker",
        output="screen",
        parameters=[default_params],
    )

    return LaunchDescription([
        costmap_node,
    ])