import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('pure_pursuit'),
        'config',
        'pure_pursuit_params.yaml'
    )

    pure_pursuit_node = Node(
        package='pure_pursuit',
        executable='pure_pursuit',
        namespace='arcus',
        name='pure_pursuit',
        parameters=[params],
        output='screen'
    )

    return LaunchDescription([pure_pursuit_node])