from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to your map file (YAML that points to a PGM/PNG map image)
    map_file = os.path.join(
        get_package_share_directory('f1tenth_gym_ros'),  # replace with your package name
        'maps',
        'levine.yaml'
    )

    # Path to your AMCL config
    amcl_config = os.path.join(
        get_package_share_directory('f1tenth_gym_ros'),
        'config',
        'amcl.yaml'
    )

    return LaunchDescription([
        # Map server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': '/home/user/map.yaml'}]
        ),

        # AMCL
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_config]
        ),
    ])
