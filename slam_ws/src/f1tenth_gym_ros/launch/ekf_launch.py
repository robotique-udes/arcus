from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(
            get_package_share_directory('f1tenth_gym_ros'),
            'config',
            'ekf.yaml'
            )],
            remappings=[
                ('/odometry/filtered', '/odometry/filtered')
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            parameters=[{'use_sim_time': True}]
        )
    ])