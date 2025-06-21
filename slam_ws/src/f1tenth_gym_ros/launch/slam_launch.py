# slam_launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the launch directory
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value='/home/charles/sim_ws/src/f1tenth_gym_ros/config/mapper_params_online_async.yaml',
        description='Path to slam_toolbox parameters file'
    )

    # SLAM Toolbox node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/map', '/slam_map'),
            ('/scan', '/scan'), 
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        slam_toolbox_node
    ])