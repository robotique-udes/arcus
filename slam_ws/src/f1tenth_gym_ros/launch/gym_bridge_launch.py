# MIT License
# (copyright header...)

import os
import yaml
import datetime
import glob

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def get_latest_map_yaml(map_dir: str):
    """Return the path to the most recently modified .yaml file in map_dir, or None if none exist."""
    yaml_files = glob.glob(os.path.join(map_dir, "*.yaml"))
    if not yaml_files:
        return None
    latest = max(yaml_files, key=os.path.getmtime)
    return latest

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('f1tenth_gym_ros'),
        'config',
        'sim.yaml'
    )
    config_dict = yaml.safe_load(open(config, 'r'))
    has_opp = config_dict['bridge']['ros__parameters']['num_agent'] > 1
    simulated_localization = config_dict['bridge']['ros__parameters']['use_sim_localization']
    run_slam = config_dict['bridge']['ros__parameters']['run_slam']
    map_path = config_dict['bridge']['ros__parameters']['map_path'] + '.yaml'
    if simulated_localization and not run_slam:
        maps_dir = config_dict['bridge']['ros__parameters']['slam_maps_dir']
        latest_map_yaml = get_latest_map_yaml(maps_dir)
        map_path = latest_map_yaml if latest_map_yaml is not None else maps_dir + ".yaml"
        print(f"[INFO] Loading map from {latest_map_yaml}")

    # === Nodes ===
    bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        parameters=[config]
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'gym_bridge.rviz')]
    )
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[{'yaml_filename': map_path},
                    {'topic': 'map'},
                    {'frame_id': 'map'},
                    {'output': 'screen'},
                    {'use_sim_time': True}]
    )
    if simulated_localization and not run_slam:
        nav_lifecycle_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl']}]
        )
    else:
        nav_lifecycle_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server']}]
        )
    ego_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ego_robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'ego_racecar.xacro')])}],
        remappings=[('/robot_description', 'ego_robot_description')]
    )
    opp_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='opp_robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'opp_racecar.xacro')])}],
        remappings=[('/robot_description', 'opp_robot_description')]
    )
    imu_node = Node(
        package='fake_imu',
        executable='fake_imu_publisher',
        name='fake_imu',
        parameters=[config]
    )
    ekf_node = Node(
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
    )
    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[os.path.join(
            get_package_share_directory('f1tenth_gym_ros'),
            'config',
            'amcl.yaml'
        )],
        remappings=[
            ("/odom", "/odometry/filtered")
        ]
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory('f1tenth_gym_ros'),
            'config',
            'mapper_params_online_async.yaml')
        ],
        remappings=[
            ('/map', '/slam_map'),
            ('/scan', '/scan'),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
        ]
    )
    # === Finalize ===
    ld.add_action(rviz_node)
    ld.add_action(bridge_node)
    ld.add_action(nav_lifecycle_node)
    ld.add_action(map_server_node)
    ld.add_action(ego_robot_publisher)
    ld.add_action(imu_node)
    if has_opp:
        ld.add_action(opp_robot_publisher)
    if simulated_localization and not run_slam:
        ld.add_action(ekf_node)
        ld.add_action(amcl_node)
    elif run_slam:
        ld.add_action(slam_toolbox_node)

    return ld
# Note: If both simulated_localization and run_slam are true, only SLAM will run.