from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    # 1. Declare the 'sim' argument as a boolean string (default to false)
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='Use simulation configuration if true (true/false)'
    )

    # 2. Safely evaluate the string by forcing quotes around the substitution
    config_file_name = PythonExpression([
        "'localize_sim.yaml' if '", LaunchConfiguration('sim'), "'.lower() in ['true', '1'] else 'localize.yaml'"
    ])

    localize_config_path = PathJoinSubstitution([
        FindPackageShare('particle_filter'),
        'config',
        config_file_name
    ])

    # 3. Read the map name safely from the base configuration file
    base_config_path = os.path.join(
        get_package_share_directory('particle_filter'),
        'config',
        'localize.yaml'
    )
    with open(localize_config_path, 'r') as f:
        localize_config_dict = yaml.safe_load(f)
    
    map_name = localize_config_dict['map_server']['ros__parameters']['map']

    # 4. Declare the argument using the delayed ROS substitution path
    localize_la = DeclareLaunchArgument(
        'localize_config',
        default_value=localize_config_path,
        description='Localization configs')

    ld = LaunchDescription([
        sim_arg,
        localize_la
    ])

    # 5. Safely handle use_sim_time evaluation using the same quoted trick
    use_sim_time_expr = PythonExpression([
        "True if '", LaunchConfiguration('sim'), "'.lower() in ['true', '1'] else False"
    ])

    # nodes
    pf_node = Node(
        package='particle_filter',
        executable='particle_filter_node',
        name='particle_filter',
        parameters=[LaunchConfiguration('localize_config')]
    )
    
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': os.path.join(get_package_share_directory('particle_filter'), 'maps', map_name + '.yaml')},
                    {'topic_name': 'map'},
                    {'frame_id': 'map'},
                    {'output': 'screen'},
                    {'use_sim_time': use_sim_time_expr}]
    )
    
    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time_expr},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )

    # finalize
    ld.add_action(nav_lifecycle_node)
    ld.add_action(map_server_node)
    ld.add_action(pf_node)

    return ld
