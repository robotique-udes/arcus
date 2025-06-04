import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():

    joy_node = launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
            name='joy',)
    
    controller_node = launch_ros.actions.Node(
            package='drive_controller',
            executable='controller_node',
            name='controller_node',)
    
    vesc_dir = get_package_share_directory('vesc_driver')
    vesc_launch = launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                    vesc_dir + '/launch/vesc_driver_node.launch.py'))
    

    return launch.LaunchDescription([
        joy_node,
        controller_node,
        vesc_launch,
  ])