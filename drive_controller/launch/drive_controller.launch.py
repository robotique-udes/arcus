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
    

    return launch.LaunchDescription([
        joy_node,
        controller_node,
  ])