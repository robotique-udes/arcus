import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.DeclareLaunchArgument(
            'deadzone',
            default_value='0.1',
            description='The joystick deadzone'
        ),
        launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
            name='joy',),
        launch_ros.actions.Node(
            package='drive_controller',
            executable='drive_controller',
            name='drive_controller',),
        
  ])