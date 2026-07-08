#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from rcl_interfaces.msg import ParameterType, SetParametersResult
from ros2param.api import dump_parameters, load_parameters

class ParamSaverNode(Node):
    def __init__(self):
        super().__init__('param_saver_node')
        
        self.workspace_defaults = {
            '/arcus/gap_follow': '/sim_ws/src/gap_follow/config/gap_follow.yaml',
            '/arcus/pure_pursuit': '/sim_ws/src/pure_pursuit/config/pure_pursuit_params.yaml'
        }
        
        self.profiles_root = '/sim_ws/src/config_profiles'
        
        self.current_config = 'default'
        self.declare_parameter('config_name', 'default')
        self.add_on_set_parameters_callback(self.on_profile_switch)
        
        self.save_srv = self.create_service(Trigger, '/arcus/save_parameters', self.save_callback)
        self.get_logger().info("Global Parameter Saver Service is online and ready.")

    def get_profile_dir(self, profile):
        return os.path.join(self.profiles_root, profile)

    def on_profile_switch(self, params):
        for param in params:
            if param.name == 'config_name' and param.type == ParameterType.PARAMETER_STRING:
                new_profile = str(param.value)
                profile_dir = self.get_profile_dir(new_profile)
                
                self.current_config = new_profile
                
                # Check if this profile folder exists and has configs
                if os.path.exists(profile_dir) and os.listdir(profile_dir):
                    self.get_logger().info(f"Loading Profile: {new_profile}")
                    for node in self.workspace_defaults.keys():
                        yaml_file = os.path.join(profile_dir, f"{node.split('/')[-1]}.yaml")
                        if os.path.exists(yaml_file):
                            load_parameters(node=self, node_name=node, parameter_file=yaml_file)
                else:
                    self.get_logger().info(f"Profile '{new_profile}' target empty. Snapping active state as baseline...")
                    self.execute_dump(profile=new_profile)
                    
                return SetParametersResult(successful=True)
        return SetParametersResult(successful=True)

    def save_callback(self, request, response):
        success = self.execute_dump(profile=self.current_config)
        response.success = success
        response.message = f"Profile '{self.current_config}' synced cleanly across configuration matrix."
        return response

    def execute_dump(self, profile):
        profile_dir = self.get_profile_dir(profile)
        os.makedirs(profile_dir, exist_ok=True)
        
        success = True
        for node in self.workspace_defaults.keys():
            node_filename = f"{node.split('/')[-1]}.yaml"
            profile_yaml_path = os.path.join(profile_dir, node_filename)
            
            dump_profile_res = dump_parameters(node=self, node_name=node, parameter_file=profile_yaml_path)

            dump_workspace_res = dump_parameters(node=self, node_name=node, parameter_file=self.workspace_defaults[node])
            
            if not (dump_profile_res and dump_workspace_res):
                success = False
        return success

def main(args=None):
    rclpy.init(args=args)
    node = ParamSaverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()