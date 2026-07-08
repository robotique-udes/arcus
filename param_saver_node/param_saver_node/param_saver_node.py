#!/usr/bin/env python3
import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from rcl_interfaces.srv import GetParameters, ListParameters, SetParameters
from rcl_interfaces.msg import ParameterType, SetParametersResult, Parameter, ParameterValue

class ParamSaverNode(Node):
    def __init__(self):
        super().__init__('param_saver_node')
        
        self.cb_group = ReentrantCallbackGroup()

        self.base_path = '/sim_ws/src/arcus'
        
        self.workspace_defaults = {
            '/arcus/gap_follow': os.path.join(self.base_path, 'gap_follow/config/gap_follow.yaml'),
            '/arcus/pure_pursuit': os.path.join(self.base_path, 'pure_pursuit/config/pure_pursuit_params.yaml'),
            '/master_node': os.path.join(self.base_path, 'arcus_master/config/master_node.yaml')
        }
        
        self.profiles_root = os.path.join(self.base_path, 'config_profiles')
        self.current_config = 'default'
        
        self.param_clients = {}
        for node_name in self.workspace_defaults.keys():
            self.param_clients[node_name] = {
                'list': self.create_client(ListParameters, f'{node_name}/list_parameters', callback_group=self.cb_group),
                'get': self.create_client(GetParameters, f'{node_name}/get_parameters', callback_group=self.cb_group),
                'set': self.create_client(SetParameters, f'{node_name}/set_parameters', callback_group=self.cb_group)
            }
        
        self.declare_parameter('config_name', 'default')
        self.add_on_set_parameters_callback(self.on_profile_switch)
        
        self.save_srv = self.create_service(
            Trigger, 
            '/arcus/save_parameters', 
            self.save_callback,
            callback_group=self.cb_group
        )
        self.get_logger().info("Global Parameter Saver Service is online and ready.")

    def get_profile_dir(self, profile):
        return os.path.join(self.profiles_root, profile)

    def on_profile_switch(self, params):
        for param in params:
            if param.name == 'config_name' and param.type == ParameterType.PARAMETER_STRING:
                new_profile = str(param.value)
                profile_dir = self.get_profile_dir(new_profile)
                self.current_config = new_profile
                
                if os.path.exists(profile_dir) and os.listdir(profile_dir):
                    self.get_logger().info(f"Loading Profile: {new_profile}")
                    for node in self.workspace_defaults.keys():
                        yaml_file = os.path.join(profile_dir, f"{node.split('/')[-1]}.yaml")
                        if os.path.exists(yaml_file):
                            self.execute_load(node_name=node, yaml_path=yaml_file)
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
        
        overall_success = True
        
        for node_name in self.workspace_defaults.keys():
            list_client = self.param_clients[node_name]['list']
            get_client = self.param_clients[node_name]['get']

            if not list_client.wait_for_service(timeout_sec=1.0) or not get_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(f"Skipping dump for {node_name}: Services unavailable.")
                continue

            req_list = ListParameters.Request()
            try:
                res_list = list_client.call(req_list)
                param_names = [p for p in res_list.result.names if not p.startswith('qos_overrides') and p != 'use_sim_time']
            except Exception as e:
                self.get_logger().error(f"Failed listing parameters for {node_name}: {e}")
                overall_success = False
                continue

            if not param_names:
                continue

            req_get = GetParameters.Request()
            req_get.names = param_names
            try:
                res_get = get_client.call(req_get)
                current_params = {}
                for name, p_val in zip(param_names, res_get.values):
                    if p_val.type == ParameterType.PARAMETER_BOOL:
                        current_params[name] = p_val.bool_value
                    elif p_val.type == ParameterType.PARAMETER_INTEGER:
                        current_params[name] = p_val.integer_value
                    elif p_val.type == ParameterType.PARAMETER_DOUBLE:
                        current_params[name] = p_val.double_value
                    elif p_val.type == ParameterType.PARAMETER_STRING:
                        current_params[name] = p_val.string_value
            except Exception as e:
                self.get_logger().error(f"Failed fetching parameters for {node_name}: {e}")
                overall_success = False
                continue
            
            yaml_data = self.format_to_ros_yaml(node_name, current_params)
            node_filename = f"{node_name.split('/')[-1]}.yaml"
            paths_to_write = [os.path.join(profile_dir, node_filename), self.workspace_defaults[node_name]]
            
            for yaml_path in paths_to_write:
                try:
                    os.makedirs(os.path.dirname(yaml_path), exist_ok=True)
                    with open(yaml_path, 'w') as f:
                        yaml.dump(yaml_data, f, default_flow_style=False)
                    self.get_logger().info(f"Successfully saved {node_name} -> {yaml_path}")
                except Exception as e:
                    self.get_logger().error(f"Failed to write configuration file {yaml_path}: {e}")
                    overall_success = False

        return overall_success

    def execute_load(self, node_name, yaml_path):
        set_client = self.param_clients[node_name]['set']
        if not set_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(f"Cannot load profile into {node_name}: Set parameter service offline.")
            return False

        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
            
            parts = [p for p in node_name.split('/') if p]
            curr_layer = data
            for part in parts:
                curr_layer = curr_layer.get(part, {})
            params_dict = curr_layer.get('ros__parameters', {})
            
            if not params_dict:
                return False

            req_set = SetParameters.Request()
            for name, val in params_dict.items():
                p = Parameter()
                p.name = name
                p.value = ParameterValue()
                
                if isinstance(val, bool):
                    p.value.type = ParameterType.PARAMETER_BOOL
                    p.value.bool_value = val
                elif isinstance(val, int):
                    p.value.type = ParameterType.PARAMETER_INTEGER
                    p.value.integer_value = val
                elif isinstance(val, float):
                    p.value.type = ParameterType.PARAMETER_DOUBLE
                    p.value.double_value = val
                elif isinstance(val, str):
                    p.value.type = ParameterType.PARAMETER_STRING
                    p.value.string_value = val
                else:
                    continue
                req_set.parameters.append(p)

            # Fire off request synchronously safely under the MultiThreadedExecutor
            res = set_client.call(req_set)
            self.get_logger().info(f"Loaded profile updates cleanly into {node_name}")
            return True

        except Exception as e:
            self.get_logger().error(f"Failed loading profile configurations from {yaml_path}: {e}")
            return False

    def format_to_ros_yaml(self, node_name, params_dict):
        parts = [p for p in node_name.split('/') if p]
        inner_layer = {"ros__parameters": params_dict}
        for part in reversed(parts):
            inner_layer = {part: inner_layer}
        return inner_layer

def main(args=None):
    rclpy.init(args=args)
    node = ParamSaverNode()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()