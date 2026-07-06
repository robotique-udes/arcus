#!/usr/bin/env python3
import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from rcl_interfaces.srv import GetParameters, ListParameters
from rcl_interfaces.msg import ParameterType

class ParamSaverNode(Node):
    def __init__(self):
        super().__init__('param_saver_node')
        
        self.cb_group = ReentrantCallbackGroup()
        
        self.yaml_mappings = {
            '/arcus/gap_follow': '/sim_ws/src/gap_follow/config/gap_follow.yaml',
            '/arcus/pure_pursuit': '/sim_ws/src/pure_pursuit/config/pure_pursuit_params.yaml'
        }
        
        self.param_clients = {}
        for node_name in self.yaml_mappings.keys():
            self.param_clients[node_name] = {
                'list': self.create_client(ListParameters, f'{node_name}/list_parameters', callback_group=self.cb_group),
                'get': self.create_client(GetParameters, f'{node_name}/get_parameters', callback_group=self.cb_group)
            }
        
        self.srv = self.create_service(
            Trigger, 
            '/arcus/save_parameters', 
            self.save_parameters_callback,
            callback_group=self.cb_group
        )
        self.get_logger().info("Global Parameter Saver Service is online and ready.")

    async def save_parameters_callback(self, request, response):
        self.get_logger().info("Global save triggered. Processing all tracked nodes...")
        saved_nodes = []
        failed_nodes = []

        for node_name, yaml_path in self.yaml_mappings.items():
            list_client = self.param_clients[node_name]['list']
            get_client = self.param_clients[node_name]['get']

            if not list_client.wait_for_service(timeout_sec=1.0) or not get_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(f"Skipping {node_name}: Node parameter services are unavailable.")
                continue

            req_list = ListParameters.Request()
            try:
                res_list = await list_client.call_async(req_list)
                param_names = [p for p in res_list.result.names if not p.startswith('qos_overrides') and p != 'use_sim_time']
            except Exception as e:
                self.get_logger().error(f"Failed to retrieve parameter list for {node_name}: {e}")
                failed_nodes.append(node_name)
                continue

            if not param_names:
                self.get_logger().warn(f"Skipping {node_name}: No user parameters found to save.")
                continue

            req_get = GetParameters.Request()
            req_get.names = param_names
            try:
                res_get = await get_client.call_async(req_get)
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
                self.get_logger().error(f"Failed to fetch parameter values for {node_name}: {e}")
                failed_nodes.append(node_name)
                continue
            
            yaml_data = self.format_to_ros_yaml(node_name, current_params)

            try:
                os.makedirs(os.path.dirname(yaml_path), exist_ok=True)
                with open(yaml_path, 'w') as f:
                    yaml.dump(yaml_data, f, default_flow_style=False)
                self.get_logger().info(f"Successfully saved {node_name} -> {yaml_path}")
                saved_nodes.append(node_name)
            except Exception as e:
                self.get_logger().error(f"Failed to write configuration file for {node_name}: {str(e)}")
                failed_nodes.append(node_name)

        if failed_nodes:
            response.success = False
            response.message = f"Saved: {saved_nodes}. Failed: {failed_nodes}."
        else:
            response.success = True
            response.message = f"Successfully synced and wrote configurations for: {saved_nodes}"
            
        return response

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