#!/usr/bin/env python3
import os
import yaml
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from rcl_interfaces.srv import GetParameters, ListParameters
from rcl_interfaces.msg import ParameterType

class ParamSaverNode(Node):
    def __init__(self):
        super().__init__('param_saver_node')
        
        # Absolute source repository paths for your YAML files
        self.yaml_mappings = {
            '/arcus/gap_follow': '/sim_ws/src/gap_follow/config/gap_follow.yaml',
            '/arcus/pure_pursuit': '/sim_ws/src/pure_pursuit/config/pure_pursuit_params.yaml'
        }
        
        self.srv = self.create_service(Trigger, '/arcus/save_parameters', self.save_parameters_callback)
        self.get_logger().info("Global Parameter Saver Service is online.")

    def save_parameters_callback(self, request, response):
        self.get_logger().info("Global save triggered. Processing all tracked nodes...")
        saved_nodes = []
        failed_nodes = []

        for node_name, yaml_path in self.yaml_mappings.items():
            # 1. Dynamically list all current parameters for the given node namespace
            param_names = self.get_node_param_names(node_name)
            if not param_names:
                self.get_logger().warn(f"Skipping {node_name}: Node not running or no parameters found.")
                continue

            # 2. Query the live parameter values from the node
            current_params = self.fetch_param_values(node_name, param_names)
            if not current_params:
                failed_nodes.append(node_name)
                continue
            
            # 3. Format back into the exact standard ROS 2 namespace layout
            yaml_data = self.format_to_ros_yaml(node_name, current_params)

            # 4. Safely write back to the workspace configuration source file
            try:
                os.makedirs(os.path.dirname(yaml_path), exist_ok=True)
                with open(yaml_path, 'w') as f:
                    yaml.dump(yaml_data, f, default_flow_style=False)
                self.get_logger().info(f"Successfully saved {node_name} -> {yaml_path}")
                saved_nodes.append(node_name)
            except Exception as e:
                self.get_logger().error(f"Failed to write configuration for {node_name}: {str(e)}")
                failed_nodes.append(node_name)

        if failed_nodes:
            response.success = False
            response.message = f"Saved: {saved_nodes}. Failed: {failed_nodes}."
        else:
            response.success = True
            response.message = f"Successfully synced and wrote configurations for: {saved_nodes}"
            
        return response

    def get_node_param_names(self, node_name):
        # Uses the ROS 2 parameter service to dynamically find what's running
        client = self.create_client(ListParameters, f'{node_name}/list_parameters')
        if not client.wait_for_service(timeout_sec=1.0):
            return []
        
        req = ListParameters.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        
        if future.result() is not None:
            # Filter out read-only/system parameters like 'qos_overrides'
            return [p for p in future.result().result.names if not p.startswith('qos_overrides') and p != 'use_sim_time']
        return []

    def fetch_param_values(self, node_name, param_names):
        client = self.create_client(GetParameters, f'{node_name}/get_parameters')
        if not client.wait_for_service(timeout_sec=1.0):
            return {}

        req = GetParameters.Request()
        req.names = param_names
        
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        
        params_dict = {}
        if future.result() is not None:
            for name, p_val in zip(param_names, future.result().values):
                if p_val.type == ParameterType.PARAMETER_BOOL:
                    params_dict[name] = p_val.bool_value
                elif p_val.type == ParameterType.PARAMETER_INTEGER:
                    params_dict[name] = p_val.integer_value
                elif p_val.type == ParameterType.PARAMETER_DOUBLE:
                    params_dict[name] = p_val.double_value
                elif p_val.type == ParameterType.PARAMETER_STRING:
                    params_dict[name] = p_val.string_value
        return params_dict

    def format_to_ros_yaml(self, node_name, params_dict):
        # Breaks down the absolute /ns/node paths to nested YAML dictionaries
        parts = [p for p in node_name.split('/') if p]
        inner_layer = {"ros__parameters": params_dict}
        for part in reversed(parts):
            inner_layer = {part: inner_layer}
        return inner_layer

def main(args=None):
    rclpy.init(args=args)
    node = ParamSaverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()