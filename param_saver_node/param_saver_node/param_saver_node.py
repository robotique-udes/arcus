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
    """
    ParamSaverNode: A centralized ROS 2 node for saving, loading, and switching parameter profiles across multiple nodes.

    Use Cases:
    ----------
    1. Parameter Persistence:
    Retrieves current runtime parameter values from target nodes via ROS 2 parameter services
    and saves them into structured YAML configuration files.

    2. Dynamic Profile Management:
    Switch between runtime parameter profiles (e.g., 'default', 'racing', 'testing').
    If a requested profile directory exists, parameters are loaded into target nodes at runtime.
    If it does not exist, a new profile directory is created using current parameter states as a baseline.

    3. Profile Discovery:
    Scans the configuration directory and exports a list of available profile names as a node parameter
    for external discovery by GUIs or orchestrators.

    CLI Usage Examples:
    -------------------
    1. Save Current Parameters:
    Trigger a parameter dump to save runtime values across all managed nodes to the active profile:
    
    $ ros2 service call /arcus/save_parameters std_srvs/srv/Trigger "{}"

    2. Switch Active Profile:
    Set the active profile name to automatically switch parameter sets across target nodes:
    
    $ ros2 param set /arcus/param_saver_node config_name "racing"

    3. Query Active Profile:
    Retrieve the currently active configuration profile name:
   
    $ ros2 param get /arcus/param_saver_node config_name

    4. Query Available Profiles:
    Retrieve the list of detected configuration profile directories:
    
    $ ros2 param get /arcus/param_saver_node available_profiles
    """
    def __init__(self):
        super().__init__('param_saver_node')

        # ==========================================
        # FILE PATHS
        # ==========================================
        # Adjust base_path and workspace_defaults based on the location of the config files
        # This varies whether you are in simulation or on the car
        self.base_path = '/sim_ws/src/arcus'

        self.node_config_path = os.path.join(self.base_path, 'param_saver_node/config/param_saver_node.yaml')

        self.workspace_defaults = {
            '/arcus/gap_follow': os.path.join(self.base_path, 'gap_follow/config/gap_follow.yaml'),
            '/arcus/pure_pursuit': os.path.join(self.base_path, 'pure_pursuit/config/pure_pursuit_params.yaml'),
            '/master_node': os.path.join(self.base_path, 'arcus_master/config/master_node.yaml')
        }

        self.cb_group = ReentrantCallbackGroup()
        
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

        initial_profiles = self.get_available_profile_names()
        self.declare_parameter('available_profiles', initial_profiles)

        profiles_str = ", ".join([f"'{p}'" for p in initial_profiles])
        count = len(initial_profiles)
        self.get_logger().info(
            f"Global Parameter Saver Service is online and ready with {count} "
            f"profile{'s' if count != 1 else ''}: {profiles_str}."
        )

    def get_profile_dir(self, profile):
        return os.path.join(self.profiles_root, profile)

    def save_node_state(self, profile_name):
        """
        Saves the current config_name to the param_saver_node's own YAML file 
        so it persists across reboots.
        """
        yaml_data = self.format_to_ros_yaml(self.get_name(), {'config_name': profile_name})
        try:
            os.makedirs(os.path.dirname(self.node_config_path), exist_ok=True)
            with open(self.node_config_path, 'w') as f:
                yaml.dump(yaml_data, f, default_flow_style=False)
            self.get_logger().info(f"Node state updated: saved '{profile_name}' to {self.node_config_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to write node configuration file {self.node_config_path}: {e}")

    def on_profile_switch(self, params):
        """
        Parameter event callback triggered when the 'config_name' parameter changes.

        Manages profile lifecycle and runtime transitions according to three scenarios:
        
        1. Redundant Request:
           If the requested profile matches self.current_config, the switch is ignored
           to prevent unnecessary I/O and service calls.
        
        2. Existing Profile Switch:
           If the profile directory exists and contains configuration files, parameters 
           are parsed from YAML files and pushed to target nodes via SetParameters service calls.
        
        3. New Profile Creation:
           If the profile directory does not exist, a new profile is created using the 
           active state of target parameters as a baseline configuration dump.

        :param params: List of Parameter objects updated during the parameter event.
        :type params: list[rclpy.parameter.Parameter]
        :returns: Result object indicating successful handling of parameter update.
        :rtype: rcl_interfaces.msg.SetParametersResult
        """
        for param in params:
            if param.name == 'config_name':
                new_profile = str(param.value)

                if new_profile == self.current_config:
                    self.get_logger().info(f"Profile '{new_profile}' is already active. Skipping redundant reload.")
                    return SetParametersResult(successful=True)

                profile_dir = self.get_profile_dir(new_profile)
                self.current_config = new_profile

                self.save_node_state(new_profile)
                
                if new_profile == 'default':
                    self.get_logger().info("Loading default workspace configuration...")
                    for node, yaml_path in self.workspace_defaults.items():
                        if os.path.exists(yaml_path):
                            self.execute_load(node_name=node, yaml_path=yaml_path)
                elif os.path.exists(profile_dir) and os.listdir(profile_dir):
                    self.get_logger().info(f"Loading existing profile: {new_profile}")
                    for node in self.workspace_defaults.keys():
                        yaml_file = os.path.join(profile_dir, f"{node.split('/')[-1]}.yaml")
                        if os.path.exists(yaml_file):
                            self.execute_load(node_name=node, yaml_path=yaml_file)
                else:
                    self.get_logger().info(f"Creating new profile '{new_profile}', using active state as baseline...")
                    self.execute_dump(profile=new_profile)
                    
                return SetParametersResult(successful=True)
        return SetParametersResult(successful=True)

    def save_callback(self, request, response):
        success = self.execute_dump(profile=self.current_config)
        response.success = success
        response.message = f"Profile '{self.current_config}' synced across configuration matrix."
        return response

    def execute_dump(self, profile):
        """
        Retrieves runtime parameters from target nodes and saves them to disk.

        Queries target nodes via ListParameters and GetParameters services, formats 
        the retrieved values into ROS 2 standard YAML structures, and writes them 
        to the specified profile directory.

        :param profile: Target profile directory name to write configuration YAMLs to.
        :type profile: str
        :returns: True if all target node parameters were dumped successfully, False otherwise.
        :rtype: bool
        """
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
                    elif p_val.type == ParameterType.PARAMETER_BOOL_ARRAY:
                        current_params[name] = list(p_val.bool_array_value)
                    elif p_val.type == ParameterType.PARAMETER_INTEGER_ARRAY:
                        current_params[name] = list(p_val.integer_array_value)
                    elif p_val.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
                        current_params[name] = list(p_val.double_array_value)
                    elif p_val.type == ParameterType.PARAMETER_STRING_ARRAY:
                        current_params[name] = list(p_val.string_array_value)
            except Exception as e:
                self.get_logger().error(f"Failed fetching parameters for {node_name}: {e}")
                overall_success = False
                continue
            
            yaml_data = self.format_to_ros_yaml(node_name, current_params)

            if profile == 'default':
                paths_to_write = [self.workspace_defaults[node_name]]
            else:
                node_filename = f"{node_name.split('/')[-1]}.yaml"
                paths_to_write = [os.path.join(profile_dir, node_filename)]

            for yaml_path in paths_to_write:
                try:
                    os.makedirs(os.path.dirname(yaml_path), exist_ok=True)
                    with open(yaml_path, 'w') as f:
                        yaml.dump(yaml_data, f, default_flow_style=False)
                    self.get_logger().info(f"Successfully saved {node_name} -> {yaml_path}")
                except Exception as e:
                    self.get_logger().error(f"Failed to write configuration file {yaml_path}: {e}")
                    overall_success = False

        self.set_parameters([
            rclpy.parameter.Parameter(
                'available_profiles',
                rclpy.parameter.Parameter.Type.STRING_ARRAY,
                self.get_available_profile_names()
            )
        ])

        return overall_success

    def execute_load(self, node_name, yaml_path):
        """
        Loads parameter values from a YAML file and applies them to a target node.

        Parses ROS 2 parameter YAML structure and sends a SetParameters service 
        request to update the active parameter values on the target node.

        :param node_name: Fully qualified name of the target node (e.g., '/arcus/gap_follow').
        :type node_name: str
        :param yaml_path: Absolute file path to the source YAML configuration file.
        :type yaml_path: str
        :returns: True if parameters were successfully pushed to the node, False otherwise.
        :rtype: bool
        """
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
                elif isinstance(val, list):
                    if all(isinstance(x, bool) for x in val):
                        p.value.type = ParameterType.PARAMETER_BOOL_ARRAY
                        p.value.bool_array_value = val
                    elif all(isinstance(x, int) for x in val):
                        p.value.type = ParameterType.PARAMETER_INTEGER_ARRAY
                        p.value.integer_array_value = val
                    elif all(isinstance(x, float) or isinstance(x, int) for x in val):
                        p.value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
                        p.value.double_array_value = [float(x) for x in val]
                    elif all(isinstance(x, str) for x in val):
                        p.value.type = ParameterType.PARAMETER_STRING_ARRAY
                        p.value.string_array_value = val
                else:
                    continue
                req_set.parameters.append(p)

            res = set_client.call(req_set)
            self.get_logger().info(f"Loaded profile updates into {node_name}")
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

    def get_available_profile_names(self):
        if not os.path.exists(self.profiles_root):
            return ['default']
        try:
            profiles = [d for d in os.listdir(self.profiles_root) 
                        if os.path.isdir(os.path.join(self.profiles_root, d))]
            return profiles if profiles else ['default']
        except Exception:
            return ['default']

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