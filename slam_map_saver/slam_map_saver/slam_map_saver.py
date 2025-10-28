#!/usr/bin/env python3
# Simple ROS 2 node that invokes the Nav2 map_saver_cli tool to write the current map to disk.
# This file launches a subprocess (ros2 run ...) to save the map and logs success or failure.

import rclpy
from rclpy.node import Node
import subprocess
import datetime
import os


class MapSaverCliNode(Node):
    """
    Minimal ROS 2 node wrapper around the nav2 map_saver_cli utility.

    The node does not run continuously — it exists only to provide a ROS 2
    logger and then calls the external map_saver_cli executable to dump the
    current map (/slam_map) to a timestamped file.
    """
    def __init__(self):
        # Initialize the ROS 2 node with a name and log that the node is ready.
        super().__init__('map_saver_cli_node')
        self.get_logger().info("MapSaverCliNode ready!")

    def save_map(self):
        """
        Build a timestamped filename, ensure the target directory exists, and
        call the nav2 map_saver_cli tool to save the map.

        The method:
        - Uses datetime to generate a unique filename.
        - Expands a hardcoded path for the maps directory.
        - Creates the directory if missing.
        - Invokes: ros2 run nav2_map_server map_saver_cli -f <map_file> --ros-args --remap /map:=/slam_map
        - Logs success or failure.
        """
        # Timestamp for unique file names: e.g. slam_map_20251028_123456
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

        # Directory where maps will be written. expanduser allows ~ usage if present.
        maps_dir = os.path.expanduser('/sim_ws/src/arcus/slam_map_saver/slam_maps')
        os.makedirs(maps_dir, exist_ok=True)

        # Base path (no extension) passed to map_saver_cli; it will create .yaml and .pgm
        map_file = os.path.join(maps_dir, f'slam_map_{timestamp}')

        self.get_logger().info(f"Saving map to {map_file}...")
        try:
            # Call the nav2 map_saver_cli via ros2 run. Remap /map to /slam_map so the correct topic is saved.
            subprocess.run(
                ["ros2", "run", "nav2_map_server", "map_saver_cli",
                 "-f", map_file,
                 "--ros-args", "--remap", "/map:=/slam_map"],
                check=True
            )

            # On success, map_saver_cli will have written both .yaml and .pgm files.
            self.get_logger().info(f"✅ Map saved to {map_file}.yaml and {map_file}.pgm")
        except subprocess.CalledProcessError as e:
            # Log an error if the subprocess returns a non-zero exit code.
            self.get_logger().error(f"❌ Failed to save map: {e}")


def main(args=None):
    """
    Entry point when the script is run directly.

    Initializes rclpy, creates the MapSaverCliNode, calls save_map once,
    then cleans up and shuts down the ROS 2 client library.
    """
    rclpy.init(args=args)
    node = MapSaverCliNode()
    node.save_map()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
