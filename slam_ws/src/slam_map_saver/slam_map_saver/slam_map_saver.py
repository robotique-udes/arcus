#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import subprocess
import datetime
import os

class MapSaverCliNode(Node):
    def __init__(self):
        super().__init__('map_saver_cli_node')
        self.get_logger().info("MapSaverCliNode ready!")

    def save_map(self):
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        maps_dir = os.path.expanduser('~/arcus/slam_ws/src/slam_map_saver/slam_maps')
        os.makedirs(maps_dir, exist_ok=True)
        map_file = os.path.join(maps_dir, f'slam_map_{timestamp}')

        self.get_logger().info(f"Saving map to {map_file}...")
        try:
            subprocess.run(
                ["ros2", "run", "nav2_map_server", "map_saver_cli",
                "-f", map_file,
                "--ros-args", "--remap", "/map:=/slam_map"],
                check=True
            )

            self.get_logger().info(f"✅ Map saved to {map_file}.yaml and {map_file}.pgm")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"❌ Failed to save map: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MapSaverCliNode()
    node.save_map()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
