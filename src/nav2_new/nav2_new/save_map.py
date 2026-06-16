"""
save_map – Convenience wrapper to save the SLAM map to disk.

Usage:
  ros2 run nav2_new save_map
  ros2 run nav2_new save_map --ros-args -p name:=lab_map
  ros2 run nav2_new save_map --ros-args -p output_dir:=/my/path -p name:=arena
"""
import os
import subprocess
import datetime

import rclpy
from rclpy.node import Node


class SaveMap(Node):
    def __init__(self):
        super().__init__('save_map')

        self.declare_parameter(
            'output_dir',
            os.path.join(os.path.expanduser('~'), 'roboto_maps'))
        self.declare_parameter('name', '')   # empty → auto timestamp

        output_dir = self.get_parameter('output_dir').value
        name = self.get_parameter('name').value or \
            f'map_{datetime.datetime.now().strftime("%Y%m%d_%H%M%S")}'

        os.makedirs(output_dir, exist_ok=True)
        map_path = os.path.join(output_dir, name)

        self.get_logger().info(f'Saving map to: {map_path}')

        try:
            result = subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                 '-f', map_path,
                 '--ros-args', '-p', 'save_map_timeout:=10.0'],
                capture_output=True, text=True, timeout=30
            )
            if result.returncode == 0:
                self.get_logger().info('─' * 60)
                self.get_logger().info(f'✓ Map saved:')
                self.get_logger().info(f'  {map_path}.pgm')
                self.get_logger().info(f'  {map_path}.yaml')
                self.get_logger().info('─' * 60)
                self.get_logger().info(
                    f'Load with: ros2 launch nav2_new lab_test.launch.py '
                    f'map:={map_path}.yaml')
            else:
                self.get_logger().error(f'map_saver_cli failed:\n{result.stderr}')
        except subprocess.TimeoutExpired:
            self.get_logger().error(
                'Timeout — is SLAM publishing /map? Run slam.launch.py first.')
        except FileNotFoundError:
            self.get_logger().error(
                'map_saver_cli not found — install ros-humble-nav2-map-server')

        raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = SaveMap()
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
