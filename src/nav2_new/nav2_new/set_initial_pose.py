"""
set_initial_pose – Publish AMCL initial pose based on team color.

Reads the spawn from arena_waypoints.yaml and publishes on /initialpose.
"""
import math
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class SetInitialPose(Node):
    def __init__(self):
        super().__init__('set_initial_pose')

        self.declare_parameter('team', 'red')
        self.declare_parameter('waypoints_file', '')

        team = self.get_parameter('team').value
        wp_file = self.get_parameter('waypoints_file').value

        if not wp_file:
            self.get_logger().error('waypoints_file is required')
            raise SystemExit(1)

        with open(wp_file, 'r') as f:
            cfg = yaml.safe_load(f)

        spawn = cfg.get('spawns', {}).get(team)
        if spawn is None:
            self.get_logger().error(f'No spawn defined for team "{team}"')
            raise SystemExit(1)

        self.spawn = spawn
        self.get_logger().info(
            f'Team={team}  spawn=({spawn["x"]:.2f}, {spawn["y"]:.2f}, yaw={spawn["yaw"]:.2f})')

        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        self.count = 0
        self.timer = self.create_timer(1.0, self._tick)

    def _tick(self):
        if self.count >= 5:
            self.get_logger().info('Initial pose published successfully.')
            self.timer.cancel()
            raise SystemExit(0)

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(self.spawn['x'])
        msg.pose.pose.position.y = float(self.spawn['y'])
        qx, qy, qz, qw = yaw_to_quat(float(self.spawn['yaw']))
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0]  = 0.25
        msg.pose.covariance[7]  = 0.25
        msg.pose.covariance[35] = 0.068

        self.pub.publish(msg)
        self.count += 1
        self.get_logger().info(f'Published initial pose ({self.count}/5)')


def main(args=None):
    rclpy.init(args=args)
    try:
        node = SetInitialPose()
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
