"""
set_initial_pose.py

Publishes AMCL initial pose from arena_waypoints.yaml spawns[team].

Use cases:
  - At startup, automatically seed AMCL with the spawn for red/blue.
  - At match reset/end, publish the spawn pose again so the next match starts clean.

Publishes:
  /initialpose geometry_msgs/PoseWithCovarianceStamped
  /nav_initial_pose_set std_msgs/Bool
  /nav_initial_pose_status std_msgs/String
"""

import math
import time
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String, Bool


def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class SetInitialPose(Node):
    def __init__(self):
        super().__init__('set_initial_pose')

        self.declare_parameter('team', 'red')
        self.declare_parameter('use_team_topic', True)
        self.declare_parameter('team_topic', '/team')
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('initialpose_topic', '/initialpose')
        self.declare_parameter('publish_count', 8)
        self.declare_parameter('publish_period', 0.5)
        self.declare_parameter('start_delay', 2.0)
        self.declare_parameter('fallback_team_after_sec', 8.0)
        self.declare_parameter('cov_x', 0.25)
        self.declare_parameter('cov_y', 0.25)
        self.declare_parameter('cov_yaw', 0.068)
        self.declare_parameter('republish_on_match_ended', True)
        self.declare_parameter('republish_on_team_change', True)
        self.declare_parameter('match_ended_topic', '/match_ended')

        self.team = str(self.get_parameter('team').value).lower()
        self.use_team_topic = bool(self.get_parameter('use_team_topic').value)
        self.team_topic = self.get_parameter('team_topic').value
        self.wp_file = self.get_parameter('waypoints_file').value
        self.initialpose_topic = self.get_parameter('initialpose_topic').value
        self.publish_count = int(self.get_parameter('publish_count').value)
        self.publish_period = float(self.get_parameter('publish_period').value)
        self.start_delay = float(self.get_parameter('start_delay').value)
        self.fallback_team_after_sec = float(self.get_parameter('fallback_team_after_sec').value)
        self.cov_x = float(self.get_parameter('cov_x').value)
        self.cov_y = float(self.get_parameter('cov_y').value)
        self.cov_yaw = float(self.get_parameter('cov_yaw').value)
        self.republish_on_match_ended = bool(self.get_parameter('republish_on_match_ended').value)
        self.republish_on_team_change = bool(self.get_parameter('republish_on_team_change').value)
        self.match_ended_topic = self.get_parameter('match_ended_topic').value

        if not self.wp_file:
            self.get_logger().error('waypoints_file is required')
            raise SystemExit(1)

        with open(self.wp_file, 'r') as f:
            self.cfg = yaml.safe_load(f)
        self.spawns = self.cfg.get('spawns', {})

        self.pub = self.create_publisher(PoseWithCovarianceStamped, self.initialpose_topic, 10)
        self.pub_set = self.create_publisher(Bool, '/nav_initial_pose_set', 10)
        self.pub_status = self.create_publisher(String, '/nav_initial_pose_status', 10)

        self.team_received = not self.use_team_topic
        self.start_time = time.monotonic()
        self.burst_remaining = 0
        self.next_publish_time = 0.0
        self.started = False
        self.first_burst_scheduled = False

        if self.use_team_topic:
            self.create_subscription(String, self.team_topic, self._on_team, 10)
        if self.republish_on_match_ended:
            self.create_subscription(Bool, self.match_ended_topic, self._on_match_ended, 10)

        self.timer = self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f'set_initial_pose waiting. default_team={self.team} use_team_topic={self.use_team_topic}'
        )

    def _publish_status(self, text: str):
        m = String(); m.data = text
        self.pub_status.publish(m)

    def _publish_set(self, value: bool):
        m = Bool(); m.data = value
        self.pub_set.publish(m)

    def _schedule_burst(self, reason: str):
        self.burst_remaining = self.publish_count
        self.next_publish_time = time.monotonic()
        self._publish_set(False)
        self._publish_status(f'scheduled team={self.team} reason={reason}')
        self.get_logger().info(f'Scheduled initial pose burst: team={self.team}, reason={reason}')

    def _on_team(self, msg: String):
        t = msg.data.strip().lower()
        if t not in ('red', 'blue'):
            return
        if t != self.team:
            self.get_logger().info(f'team from micro: {self.team} -> {t}')
            self.team = t
            if self.republish_on_team_change and self.team_received:
                self._schedule_burst('team_changed')
        self.team_received = True

    def _on_match_ended(self, msg: Bool):
        if msg.data:
            self._schedule_burst('match_ended')

    def _tick(self):
        now = time.monotonic()

        if not self.team_received:
            if now - self.start_time < self.fallback_team_after_sec:
                return
            self.team_received = True
            self.get_logger().warn(
                f'No /team received after {self.fallback_team_after_sec:.1f}s; using fallback team={self.team}'
            )

        if not self.first_burst_scheduled:
            if now - self.start_time < self.start_delay:
                return
            self.first_burst_scheduled = True
            self._schedule_burst('startup')

        if self.burst_remaining <= 0:
            return
        if now < self.next_publish_time:
            return

        spawn = self.spawns.get(self.team)
        if spawn is None:
            self.get_logger().error(f'No spawn defined for team "{self.team}"')
            self._publish_status(f'error no_spawn team={self.team}')
            self.burst_remaining = 0
            return

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(spawn['x'])
        msg.pose.pose.position.y = float(spawn['y'])
        qx, qy, qz, qw = yaw_to_quat(float(spawn['yaw']))
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = self.cov_x
        msg.pose.covariance[7] = self.cov_y
        msg.pose.covariance[35] = self.cov_yaw

        self.pub.publish(msg)
        self.burst_remaining -= 1
        sent = self.publish_count - self.burst_remaining
        self.next_publish_time = now + self.publish_period

        self.get_logger().info(
            f'Published initial pose for team={self.team} ({sent}/{self.publish_count}) '
            f'x={spawn["x"]:.2f} y={spawn["y"]:.2f} yaw={spawn["yaw"]:.2f}'
        )

        if self.burst_remaining == 0:
            self._publish_set(True)
            self._publish_status(
                f'set team={self.team} x={float(spawn["x"]):.3f} y={float(spawn["y"]):.3f} yaw={float(spawn["yaw"]):.3f}'
            )
            self.get_logger().info('Initial pose publishing complete.')


def main(args=None):
    rclpy.init(args=args)
    try:
        node = SetInitialPose()
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
