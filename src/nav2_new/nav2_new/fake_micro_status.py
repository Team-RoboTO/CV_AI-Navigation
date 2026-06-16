"""
fake_micro_status – Simulate the STM32 publishing /micro_status.

Publishes JSON strings matching the micro_status_parser protocol.
Useful for testing game_state_manager and full pipeline without hardware.

Interactive mode via service calls:
  ros2 topic pub --once /fake_micro/start_match std_msgs/Bool "data: true"
  ros2 topic pub --once /fake_micro/set_hp std_msgs/Int32 "data: 20"
  ros2 topic pub --once /fake_micro/set_team std_msgs/String "data: 'blue'"
  ros2 topic pub --once /fake_micro/center_captured std_msgs/Bool "data: true"

Usage:
  ros2 run nav2_new fake_micro_status
  ros2 run nav2_new fake_micro_status --ros-args -p team:=blue
"""
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Bool


class FakeMicroStatus(Node):
    def __init__(self):
        super().__init__('fake_micro_status')

        self.declare_parameter('team', 'red')
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('start_match_now', False)

        # State
        self.team = self.get_parameter('team').value
        self.health = 100
        self.match_running = bool(self.get_parameter('start_match_now').value)
        self.match_time = 0
        self.center_captured = False
        self.enemy_captured = False
        self.t0 = time.monotonic()

        # Control topics
        self.create_subscription(Bool, '/fake_micro/start_match',
                                 self._on_start_match, 10)
        self.create_subscription(Int32, '/fake_micro/set_hp',
                                 self._on_set_hp, 10)
        self.create_subscription(String, '/fake_micro/set_team',
                                 self._on_set_team, 10)
        self.create_subscription(Bool, '/fake_micro/center_captured',
                                 self._on_center, 10)

        # Output
        self.pub = self.create_publisher(String, '/micro_status', 10)

        rate = float(self.get_parameter('rate').value)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f'fake_micro_status started. team={self.team}  '
            f'match_running={self.match_running}')
        self.get_logger().info(
            'Use these commands to simulate game events:')
        self.get_logger().info(
            '  ros2 topic pub --once /fake_micro/start_match std_msgs/Bool "data: true"')
        self.get_logger().info(
            '  ros2 topic pub --once /fake_micro/set_hp std_msgs/Int32 "data: 20"')
        self.get_logger().info(
            '  ros2 topic pub --once /fake_micro/center_captured std_msgs/Bool "data: true"')

    def _on_start_match(self, msg: Bool):
        self.match_running = bool(msg.data)
        if self.match_running:
            self.t0 = time.monotonic()
            self.get_logger().info('[fake] Match STARTED')
        else:
            self.get_logger().info('[fake] Match STOPPED')

    def _on_set_hp(self, msg: Int32):
        self.health = max(0, min(100, int(msg.data)))
        self.get_logger().info(f'[fake] HP set to {self.health}')

    def _on_set_team(self, msg: String):
        t = msg.data.strip().lower()
        if t in ('red', 'blue'):
            self.team = t
            self.get_logger().info(f'[fake] Team set to {t}')

    def _on_center(self, msg: Bool):
        self.center_captured = bool(msg.data)
        self.get_logger().info(
            f'[fake] center_captured = {self.center_captured}')

    def _tick(self):
        if self.match_running:
            self.match_time = int(time.monotonic() - self.t0)

        data = {
            'team': self.team,
            'health': self.health,
            'match_state': 'running' if self.match_running else 'waiting',
            'match_time': self.match_time,
            'center_captured': self.center_captured,
            'enemy_captured_center': self.enemy_captured,
        }

        msg = String()
        msg.data = json.dumps(data)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = FakeMicroStatus()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
