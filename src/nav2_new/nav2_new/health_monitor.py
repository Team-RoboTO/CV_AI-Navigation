"""
health_monitor – Switch navigation strategy based on robot HP.

Subscribes to /health (std_msgs/Int32 or Float32 — interpreted as %HP 0-100)
and publishes to /strategy (std_msgs/String) to command the waypoint_manager.

Logic (configurable in arena_waypoints.yaml under `health:`):
  • HP < retreat_threshold  → switch to low_hp_strategy  (default: retreat_to_spawn)
  • HP > safe_threshold     → switch back to default_strategy (default: rush_center)
  • Between thresholds      → no change (hysteresis)

Publish your game HP topic to /health from the referee/game-state node.
If you don't have a referee feed, just don't launch this node.
"""
import yaml

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32


class HealthMonitor(Node):
    def __init__(self):
        super().__init__('health_monitor')

        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('health_topic', '/health')

        wp_file = self.get_parameter('waypoints_file').value
        if not wp_file:
            self.get_logger().error('waypoints_file is required')
            raise SystemExit(1)

        with open(wp_file, 'r') as f:
            cfg = yaml.safe_load(f)

        hcfg = cfg.get('health', {})
        self.retreat_th = float(hcfg.get('retreat_threshold', 30))
        self.safe_th = float(hcfg.get('safe_threshold', 70))
        self.low_hp_strategy = hcfg.get('low_hp_strategy', 'retreat_to_spawn')
        self.default_strategy = hcfg.get('default_strategy', 'rush_center')

        health_topic = self.get_parameter('health_topic').value

        self.current_strategy = self.default_strategy
        self.last_hp = 100.0

        # Accept both Int32 and Float32 on the same topic
        self.create_subscription(Int32, health_topic, self._on_int, 10)
        self.create_subscription(Float32, health_topic, self._on_float, 10)

        self.pub = self.create_publisher(String, '/strategy', 10)

        self.get_logger().info(
            f'Health monitor: retreat<{self.retreat_th}  safe>{self.safe_th}  '
            f'low_hp→{self.low_hp_strategy}  default→{self.default_strategy}')

    def _on_int(self, msg: Int32):
        self._update(float(msg.data))

    def _on_float(self, msg: Float32):
        self._update(float(msg.data))

    def _update(self, hp: float):
        self.last_hp = hp

        new_strategy = None
        if hp < self.retreat_th and self.current_strategy != self.low_hp_strategy:
            new_strategy = self.low_hp_strategy
            reason = f'HP {hp:.0f}% < {self.retreat_th:.0f}%'
        elif hp > self.safe_th and self.current_strategy == self.low_hp_strategy:
            new_strategy = self.default_strategy
            reason = f'HP {hp:.0f}% > {self.safe_th:.0f}%'

        if new_strategy is not None:
            self.get_logger().info(
                f'{reason} → switching strategy: {self.current_strategy} → {new_strategy}')
            self.current_strategy = new_strategy
            msg = String()
            msg.data = new_strategy
            self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = HealthMonitor()
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
