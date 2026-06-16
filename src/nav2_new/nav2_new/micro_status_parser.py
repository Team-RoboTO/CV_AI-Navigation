"""
micro_status_parser – Bridge /micro_status (JSON string) → individual ROS topics.

Your STM32 publishes a single JSON-encoded string on /micro_status carrying
ALL game telemetry. This node parses it and republishes the fields on
individual, ROS-idiomatic topics so other nodes can subscribe to what they
care about.

Expected JSON schema on /micro_status (std_msgs/String):

    {
      "team": "blue",                 # "red" | "blue"
      "health": 85,                   # 0-100 %HP
      "match_state": "running",       # "waiting" | "running" | "paused" | "ended"
      "center_captured": false,       # bool — we're holding the center
      "enemy_captured_center": false, # bool — enemy holds it
      "match_time": 123,              # seconds since match start
      "shooter_heat": 40,             # 0-max, optional
      "shooter_can_fire": true        # optional
    }

Republished topics:
    /team                std_msgs/String    ("red" | "blue")
    /health              std_msgs/Int32     (0-100)
    /match_state         std_msgs/String
    /match_started       std_msgs/Bool      (true iff match_state == "running")
    /center_captured     std_msgs/Bool
    /enemy_captured_center  std_msgs/Bool
    /match_time          std_msgs/Int32     (seconds)

Missing / unknown keys are skipped silently.
"""
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Bool


class MicroStatusParser(Node):
    def __init__(self):
        super().__init__('micro_status_parser')

        self.declare_parameter('input_topic', '/micro_status')
        input_topic = self.get_parameter('input_topic').value

        self.create_subscription(String, input_topic, self._on_status, 10)

        # Publishers — latched (transient local) for state info
        self.pub_team = self.create_publisher(String, '/team', 10)
        self.pub_health = self.create_publisher(Int32, '/health', 10)
        self.pub_match_state = self.create_publisher(String, '/match_state', 10)
        self.pub_match_started = self.create_publisher(Bool, '/match_started', 10)
        self.pub_center_captured = self.create_publisher(
            Bool, '/center_captured', 10)
        self.pub_enemy_captured = self.create_publisher(
            Bool, '/enemy_captured_center', 10)
        self.pub_match_time = self.create_publisher(Int32, '/match_time', 10)

        # Track last values so we don't spam when unchanged
        self._last = {}

        self.get_logger().info(f'micro_status_parser listening on {input_topic}')

    def _on_status(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'Bad JSON on micro_status: {e}')
            return
        except Exception as e:
            self.get_logger().warn(f'micro_status parse error: {e}')
            return

        self._pub_string('team', data.get('team'), self.pub_team)
        self._pub_int('health', data.get('health'), self.pub_health)
        self._pub_string('match_state', data.get('match_state'),
                         self.pub_match_state)
        self._pub_int('match_time', data.get('match_time'), self.pub_match_time)

        match_state = data.get('match_state')
        if match_state is not None:
            self._pub_bool('match_started', match_state == 'running',
                           self.pub_match_started)

        if 'center_captured' in data:
            self._pub_bool('center_captured', bool(data['center_captured']),
                           self.pub_center_captured)
        if 'enemy_captured_center' in data:
            self._pub_bool('enemy_captured_center',
                           bool(data['enemy_captured_center']),
                           self.pub_enemy_captured)

    def _pub_string(self, key, value, pub):
        if value is None:
            return
        value = str(value)
        if self._last.get(key) == value:
            return
        self._last[key] = value
        m = String()
        m.data = value
        pub.publish(m)

    def _pub_int(self, key, value, pub):
        if value is None:
            return
        try:
            value = int(value)
        except (TypeError, ValueError):
            return
        if self._last.get(key) == value:
            return
        self._last[key] = value
        m = Int32()
        m.data = value
        pub.publish(m)

    def _pub_bool(self, key, value, pub):
        value = bool(value)
        if self._last.get(key) == value:
            return
        self._last[key] = value
        m = Bool()
        m.data = value
        pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MicroStatusParser()
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
