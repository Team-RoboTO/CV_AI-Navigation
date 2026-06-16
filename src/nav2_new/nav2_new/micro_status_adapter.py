"""
micro_status_adapter – Adapt the serial bridge's /micro_status packet to
individual topics consumed by game_state_manager / waypoint_manager.

Input (from serial_new_communication_USB_C_final.py in the CV container):
    /micro_status  std_msgs/Float32MultiArray  data = [
        [0] color        # 0.0 = red, 1.0 = blue (convention TBD — set TEAM_MAP below)
        [1] start        # 1.0 = match running, 0.0 = not yet
        [2] health       # 0-100 %HP
        [3] ammo         # ignored
        [4] center       # 1.0 = we captured the center
        [5] resupply     # ignored
    ]

Output:
    /team              std_msgs/String   ("red" | "blue")
    /match_started     std_msgs/Bool
    /health            std_msgs/Int32    (0-100)
    /center_captured   std_msgs/Bool

These are the exact topics game_state_manager subscribes to, so plug-and-play.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, Bool, Int32


# Convention: which packet value maps to which team color.
# ⚠ Verify with the firmware team! If the convention flips, just swap these.
TEAM_MAP = {
    0.0: 'red',
    1.0: 'blue',
}


class MicroStatusAdapter(Node):
    def __init__(self):
        super().__init__('micro_status_adapter')

        self.declare_parameter('input_topic', '/micro_status')
        input_topic = self.get_parameter('input_topic').value

        self.create_subscription(
            Float32MultiArray, input_topic, self._on_packet, 10)

        self.pub_team = self.create_publisher(String, '/team', 10)
        self.pub_match_started = self.create_publisher(Bool, '/match_started', 10)
        self.pub_health = self.create_publisher(Int32, '/health', 10)
        self.pub_center = self.create_publisher(Bool, '/center_captured', 10)

        # Track last published values to avoid spam when unchanged
        self._last = {}

        self.get_logger().info(
            f'micro_status_adapter listening on {input_topic}')

    def _on_packet(self, msg: Float32MultiArray):
        data = list(msg.data)
        if len(data) < 5:
            self.get_logger().warn_once(
                f'/micro_status packet too short: {len(data)} < 5')
            return

        color_raw = round(float(data[0]))
        start_raw = float(data[1])
        health_raw = float(data[2])
        center_raw = float(data[4])

        # --- Team color ---
        team = TEAM_MAP.get(float(color_raw))
        if team and self._last.get('team') != team:
            m = String()
            m.data = team
            self.pub_team.publish(m)
            self._last['team'] = team
            self.get_logger().info(f'team → {team}')

        # --- Match started ---
        started = start_raw > 0.5
        if self._last.get('match_started') != started:
            m = Bool()
            m.data = started
            self.pub_match_started.publish(m)
            self._last['match_started'] = started
            self.get_logger().info(f'match_started → {started}')

        # --- Health ---
        hp = int(max(0, min(100, health_raw)))
        if self._last.get('health') != hp:
            m = Int32()
            m.data = hp
            self.pub_health.publish(m)
            self._last['health'] = hp

        # --- Center captured ---
        captured = center_raw > 0.5
        if self._last.get('center_captured') != captured:
            m = Bool()
            m.data = captured
            self.pub_center.publish(m)
            self._last['center_captured'] = captured
            self.get_logger().info(f'center_captured → {captured}')


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MicroStatusAdapter()
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
