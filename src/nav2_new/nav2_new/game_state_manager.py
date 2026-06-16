"""
game_state_manager – Unified behavior coordinator for the match.

Subscribes to the individual telemetry topics published by micro_status_parser
and publishes high-level commands:
    /strategy   std_msgs/String   — to waypoint_manager
    /team       std_msgs/String   — echoed / forced, read by waypoint_manager

State machine:

    [WAITING]           match_state != "running"  → wait_at_spawn
        │  match_started
        ▼
    [RUSHING]           go to center using team tunnel
        │  center_captured=true
        ▼
    [HOLDING]           patrol diagonal of center
        │
        ├─ HP < retreat_threshold   → [RETREATING]
        │
        │  center_captured=false (lost it)
        ▼
    [RUSHING]           back to rush

    [RETREATING]        retreat_to_spawn
        │  HP > safe_threshold
        ▼
    [RUSHING]

Team color is read from /team. Missing/unknown → stays WAITING.
"""
import yaml

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Bool, Float32


STATE_WAITING = 'waiting'
STATE_RUSHING = 'rushing'
STATE_HOLDING = 'holding'
STATE_RETREATING = 'retreating'


class GameStateManager(Node):
    def __init__(self):
        super().__init__('game_state_manager')

        self.declare_parameter('waypoints_file', '')

        wp_file = self.get_parameter('waypoints_file').value
        if not wp_file:
            self.get_logger().error('waypoints_file is required')
            raise SystemExit(1)

        with open(wp_file, 'r') as f:
            cfg = yaml.safe_load(f)

        h = cfg.get('health', {})
        self.retreat_th = float(h.get('retreat_threshold', 30))
        self.safe_th = float(h.get('safe_threshold', 70))
        self.s_low_hp = h.get('low_hp_strategy', 'retreat_to_spawn')
        self.s_default = h.get('default_strategy', 'rush_center')
        self.s_hold = h.get('hold_strategy', 'hold_center_diagonal')
        self.s_wait = 'wait_at_spawn'

        # State
        self.state = STATE_WAITING
        self.team = None
        self.health = 100
        self.match_started = False
        self.center_captured = False

        # Subscriptions
        self.create_subscription(String, '/team', self._on_team, 10)
        self.create_subscription(Int32, '/health', self._on_health_int, 10)
        self.create_subscription(Float32, '/health', self._on_health_float, 10)
        self.create_subscription(Bool, '/match_started', self._on_match_started, 10)
        self.create_subscription(
            Bool, '/center_captured', self._on_center_captured, 10)

        # Publishers
        self.pub_strategy = self.create_publisher(String, '/strategy', 10)
        self.pub_state = self.create_publisher(
            String, '/game_state_manager/state', 10)

        # FSM tick at 2 Hz (enough — transitions are event-driven mostly)
        self.create_timer(0.5, self._tick)

        self.get_logger().info(
            f'game_state_manager started. '
            f'retreat<{self.retreat_th}%  safe>{self.safe_th}%')

    # ─── Callbacks ───────────────────────────────────────────────────────────

    def _on_team(self, msg: String):
        new = msg.data.strip().lower()
        if new in ('red', 'blue') and new != self.team:
            self.get_logger().info(f'Team set to: {new}')
            self.team = new

    def _on_health_int(self, msg: Int32):
        self.health = float(msg.data)

    def _on_health_float(self, msg: Float32):
        self.health = float(msg.data)

    def _on_match_started(self, msg: Bool):
        new = bool(msg.data)
        if new and not self.match_started:
            self.get_logger().info('MATCH STARTED')
        elif not new and self.match_started:
            self.get_logger().info('Match ended / paused')
        self.match_started = new

    def _on_center_captured(self, msg: Bool):
        new = bool(msg.data)
        if new and not self.center_captured:
            self.get_logger().info('Center CAPTURED by us')
        elif not new and self.center_captured:
            self.get_logger().info('Center LOST')
        self.center_captured = new

    # ─── FSM ─────────────────────────────────────────────────────────────────

    def _tick(self):
        new_state = self._compute_state()

        if new_state != self.state:
            self.get_logger().info(
                f'STATE {self.state} → {new_state}  '
                f'(HP={self.health:.0f}%, team={self.team}, '
                f'match={self.match_started}, center={self.center_captured})')
            self.state = new_state
            self._publish_strategy_for_state()

        # Publish state continuously
        m = String()
        m.data = f'{self.state} team={self.team} hp={self.health:.0f}'
        self.pub_state.publish(m)

    def _compute_state(self) -> str:
        # No team yet — can't act
        if self.team is None:
            return STATE_WAITING

        # Match not running → wait
        if not self.match_started:
            return STATE_WAITING

        # Low HP trumps everything (except waiting)
        if self.health < self.retreat_th:
            return STATE_RETREATING

        # Still retreating? only exit when safe
        if self.state == STATE_RETREATING and self.health <= self.safe_th:
            return STATE_RETREATING

        # Center is ours → hold it
        if self.center_captured:
            return STATE_HOLDING

        # Default: push for center
        return STATE_RUSHING

    def _publish_strategy_for_state(self):
        strategy = {
            STATE_WAITING:    self.s_wait,
            STATE_RUSHING:    self.s_default,
            STATE_HOLDING:    self.s_hold,
            STATE_RETREATING: self.s_low_hp,
        }[self.state]

        m = String()
        m.data = strategy
        self.pub_strategy.publish(m)
        self.get_logger().info(f'→ /strategy: {strategy}')


def main(args=None):
    rclpy.init(args=args)
    try:
        node = GameStateManager()
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
