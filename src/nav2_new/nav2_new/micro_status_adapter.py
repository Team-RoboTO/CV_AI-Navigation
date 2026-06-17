"""
micro_status_adapter.py

Adapts /micro_status Float32MultiArray from the CV serial bridge into clean
navigation topics and referee/game topics.

Expected /micro_status layout from updated serial_bridge.cpp:
  data[0..9]    = RX values from the micro
  data[10..15]  = TX echo [turret_yaw, turret_pitch, shoot, nav_x, nav_y, nav_angle]

Firmware RX layout from the screenshot you sent:
  0 = current turret yaw                    [rad]
  1 = current turret pitch                  [deg or firmware unit]
  2 = chassis vx/status
  3 = chassis vy/status
  4 = color/team                            0 red, 1 blue
  5 = referee game_progress                 1 prep, 2 15s init, 3 5s countdown,
                                            4 in match, 5 match settling/ended
  6 = health/current HP
  7 = resupply zone bit/status
  8 = center status                         0 free, 1 ours, 2 enemy
  9 = reserved/fake

Publishes:
  /team std_msgs/String                     "red" | "blue"
  /game_progress std_msgs/Int32             raw referee game_progress
  /game_phase std_msgs/String               PREPARATION | INIT_15S | COUNTDOWN_5S | IN_MATCH | MATCH_SETTLING | UNKNOWN_N
  /match_started std_msgs/Bool              true when game_progress == in_match_value
  /match_ended std_msgs/Bool                true when game_progress == match_ended_value or after leaving IN_MATCH
  /health std_msgs/Int32
  /resupply_status std_msgs/Int32
  /center_status std_msgs/Int32             0 free, 1 ours, 2 enemy
  /center_captured std_msgs/Bool            true when center_status == ours
  /enemy_center_captured std_msgs/Bool      true when center_status == enemy
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, Bool, Int32


class MicroStatusAdapter(Node):
    def __init__(self):
        super().__init__('micro_status_adapter')

        self.declare_parameter('input_topic', '/micro_status')

        self.declare_parameter('team_index', 4)
        self.declare_parameter('game_progress_index', 5)
        # Alias kept for compatibility with older params files.
        self.declare_parameter('match_started_index', 5)
        self.declare_parameter('health_index', 6)
        self.declare_parameter('resupply_status_index', 7)
        self.declare_parameter('center_status_index', 8)

        self.declare_parameter('red_value', 0.0)
        self.declare_parameter('blue_value', 1.0)

        self.declare_parameter('preparation_value', 1)
        self.declare_parameter('init_15s_value', 2)
        self.declare_parameter('countdown_5s_value', 3)
        self.declare_parameter('in_match_value', 4)
        self.declare_parameter('match_ended_value', 5)
        self.declare_parameter('match_started_threshold', 0.5)  # fallback only

        self.declare_parameter('center_free_value', 0)
        self.declare_parameter('center_ours_value', 1)
        self.declare_parameter('center_enemy_value', 2)

        self.input_topic = self.get_parameter('input_topic').value
        self.team_index = int(self.get_parameter('team_index').value)
        self.game_progress_index = int(self.get_parameter('game_progress_index').value)
        self.health_index = int(self.get_parameter('health_index').value)
        self.resupply_status_index = int(self.get_parameter('resupply_status_index').value)
        self.center_status_index = int(self.get_parameter('center_status_index').value)

        self.red_value = float(self.get_parameter('red_value').value)
        self.blue_value = float(self.get_parameter('blue_value').value)
        self.match_started_threshold = float(self.get_parameter('match_started_threshold').value)

        self.prep_value = int(self.get_parameter('preparation_value').value)
        self.init15_value = int(self.get_parameter('init_15s_value').value)
        self.countdown5_value = int(self.get_parameter('countdown_5s_value').value)
        self.in_match_value = int(self.get_parameter('in_match_value').value)
        self.match_ended_value = int(self.get_parameter('match_ended_value').value)

        self.center_free_value = int(self.get_parameter('center_free_value').value)
        self.center_ours_value = int(self.get_parameter('center_ours_value').value)
        self.center_enemy_value = int(self.get_parameter('center_enemy_value').value)

        self.create_subscription(Float32MultiArray, self.input_topic, self._on_packet, 10)

        self.pub_team = self.create_publisher(String, '/team', 10)
        self.pub_game_progress = self.create_publisher(Int32, '/game_progress', 10)
        self.pub_game_phase = self.create_publisher(String, '/game_phase', 10)
        self.pub_match_started = self.create_publisher(Bool, '/match_started', 10)
        self.pub_match_ended = self.create_publisher(Bool, '/match_ended', 10)
        self.pub_health = self.create_publisher(Int32, '/health', 10)
        self.pub_resupply = self.create_publisher(Int32, '/resupply_status', 10)
        self.pub_center_status = self.create_publisher(Int32, '/center_status', 10)
        self.pub_center_captured = self.create_publisher(Bool, '/center_captured', 10)
        self.pub_enemy_center = self.create_publisher(Bool, '/enemy_center_captured', 10)

        self._last = {}
        self._was_in_match = False
        self.get_logger().info(
            f'micro_status_adapter listening on {self.input_topic}; indices '
            f'team={self.team_index}, progress={self.game_progress_index}, '
            f'hp={self.health_index}, resupply={self.resupply_status_index}, center={self.center_status_index}'
        )

    def _get(self, data, idx, name):
        if idx < 0 or idx >= len(data):
            self.get_logger().warn_once(
                f'/micro_status packet too short for {name}: index {idx}, len={len(data)}'
            )
            return None
        v = float(data[idx])
        if math.isnan(v) or math.isinf(v):
            return None
        return v

    def _publish_if_changed(self, key, pub, msg):
        value = msg.data
        if self._last.get(key) != value:
            pub.publish(msg)
            self._last[key] = value
            self.get_logger().info(f'{key} -> {value}')
        else:
            # Republishing is useful for late subscribers; keep it light.
            pub.publish(msg)

    def _phase_from_progress(self, progress: int) -> str:
        if progress == self.prep_value:
            return 'PREPARATION'
        if progress == self.init15_value:
            return 'INIT_15S'
        if progress == self.countdown5_value:
            return 'COUNTDOWN_5S'
        if progress == self.in_match_value:
            return 'IN_MATCH'
        if progress == self.match_ended_value:
            return 'MATCH_SETTLING'
        return f'UNKNOWN_{progress}'

    def _on_packet(self, msg: Float32MultiArray):
        data = list(msg.data)

        team_raw = self._get(data, self.team_index, 'team')
        progress_raw = self._get(data, self.game_progress_index, 'game_progress')
        health_raw = self._get(data, self.health_index, 'health')
        resupply_raw = self._get(data, self.resupply_status_index, 'resupply_status')
        center_raw = self._get(data, self.center_status_index, 'center_status')

        if team_raw is not None:
            if abs(team_raw - self.red_value) < 0.25:
                team = 'red'
            elif abs(team_raw - self.blue_value) < 0.25:
                team = 'blue'
            else:
                team = None
            if team is not None:
                m = String(); m.data = team
                self._publish_if_changed('team', self.pub_team, m)

        if progress_raw is not None:
            progress = int(round(progress_raw))
            phase = self._phase_from_progress(progress)
            in_match = progress == self.in_match_value
            # ended is true for explicit match settling, and also for the first transition out of IN_MATCH.
            match_ended = (progress == self.match_ended_value) or (self._was_in_match and not in_match)
            self._was_in_match = in_match

            m = Int32(); m.data = progress
            self._publish_if_changed('game_progress', self.pub_game_progress, m)
            s = String(); s.data = phase
            self._publish_if_changed('game_phase', self.pub_game_phase, s)
            b = Bool(); b.data = in_match
            self._publish_if_changed('match_started', self.pub_match_started, b)
            e = Bool(); e.data = match_ended
            self._publish_if_changed('match_ended', self.pub_match_ended, e)

        if health_raw is not None:
            m = Int32(); m.data = int(max(0, min(1000, round(health_raw))))
            self._publish_if_changed('health', self.pub_health, m)

        if resupply_raw is not None:
            m = Int32(); m.data = int(round(resupply_raw))
            self._publish_if_changed('resupply_status', self.pub_resupply, m)

        if center_raw is not None:
            center = int(round(center_raw))
            if center not in (self.center_free_value, self.center_ours_value, self.center_enemy_value):
                self.get_logger().warn(f'Unexpected center_status value: {center}')

            m = Int32(); m.data = center
            self._publish_if_changed('center_status', self.pub_center_status, m)
            ours = Bool(); ours.data = center == self.center_ours_value
            self._publish_if_changed('center_captured', self.pub_center_captured, ours)
            enemy = Bool(); enemy.data = center == self.center_enemy_value
            self._publish_if_changed('enemy_center_captured', self.pub_enemy_center, enemy)


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
