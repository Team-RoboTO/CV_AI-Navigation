"""
game_status_reporter.py

Publishes one human-readable status topic:
  /game_status std_msgs/String

The payload is compact JSON, so you can do:
  ros2 topic echo /game_status

It aggregates:
  - referee/micro state: team, game_progress/game_phase, match_started, health, center_status
  - selected strategy / waypoint manager state
  - CV readiness: recent /turret/cmd and optional /cv/detection
  - sensor readiness: recent /scan and /odom, TF odom->base_link, recent FAST-LIO errors from /rosout
  - navigation readiness: /map received, TF map->base_link exists, initial pose published,
    recent Nav2 errors from /rosout

This does not replace real diagnostics; it is a match dashboard topic for fast echo/debug.
"""

import json
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry, OccupancyGrid
from rcl_interfaces.msg import Log

import tf2_ros


class GameStatusReporter(Node):
    def __init__(self):
        super().__init__('game_status_reporter')

        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('fresh_timeout_sec', 1.0)
        self.declare_parameter('error_hold_sec', 3.0)
        self.declare_parameter('tf_timeout_sec', 0.05)

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.fresh_timeout = float(self.get_parameter('fresh_timeout_sec').value)
        self.error_hold = float(self.get_parameter('error_hold_sec').value)
        self.tf_timeout = float(self.get_parameter('tf_timeout_sec').value)
        rate = float(self.get_parameter('publish_rate').value)

        self.team = 'unknown'
        self.game_progress: Optional[int] = None
        self.game_phase = 'UNKNOWN'
        self.match_started = False
        self.match_ended = False
        self.health: Optional[int] = None
        self.center_status: Optional[int] = None
        self.strategy = 'none'
        self.game_fsm_state = 'unknown'
        self.waypoint_status = 'unknown'
        self.initial_pose_set = False
        self.nav_reset_done = False
        self.cv_detecting: Optional[bool] = None

        now = self._now()
        self.last_scan = 0.0
        self.last_imu = 0.0
        self.last_odom = 0.0
        self.last_map = 0.0
        self.map_received = False
        self.last_nav_cmd = 0.0
        self.last_turret_cmd = 0.0
        self.last_initial_pose_status = 0.0
        self.last_errors = {
            'no_effective_points': -1e9,
            'nav_out_of_bounds': -1e9,
            'no_valid_trajectories': -1e9,
            'amcl_no_pose': -1e9,
            'tf_timeout': -1e9,
        }
        self.start_time = now

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(String, '/game_status', 10)

        self.create_subscription(String, '/team', lambda m: setattr(self, 'team', m.data), 10)
        self.create_subscription(Int32, '/game_progress', lambda m: setattr(self, 'game_progress', int(m.data)), 10)
        self.create_subscription(String, '/game_phase', lambda m: setattr(self, 'game_phase', m.data), 10)
        self.create_subscription(Bool, '/match_started', lambda m: setattr(self, 'match_started', bool(m.data)), 10)
        self.create_subscription(Bool, '/match_ended', lambda m: setattr(self, 'match_ended', bool(m.data)), 10)
        self.create_subscription(Int32, '/health', lambda m: setattr(self, 'health', int(m.data)), 10)
        self.create_subscription(Int32, '/center_status', lambda m: setattr(self, 'center_status', int(m.data)), 10)
        self.create_subscription(String, '/strategy', lambda m: setattr(self, 'strategy', m.data), 10)
        self.create_subscription(String, '/game_state_manager/state', lambda m: setattr(self, 'game_fsm_state', m.data), 10)
        self.create_subscription(String, '/waypoint_manager/status', lambda m: setattr(self, 'waypoint_status', m.data), 10)
        self.create_subscription(Bool, '/nav_initial_pose_set', self._on_initial_pose_set, 10)
        self.create_subscription(Bool, '/nav_match_reset_done', lambda m: setattr(self, 'nav_reset_done', bool(m.data)), 10)
        self.create_subscription(Bool, '/cv/detection', lambda m: setattr(self, 'cv_detecting', bool(m.data)), 10)

        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'last_scan', self._now()), 10)
        self.create_subscription(Imu, '/livox/imu', lambda m: setattr(self, 'last_imu', self._now()), 10)
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'last_odom', self._now()), 10)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, 10)
        self.create_subscription(Twist, '/cmd_vel_NAV', lambda m: setattr(self, 'last_nav_cmd', self._now()), 10)
        self.create_subscription(Twist, '/turret/cmd', lambda m: setattr(self, 'last_turret_cmd', self._now()), 10)
        self.create_subscription(Log, '/rosout', self._on_rosout, 50)

        self.create_timer(1.0 / max(0.1, rate), self._tick)
        self.get_logger().info('game_status_reporter publishing /game_status')

    def _now(self) -> float:
        return time.monotonic()

    def _fresh(self, stamp: float, timeout: Optional[float] = None) -> bool:
        if timeout is None:
            timeout = self.fresh_timeout
        return (self._now() - stamp) <= timeout

    def _on_initial_pose_set(self, msg: Bool):
        self.initial_pose_set = bool(msg.data)
        self.last_initial_pose_status = self._now()

    def _on_map(self, msg: OccupancyGrid):
        self.last_map = self._now()
        self.map_received = True

    def _on_rosout(self, msg: Log):
        text = (msg.name + ' ' + msg.msg).lower()
        now = self._now()
        if 'no effective points' in text or 'no point, skip this scan' in text:
            self.last_errors['no_effective_points'] = now
        if 'out of bounds of the costmap' in text or 'sensor origin' in text and 'out of bounds' in text:
            self.last_errors['nav_out_of_bounds'] = now
        if 'no valid trajectories' in text:
            self.last_errors['no_valid_trajectories'] = now
        if 'amcl cannot publish a pose' in text or 'please set the initial pose' in text:
            self.last_errors['amcl_no_pose'] = now
        if 'timed out waiting for transform' in text or 'message filter dropping message' in text:
            self.last_errors['tf_timeout'] = now

    def _tf_ok(self, target: str, source: str) -> bool:
        try:
            self.tf_buffer.lookup_transform(
                target, source, rclpy.time.Time(), timeout=Duration(seconds=self.tf_timeout))
            return True
        except Exception:
            return False

    def _recent_error(self, key: str) -> bool:
        return (self._now() - self.last_errors.get(key, -1e9)) <= self.error_hold

    def _tick(self):
        scan_ok = self._fresh(self.last_scan, 2.0)
        imu_ok = self._fresh(self.last_imu, 2.0)
        odom_topic_ok = self._fresh(self.last_odom, 2.0)
        map_ok = self.map_received
        tf_odom_base = self._tf_ok(self.odom_frame, self.base_frame)
        tf_map_base = self._tf_ok(self.map_frame, self.base_frame)

        no_eff_recent = self._recent_error('no_effective_points')
        nav_oob_recent = self._recent_error('nav_out_of_bounds')
        no_traj_recent = self._recent_error('no_valid_trajectories')
        amcl_pose_recent = self._recent_error('amcl_no_pose')
        tf_timeout_recent = self._recent_error('tf_timeout')

        sensors_ready = scan_ok and imu_ok and odom_topic_ok and tf_odom_base and not no_eff_recent
        localization_ready = map_ok and tf_map_base and self.initial_pose_set and not amcl_pose_recent
        nav_ready = sensors_ready and localization_ready and not nav_oob_recent
        cv_ready = self._fresh(self.last_turret_cmd, 1.0)

        payload = {
            'team': self.team,
            'game_progress': self.game_progress,
            'phase': self.game_phase,
            'match_started': self.match_started,
            'match_ended': self.match_ended,
            'strategy': self.strategy,
            'fsm': self.game_fsm_state,
            'waypoint_manager': self.waypoint_status,
            'health': self.health,
            'center_status': self.center_status,
            'center_meaning': {0: 'free', 1: 'ours', 2: 'enemy'}.get(self.center_status, 'unknown'),
            'pipes': {
                'cv_ready': cv_ready,
                'cv_detecting': self.cv_detecting,
                'nav_ready': nav_ready,
                'sensors_ready': sensors_ready,
                'competition_ready': localization_ready,
            },
            'checks': {
                'scan_recent': scan_ok,
                'imu_recent': imu_ok,
                'odom_recent': odom_topic_ok,
                'map_recent': map_ok,
                'tf_odom_base': tf_odom_base,
                'tf_map_base': tf_map_base,
                'initial_pose_set': self.initial_pose_set,
                'nav_cmd_recent': self._fresh(self.last_nav_cmd, 1.0),
                'turret_cmd_recent': self._fresh(self.last_turret_cmd, 1.0),
            },
            'recent_errors': {
                'fastlio_no_effective_points': no_eff_recent,
                'nav_out_of_bounds': nav_oob_recent,
                'no_valid_trajectories': no_traj_recent,
                'amcl_needs_initial_pose': amcl_pose_recent,
                'tf_or_message_filter_timeout': tf_timeout_recent,
            },
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(',', ':'))
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = GameStatusReporter()
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
