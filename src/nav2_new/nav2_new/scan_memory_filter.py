#!/usr/bin/env python3
"""
scan_memory_filter - short memory/gating filter for LaserScan obstacles.

Why this exists in this robot:
- The Livox Mid-360 is mounted on a moving turret/head.
- Nav2 costmaps assume the scan sensor is fixed with respect to base_link.
- When CV moves the head, a raw /scan can draw fake obstacles around the robot.

This node republishes /scan_nav with short memory when the head is stable and
publishes a clearing scan while the turret/CV is active or the yaw command is
changing. That keeps Nav2 from accumulating fake obstacles during autoaim.

Typical use:
  /scan -> scan_memory_filter -> /scan_nav
  AMCL/local costmap observe /scan_nav
  global costmap uses only static_layer + inflation_layer
"""

import math
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ScanMemoryFilter(Node):
    def __init__(self):
        super().__init__('scan_memory_filter')

        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_nav')
        self.declare_parameter('persistence_sec', 0.15)
        self.declare_parameter('min_range', 0.20)
        self.declare_parameter('max_range', 4.0)
        self.declare_parameter('prefer_closer', True)

        # Gating while CV/turret moves the head-mounted Livox.
        self.declare_parameter('gate_enabled', True)
        self.declare_parameter('gate_topic', '/turret/cmd')
        self.declare_parameter('cv_mode_threshold', 0.5)       # /turret/cmd.linear.z >= threshold => CV active
        self.declare_parameter('gate_hold_sec', 0.40)          # keep gate active briefly after last CV/motion event
        self.declare_parameter('gate_on_turret_motion', True)
        self.declare_parameter('turret_motion_yaw_delta', 0.03) # rad, command-space delta per received cmd
        self.declare_parameter('publish_clear_scan_when_gated', True)
        self.declare_parameter('log_gate_transitions', True)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.persistence_sec = float(self.get_parameter('persistence_sec').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.prefer_closer = bool(self.get_parameter('prefer_closer').value)

        self.gate_enabled = bool(self.get_parameter('gate_enabled').value)
        self.gate_topic = str(self.get_parameter('gate_topic').value)
        self.cv_mode_threshold = float(self.get_parameter('cv_mode_threshold').value)
        self.gate_hold_sec = float(self.get_parameter('gate_hold_sec').value)
        self.gate_on_turret_motion = bool(self.get_parameter('gate_on_turret_motion').value)
        self.turret_motion_yaw_delta = float(self.get_parameter('turret_motion_yaw_delta').value)
        self.publish_clear_scan_when_gated = bool(self.get_parameter('publish_clear_scan_when_gated').value)
        self.log_gate_transitions = bool(self.get_parameter('log_gate_transitions').value)

        self.last_ranges: Optional[List[float]] = None
        self.last_times: Optional[List[float]] = None
        self.last_signature = None

        self.last_gate_event_time = -1e9
        self.last_turret_yaw: Optional[float] = None
        self.gate_was_active = False

        self.sub = self.create_subscription(LaserScan, self.input_topic, self._cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(LaserScan, self.output_topic, qos_profile_sensor_data)

        if self.gate_enabled:
            self.turret_sub = self.create_subscription(Twist, self.gate_topic, self._turret_cb, 10)
        else:
            self.turret_sub = None

        self.get_logger().info(
            f'scan_memory_filter: {self.input_topic} -> {self.output_topic}; '
            f'persistence={self.persistence_sec:.2f}s; range=[{self.min_range:.2f}, {self.max_range:.2f}]; '
            f'gate_enabled={self.gate_enabled}; gate_topic={self.gate_topic}; gate_hold={self.gate_hold_sec:.2f}s'
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _valid(self, r: float) -> bool:
        return math.isfinite(r) and self.min_range <= r <= self.max_range

    def _turret_cb(self, msg: Twist):
        if not self.gate_enabled:
            return

        now = self._now_sec()
        gate_event = False

        # Convention used in this robot: /turret/cmd.linear.z = 1.0 means CV mode.
        if msg.linear.z >= self.cv_mode_threshold:
            gate_event = True

        # Also gate while yaw command changes. This catches idle/target slews even if mode is 0.
        yaw = float(msg.angular.z)
        if self.gate_on_turret_motion and self.last_turret_yaw is not None:
            if abs(yaw - self.last_turret_yaw) >= self.turret_motion_yaw_delta:
                gate_event = True
        self.last_turret_yaw = yaw

        if gate_event:
            self.last_gate_event_time = now

    def _gate_active(self, now: float) -> bool:
        return self.gate_enabled and ((now - self.last_gate_event_time) <= self.gate_hold_sec)

    def _reset_memory(self):
        if self.last_ranges is not None:
            self.last_ranges = [math.inf] * len(self.last_ranges)
        if self.last_times is not None:
            self.last_times = [-1e9] * len(self.last_times)

    def _make_clear_scan(self, msg: LaserScan) -> LaserScan:
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = max(float(msg.range_min), self.min_range)
        out.range_max = min(float(msg.range_max), self.max_range) if math.isfinite(msg.range_max) else self.max_range
        out.ranges = [math.inf] * len(msg.ranges)
        out.intensities = []
        return out

    def _cb(self, msg: LaserScan):
        now = self._now_sec()
        n = len(msg.ranges)
        signature = (
            n,
            round(msg.angle_min, 6),
            round(msg.angle_increment, 8),
            round(msg.range_min, 4),
            round(msg.range_max, 4),
        )

        if self.last_ranges is None or self.last_times is None or self.last_signature != signature:
            self.last_ranges = [math.inf] * n
            self.last_times = [-1e9] * n
            self.last_signature = signature
            self.get_logger().info(f'Reset scan memory: {n} beams')

        gated = self._gate_active(now)
        if gated:
            self._reset_memory()
            if self.log_gate_transitions and not self.gate_was_active:
                self.get_logger().warn('Scan gate ACTIVE: turret/CV moving; publishing clear scan and resetting memory')
            self.gate_was_active = True
            if self.publish_clear_scan_when_gated:
                self.pub.publish(self._make_clear_scan(msg))
            return

        if self.log_gate_transitions and self.gate_was_active:
            self.get_logger().info('Scan gate inactive: accepting scan again')
        self.gate_was_active = False

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = max(float(msg.range_min), self.min_range)
        out.range_max = min(float(msg.range_max), self.max_range) if math.isfinite(msg.range_max) else self.max_range
        out.intensities = msg.intensities

        out_ranges = []
        for i, r in enumerate(msg.ranges):
            if self._valid(float(r)):
                old = self.last_ranges[i]
                old_age = now - self.last_times[i]
                if self.prefer_closer and old_age <= self.persistence_sec and math.isfinite(old):
                    kept = min(float(r), old)
                else:
                    kept = float(r)
                self.last_ranges[i] = kept
                self.last_times[i] = now
                out_ranges.append(kept)
            else:
                age = now - self.last_times[i]
                if age <= self.persistence_sec and math.isfinite(self.last_ranges[i]):
                    out_ranges.append(self.last_ranges[i])
                else:
                    out_ranges.append(math.inf)

        out.ranges = out_ranges
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanMemoryFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
