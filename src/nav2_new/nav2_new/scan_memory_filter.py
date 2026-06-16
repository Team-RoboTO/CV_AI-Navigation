#!/usr/bin/env python3
"""
scan_memory_filter - short memory filter for LaserScan obstacles.

Why this exists:
- A Livox/pointcloud_to_laserscan scan can miss a close box for one or two frames.
- Nav2's local costmap then clears the obstacle and DWB may drive into it.
- This node republishes /scan_nav, keeping the last valid range for each beam for
  a short time. It is deliberately short-memory, not a permanent obstacle map.

Use /scan for AMCL and /scan_nav only for the local costmap obstacle layer.
"""

import math
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanMemoryFilter(Node):
    def __init__(self):
        super().__init__('scan_memory_filter')

        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_nav')
        self.declare_parameter('persistence_sec', 5.0)
        self.declare_parameter('min_range', 0.12)
        self.declare_parameter('max_range', 6.0)
        self.declare_parameter('prefer_closer', True)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.persistence_sec = float(self.get_parameter('persistence_sec').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.prefer_closer = bool(self.get_parameter('prefer_closer').value)

        self.last_ranges: Optional[List[float]] = None
        self.last_times: Optional[List[float]] = None
        self.last_signature = None

        self.sub = self.create_subscription(LaserScan, self.input_topic, self._cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(LaserScan, self.output_topic, qos_profile_sensor_data)

        self.get_logger().info(
            f'scan_memory_filter: {self.input_topic} -> {self.output_topic}; '
            f'persistence={self.persistence_sec:.2f}s; range=[{self.min_range:.2f}, {self.max_range:.2f}]'
        )

    def _valid(self, r: float) -> bool:
        return math.isfinite(r) and self.min_range <= r <= self.max_range

    def _cb(self, msg: LaserScan):
        now = self.get_clock().now().nanoseconds * 1e-9
        n = len(msg.ranges)
        signature = (n, round(msg.angle_min, 6), round(msg.angle_increment, 8), round(msg.range_min, 4), round(msg.range_max, 4))

        if self.last_ranges is None or self.last_times is None or self.last_signature != signature:
            self.last_ranges = [math.inf] * n
            self.last_times = [-1e9] * n
            self.last_signature = signature
            self.get_logger().info(f'Reset scan memory: {n} beams')

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = max(msg.range_min, self.min_range)
        out.range_max = min(msg.range_max, self.max_range) if math.isfinite(msg.range_max) else self.max_range
        out.intensities = msg.intensities

        out_ranges = []
        for i, r in enumerate(msg.ranges):
            if self._valid(r):
                # Update memory. If prefer_closer is true, keep closer old obstacle for one cycle
                # unless it is stale; this reduces one-frame clearing flicker.
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
