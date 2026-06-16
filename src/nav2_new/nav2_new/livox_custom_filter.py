#!/usr/bin/env python3
"""Filter Livox CustomMsg before FAST-LIO.

This node keeps the original /livox/lidar topic untouched and publishes a
filtered CustomMsg for FAST-LIO on /livox/lidar_filtered.

It removes near self-points, far points, floor/ceiling by z in the Livox/head
frame, and optional low-confidence Livox tagged points.
"""

import copy
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

try:
    from livox_ros_driver2.msg import CustomMsg
except Exception as exc:  # pragma: no cover
    CustomMsg = None
    _IMPORT_ERROR = exc
else:
    _IMPORT_ERROR = None


class LivoxCustomFilter(Node):
    def __init__(self):
        super().__init__('livox_custom_filter')

        if CustomMsg is None:
            raise RuntimeError(f'Could not import livox_ros_driver2.msg.CustomMsg: {_IMPORT_ERROR}')

        self.declare_parameter('input_topic', '/livox/lidar')
        self.declare_parameter('output_topic', '/livox/lidar_filtered')

        # Distances are in meters from the LiDAR/head origin.
        self.declare_parameter('min_range', 0.45)
        self.declare_parameter('max_range', 6.0)

        # Height filter in the corrected Livox/head frame. With a LiDAR about
        # 0.8 m above ground, floor is around z=-0.8. Keeping z>-0.55 removes
        # most floor returns while preserving low obstacles/walls.
        self.declare_parameter('min_z', -0.55)
        self.declare_parameter('max_z', 1.50)

        # Set true to remove Livox points whose tag is non-zero. In some Livox
        # drivers tag semantics vary, so this is off by default.
        self.declare_parameter('drop_nonzero_tags', False)

        # Useful if you want to verify how aggressive the filter is.
        self.declare_parameter('log_every_n_clouds', 30)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.min_z = float(self.get_parameter('min_z').value)
        self.max_z = float(self.get_parameter('max_z').value)
        self.drop_nonzero_tags = bool(self.get_parameter('drop_nonzero_tags').value)
        self.log_every_n = int(self.get_parameter('log_every_n_clouds').value)
        self._count = 0

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        # Publisher offered as RELIABLE is more compatible with reliable FAST-LIO
        # subscribers while still working with best-effort subscribers.
        pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.sub = self.create_subscription(CustomMsg, self.input_topic, self._on_cloud, qos)
        self.pub = self.create_publisher(CustomMsg, self.output_topic, pub_qos)

        self.get_logger().info(
            'LivoxCustomFilter active: '
            f'{self.input_topic} -> {self.output_topic}; '
            f'range=[{self.min_range:.2f}, {self.max_range:.2f}] m; '
            f'z=[{self.min_z:.2f}, {self.max_z:.2f}] m; '
            f'drop_nonzero_tags={self.drop_nonzero_tags}'
        )

    def _on_cloud(self, msg):
        min_r2 = self.min_range * self.min_range
        max_r2 = self.max_range * self.max_range

        kept = []
        for p in msg.points:
            x = float(p.x)
            y = float(p.y)
            z = float(p.z)
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            if z < self.min_z or z > self.max_z:
                continue
            r2 = x * x + y * y + z * z
            if r2 < min_r2 or r2 > max_r2:
                continue
            if self.drop_nonzero_tags and hasattr(p, 'tag') and int(p.tag) != 0:
                continue
            kept.append(p)

        out = copy.deepcopy(msg)
        out.points = kept
        if hasattr(out, 'point_num'):
            out.point_num = len(kept)

        self.pub.publish(out)

        self._count += 1
        if self.log_every_n > 0 and self._count % self.log_every_n == 0:
            total = len(msg.points)
            pct = (100.0 * len(kept) / total) if total else 0.0
            self.get_logger().info(f'Filtered cloud: kept {len(kept)}/{total} points ({pct:.1f}%)')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LivoxCustomFilter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
