"""
amcl_scan_stabilizer.py  (Approach B — the core fix)  [v2: high-rate TF]

PROBLEM
    AMCL uses base_link as the robot frame, but in this stack base_link is the
    HEAD (lidar+IMU). When the head rotates to track an enemy, AMCL thinks the
    whole ROBOT rotated and re-localizes -> on a symmetric map it jumps (we saw
    ~1.8 m jumps in map->odom while only the head turned).

SOLUTION
    Give AMCL a frame and a scan that DO NOT rotate with the head:
      - publish odom -> base_stable : same POSITION as base_link, but a FIXED
        (non-rotating) orientation. Head rotation no longer moves AMCL's base
        frame, so it never triggers a spurious rotational update.
      - republish /scan as /scan_amcl with the head yaw ADDED to angle_min/max,
        so the beams still point in the correct WORLD direction.

v2 CHANGE (timing fix)
    The odom->base_stable TF is now published by an INDEPENDENT high-rate timer
    (default 50 Hz) stamped with the CURRENT time, instead of only on each scan
    with the (slower, slightly old) scan timestamp. This keeps the TF cache
    dense and fresh so AMCL's initialpose lookups and message filters stop
    throwing "extrapolation into the past/future" / "earlier than transform
    cache" warnings. The de-rotated scan is still published on each scan.

AMCL CONFIG TO MATCH (nav2_params.yaml):
    base_frame_id: "base_stable"
    scan_topic:    "/scan_amcl"
    update_min_a:  0.1
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped

import tf2_ros


def _yaw_from_quat(x, y, z, w):
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


class AmclScanStabilizer(Node):
    def __init__(self):
        super().__init__('amcl_scan_stabilizer')

        self.declare_parameter('scan_in', '/scan')
        self.declare_parameter('scan_out', '/scan_amcl')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('stable_frame', 'base_stable')
        # How long back in time we tolerate when looking up the head pose.
        self.declare_parameter('tf_timeout_sec', 0.05)
        # Independent TF publish rate for odom->base_stable (keep dense/fresh).
        self.declare_parameter('tf_publish_rate_hz', 20.0)

        self.scan_in = self.get_parameter('scan_in').value
        self.scan_out = self.get_parameter('scan_out').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.stable_frame = self.get_parameter('stable_frame').value
        self.tf_timeout = float(self.get_parameter('tf_timeout_sec').value)
        self.tf_rate = float(self.get_parameter('tf_publish_rate_hz').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Latest head pose (from odom->base_link), reused between updates.
        self.last_yaw = 0.0
        self.last_tx = 0.0
        self.last_ty = 0.0
        self.have_last = False

        self.pub = self.create_publisher(LaserScan, self.scan_out, 10)
        self.sub = self.create_subscription(LaserScan, self.scan_in, self._on_scan, 10)

        # High-rate, current-time TF publisher (independent of scan timing).
        period = 1.0 / max(1.0, self.tf_rate)
        self.tf_timer = self.create_timer(period, self._publish_stable_tf)

        self.get_logger().info(
            f'amcl_scan_stabilizer v2 active: {self.scan_in} -> {self.scan_out}; '
            f'odom->{self.stable_frame} at {self.tf_rate:.0f} Hz (now-stamped); '
            f'head yaw from {self.odom_frame}->{self.base_frame}'
        )

    def _refresh_head_pose(self):
        """Update last_* from the latest odom->base_link transform."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, Time(),
                timeout=Duration(seconds=self.tf_timeout))
        except Exception:
            return False
        q = tf.transform.rotation
        self.last_yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)
        self.last_tx = tf.transform.translation.x
        self.last_ty = tf.transform.translation.y
        self.have_last = True
        return True

    def _publish_stable_tf(self):
        # Always refresh from the freshest head pose available.
        self._refresh_head_pose()
        if not self.have_last:
            return
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()  # current time -> dense cache
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.stable_frame
        tf.transform.translation.x = self.last_tx
        tf.transform.translation.y = self.last_ty
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = 0.0
        tf.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf)

    def _on_scan(self, msg: LaserScan):
        # Make sure we have a head yaw; if not yet, skip this scan.
        if not self.have_last:
            if not self._refresh_head_pose():
                return

        out = LaserScan()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.stable_frame
        out.angle_min = msg.angle_min + self.last_yaw
        out.angle_max = msg.angle_max + self.last_yaw
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = msg.ranges
        out.intensities = msg.intensities
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AmclScanStabilizer()
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
