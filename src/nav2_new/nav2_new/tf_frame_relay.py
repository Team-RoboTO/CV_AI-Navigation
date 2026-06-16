"""
tf_frame_relay - Relay FAST_LIO TF into a planar, head-referenced ROS frame.

FAST_LIO publishes the 3D pose of the LiDAR/IMU rigid body:

    camera_init -> body

This package intentionally uses a head-referenced navigation convention:

    odom -> base_link

where base_link is the planar navigation frame associated with the head/LiDAR
rigid body, not the chassis center.

Important for SLAM Toolbox:
- slam_toolbox is 2D, so odom -> base_link must be planar.
- Do not publish FAST_LIO's raw 3D roll/pitch/z as base_link.
- This relay converts the FAST_LIO orientation into yaw-only and forces z=0.
- The Livox mounting flip remains a static TF: base_link -> livox_frame.
"""

import copy
import math
from typing import Tuple

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


def _normalize_quaternion(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n <= 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    return x / n, y / n, z / n, w / n


def _multiply_quaternions(
    q1: Tuple[float, float, float, float],
    q2: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    """Return q1 * q2 for quaternions in xyzw order."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return _normalize_quaternion((
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ))


def _yaw_from_quaternion(q: Tuple[float, float, float, float]) -> float:
    """Extract Z yaw from an xyzw quaternion."""
    x, y, z, w = _normalize_quaternion(q)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    """Build a roll=0, pitch=0, yaw-only quaternion in xyzw order."""
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


class TFFrameRelay(Node):
    def __init__(self):
        super().__init__('tf_frame_relay')

        self.declare_parameter('input_topic', '/tf_fastlio')
        self.declare_parameter('source_parent_frame', 'camera_init')
        self.declare_parameter('source_child_frame', 'body')
        self.declare_parameter('target_parent_frame', 'odom')
        self.declare_parameter('target_child_frame', 'base_link')

        # Constant transform from FAST_LIO body frame to the ROS-compatible
        # head-referenced base_link frame, expressed as q_body_base in xyzw.
        # Default = identity. In deepglint-style inverted-driver mode, the
        # modified Livox driver already applies roll=180 to BOTH cloud and IMU,
        # so FAST_LIO body is already the head/base frame.
        self.declare_parameter('body_to_base_qx', 0.0)
        self.declare_parameter('body_to_base_qy', 0.0)
        self.declare_parameter('body_to_base_qz', 0.0)
        self.declare_parameter('body_to_base_qw', 1.0)

        # Critical for 2D SLAM: publish a yaw-only odom -> base_link.
        self.declare_parameter('planarize', True)
        self.declare_parameter('force_z_zero', True)

        # Set True when physical clockwise head rotation appears counterclockwise
        # in RViz. This fixes the yaw convention after planarization.
        self.declare_parameter('invert_yaw', False)

        # Translation sign corrections. ROS convention: +X forward, +Y left.
        # Keep True if physical forward/left appear as negative X/Y in RViz.
        self.declare_parameter('invert_x', False)
        self.declare_parameter('invert_y', False)

        input_topic = self.get_parameter('input_topic').value
        self.source_parent_frame = self.get_parameter('source_parent_frame').value
        self.source_child_frame = self.get_parameter('source_child_frame').value
        self.target_parent_frame = self.get_parameter('target_parent_frame').value
        self.target_child_frame = self.get_parameter('target_child_frame').value
        self.body_to_base_q = _normalize_quaternion((
            float(self.get_parameter('body_to_base_qx').value),
            float(self.get_parameter('body_to_base_qy').value),
            float(self.get_parameter('body_to_base_qz').value),
            float(self.get_parameter('body_to_base_qw').value),
        ))
        self.planarize = bool(self.get_parameter('planarize').value)
        self.force_z_zero = bool(self.get_parameter('force_z_zero').value)
        self.invert_yaw = bool(self.get_parameter('invert_yaw').value)
        self.invert_x = bool(self.get_parameter('invert_x').value)
        self.invert_y = bool(self.get_parameter('invert_y').value)

        self.sub = self.create_subscription(TFMessage, input_topic, self._on_tf, 100)
        self.pub = self.create_publisher(TFMessage, '/tf', 100)

        self.get_logger().info(
            f'TF relay active: {self.source_parent_frame}->{self.source_child_frame} '
            f'=> {self.target_parent_frame}->{self.target_child_frame}; '
            f'body_to_base_q(xyzw)={self.body_to_base_q}; '
            f'planarize={self.planarize}; force_z_zero={self.force_z_zero}; '
            f'invert_yaw={self.invert_yaw}; invert_x={self.invert_x}; invert_y={self.invert_y}'
        )

    def _on_tf(self, msg: TFMessage):
        out = TFMessage()

        for t in msg.transforms:
            if (
                t.header.frame_id != self.source_parent_frame
                or t.child_frame_id != self.source_child_frame
            ):
                continue

            t2 = copy.deepcopy(t)
            t2.header.frame_id = self.target_parent_frame
            t2.child_frame_id = self.target_child_frame

            q = t.transform.rotation
            q_body = (q.x, q.y, q.z, q.w)
            q_base_3d = _multiply_quaternions(q_body, self.body_to_base_q)

            if self.planarize:
                yaw = _yaw_from_quaternion(q_base_3d)
                if self.invert_yaw:
                    yaw = -yaw
                q_base = _quaternion_from_yaw(yaw)
            else:
                q_base = q_base_3d

            t2.transform.rotation.x = q_base[0]
            t2.transform.rotation.y = q_base[1]
            t2.transform.rotation.z = q_base[2]
            t2.transform.rotation.w = q_base[3]

            # Convert FAST_LIO planar translation signs to ROS planar conventions.
            # ROS convention: +X is forward, +Y is left.
            if self.invert_x:
                t2.transform.translation.x = -t2.transform.translation.x
            if self.invert_y:
                t2.transform.translation.y = -t2.transform.translation.y

            # The 2D navigation frame must not carry FAST_LIO vertical motion.
            if self.force_z_zero:
                t2.transform.translation.z = 0.0

            out.transforms.append(t2)

        if out.transforms:
            self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TFFrameRelay()
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
