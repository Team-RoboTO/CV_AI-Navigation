"""
tf_frame_relay - Relay FAST_LIO TF into a planar, head-referenced ROS frame,
with a micro-odometry stationary translation lock.

This node intentionally keeps the part that worked in the original package:
FAST-LIO's camera_init -> body pose is the primary odom -> base_link pose.

The only added behaviour is:
- when the micro says the chassis is not translating, freeze x/y exactly;
- still publish yaw from FAST-LIO, so head/barrel rotation is visible;
- keep a translation correction offset so releasing the lock does not jump.

This avoids the failure mode of the later experimental packages where x/y was
integrated from vx/vy in the wrong reference frame, causing diagonal motion.
"""

import copy
import math
from typing import Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
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


def _read_array(data, idx, default=0.0):
    if idx < 0 or idx >= len(data):
        return default, False
    try:
        v = float(data[idx])
    except Exception:
        return default, False
    if not math.isfinite(v):
        return default, False
    return v, True


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
        self.declare_parameter('body_to_base_qx', 0.0)
        self.declare_parameter('body_to_base_qy', 0.0)
        self.declare_parameter('body_to_base_qz', 0.0)
        self.declare_parameter('body_to_base_qw', 1.0)

        self.declare_parameter('planarize', True)
        self.declare_parameter('force_z_zero', True)
        self.declare_parameter('invert_yaw', False)
        self.declare_parameter('invert_x', False)
        self.declare_parameter('invert_y', False)

        # Micro gate. Current firmware layout:
        # data[0] = gimbal yaw (bounded; not used here)
        # data[1] = pitch (ignored)
        # data[2] = vx
        # data[3] = vy
        self.declare_parameter('use_micro_stationary_gate', True)
        self.declare_parameter('micro_status_topic', '/micro_status')
        self.declare_parameter('micro_vx_index', 2)
        self.declare_parameter('micro_vy_index', 3)
        self.declare_parameter('micro_vx_sign', 1.0)
        self.declare_parameter('micro_vy_sign', -1.0)
        self.declare_parameter('stationary_vxy_threshold', 0.12)
        self.declare_parameter('micro_timeout_sec', 0.30)
        self.declare_parameter('log_stationary_gate', True)

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

        self.use_micro_stationary_gate = bool(self.get_parameter('use_micro_stationary_gate').value)
        self.micro_status_topic = self.get_parameter('micro_status_topic').value
        self.micro_vx_index = int(self.get_parameter('micro_vx_index').value)
        self.micro_vy_index = int(self.get_parameter('micro_vy_index').value)
        self.micro_vx_sign = float(self.get_parameter('micro_vx_sign').value)
        self.micro_vy_sign = float(self.get_parameter('micro_vy_sign').value)
        self.stationary_vxy_threshold = float(self.get_parameter('stationary_vxy_threshold').value)
        self.micro_timeout_sec = float(self.get_parameter('micro_timeout_sec').value)
        self.log_stationary_gate = bool(self.get_parameter('log_stationary_gate').value)

        self.last_micro_time = None
        self.micro_vx = 0.0
        self.micro_vy = 0.0
        self.has_last_output = False
        self.last_out_x = 0.0
        self.last_out_y = 0.0
        self.corr_x = 0.0
        self.corr_y = 0.0
        self.hold_x = 0.0
        self.hold_y = 0.0
        self.holding_xy = False
        self.last_gate_state = None

        self.sub = self.create_subscription(TFMessage, input_topic, self._on_tf, 100)
        self.pub = self.create_publisher(TFMessage, '/tf', 100)
        if self.use_micro_stationary_gate:
            self.micro_sub = self.create_subscription(
                Float32MultiArray, self.micro_status_topic, self._on_micro_status, 50)
        else:
            self.micro_sub = None

        self.get_logger().info(
            f'TF relay V21 active: FAST-LIO primary {self.source_parent_frame}->{self.source_child_frame} '
            f'=> {self.target_parent_frame}->{self.target_child_frame}; '
            f'micro stationary gate={self.use_micro_stationary_gate} topic={self.micro_status_topic} '
            f'vx[{self.micro_vx_index}]*{self.micro_vx_sign} vy[{self.micro_vy_index}]*{self.micro_vy_sign} '
            f'th={self.stationary_vxy_threshold:.3f}; '
            f'planarize={self.planarize}; force_z_zero={self.force_z_zero}; '
            f'invert_yaw={self.invert_yaw}; invert_x={self.invert_x}; invert_y={self.invert_y}'
        )

    def _on_micro_status(self, msg: Float32MultiArray):
        vx, okx = _read_array(msg.data, self.micro_vx_index, 0.0)
        vy, oky = _read_array(msg.data, self.micro_vy_index, 0.0)
        if okx:
            self.micro_vx = self.micro_vx_sign * vx
        if oky:
            self.micro_vy = self.micro_vy_sign * vy
        self.last_micro_time = self.get_clock().now()

    def _micro_fresh(self) -> bool:
        if self.last_micro_time is None:
            return False
        return (self.get_clock().now() - self.last_micro_time).nanoseconds * 1e-9 <= self.micro_timeout_sec

    def _stationary_by_micro(self) -> bool:
        if not self.use_micro_stationary_gate:
            return False
        if not self._micro_fresh():
            return False
        return math.hypot(self.micro_vx, self.micro_vy) <= self.stationary_vxy_threshold

    def _apply_stationary_translation_lock(self, raw_x: float, raw_y: float) -> Tuple[float, float]:
        stationary = self._stationary_by_micro()
        state = 'stationary_lock' if stationary else ('micro_timeout' if self.use_micro_stationary_gate and not self._micro_fresh() else 'moving_lio_primary')
        if state != self.last_gate_state and self.log_stationary_gate:
            self.last_gate_state = state
            self.get_logger().info(
                f'TF gate state: {state}; vx={self.micro_vx:.3f} vy={self.micro_vy:.3f} '
                f'raw=({raw_x:.3f},{raw_y:.3f}) corr=({self.corr_x:.3f},{self.corr_y:.3f})')

        if stationary:
            if not self.holding_xy:
                if self.has_last_output:
                    self.hold_x = self.last_out_x
                    self.hold_y = self.last_out_y
                else:
                    self.hold_x = raw_x + self.corr_x
                    self.hold_y = raw_y + self.corr_y
                self.holding_xy = True
            # Track raw LIO drift with the correction offset so release has no jump.
            self.corr_x = self.hold_x - raw_x
            self.corr_y = self.hold_y - raw_y
            return self.hold_x, self.hold_y

        self.holding_xy = False
        return raw_x + self.corr_x, raw_y + self.corr_y

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

            raw_x = t.transform.translation.x
            raw_y = t.transform.translation.y
            if self.invert_x:
                raw_x = -raw_x
            if self.invert_y:
                raw_y = -raw_y

            out_x, out_y = self._apply_stationary_translation_lock(raw_x, raw_y)
            t2.transform.translation.x = out_x
            t2.transform.translation.y = out_y
            self.last_out_x = out_x
            self.last_out_y = out_y
            self.has_last_output = True

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
