"""
chassis_odom_publisher.py

Publishes a velocity-only nav_msgs/Odometry built from the chassis vx/vy that
the micro reports on /micro_status, ROTATED into the head (base_link) frame
using the cumulative head yaw on data[0].

WHY velocity-only (no position):
    A previous version of this stack integrated vx/vy into x/y and drifted
    badly on diagonals because the velocities were used in the WRONG frame
    (chassis vs head). See the note at the top of tf_frame_relay.py.
    So here we NEVER integrate position. We only output a clean twist
    (vx, vy in base_link) to be fused as a soft velocity constraint by an
    EKF (robot_localization). FAST-LIO stays the source of absolute pose.

FRAME MATH:
    vx/vy from the micro are expressed in the spinning CHASSIS frame.
    base_link is the HEAD frame. data[0] is the head yaw (cumulative).
    To express the chassis velocity in the head frame we rotate it by the
    chassis->head angle. The exact sign depends on your firmware convention,
    so 'head_yaw_sign' lets you flip it. Validate empirically (see README):
        - spin chassis in place, no translation -> output vx,vy ~ 0
        - drive straight along the barrel direction -> vx > 0, vy ~ 0

NOISE:
    The micro reports ~0.06-0.08 m/s of spurious velocity while only rotating.
    'deadband_mps' zeroes anything below a threshold so that pure rotation
    does not inject fake translation into the EKF.

Publishes:
    /odom_wheel   nav_msgs/Odometry   (twist only; pose left at origin, huge pose covariance)
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry


def _read(data, idx, default=0.0):
    if idx < 0 or idx >= len(data):
        return default, False
    try:
        v = float(data[idx])
    except Exception:
        return default, False
    if not math.isfinite(v):
        return default, False
    return v, True


class ChassisOdomPublisher(Node):
    def __init__(self):
        super().__init__('chassis_odom_publisher')

        self.declare_parameter('micro_status_topic', '/micro_status')
        self.declare_parameter('output_topic', '/odom_wheel')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        # micro_status indices (match tf_frame_relay defaults)
        self.declare_parameter('head_yaw_index', 0)
        self.declare_parameter('vx_index', 2)
        self.declare_parameter('vy_index', 3)

        # sign conventions
        self.declare_parameter('head_yaw_sign', 1.0)   # flip to -1.0 if rotation is mirrored
        self.declare_parameter('vx_sign', 1.0)
        self.declare_parameter('vy_sign', -1.0)        # matches tf_frame_relay micro_vy_sign

        # noise handling
        self.declare_parameter('deadband_mps', 0.10)   # kill spurious velocity from pure rotation
        self.declare_parameter('micro_timeout_sec', 0.30)

        # covariance for the twist (vx, vy). Larger = EKF trusts wheels less.
        # Tune up if the wheels are noisy, down if you want more correction.
        self.declare_parameter('vxy_variance', 0.05)   # (m/s)^2

        # If micro is stale, publish zero velocity with very high covariance
        # so the EKF effectively ignores it instead of coasting on old data.
        self.declare_parameter('publish_zero_on_timeout', True)

        self.micro_status_topic = self.get_parameter('micro_status_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.head_yaw_index = int(self.get_parameter('head_yaw_index').value)
        self.vx_index = int(self.get_parameter('vx_index').value)
        self.vy_index = int(self.get_parameter('vy_index').value)

        self.head_yaw_sign = float(self.get_parameter('head_yaw_sign').value)
        self.vx_sign = float(self.get_parameter('vx_sign').value)
        self.vy_sign = float(self.get_parameter('vy_sign').value)

        self.deadband_mps = float(self.get_parameter('deadband_mps').value)
        self.micro_timeout_sec = float(self.get_parameter('micro_timeout_sec').value)
        self.vxy_variance = float(self.get_parameter('vxy_variance').value)
        self.publish_zero_on_timeout = bool(self.get_parameter('publish_zero_on_timeout').value)

        self.last_micro_time = None
        self.vx_head = 0.0
        self.vy_head = 0.0

        self.pub = self.create_publisher(Odometry, self.output_topic, 50)
        self.sub = self.create_subscription(
            Float32MultiArray, self.micro_status_topic, self._on_micro, 50)

        # Publish at a steady rate so the EKF always has fresh twist.
        self.timer = self.create_timer(0.02, self._on_timer)  # 50 Hz

        self.get_logger().info(
            f'chassis_odom_publisher active: {self.micro_status_topic} -> {self.output_topic}; '
            f'head_yaw[{self.head_yaw_index}]*{self.head_yaw_sign}, '
            f'vx[{self.vx_index}]*{self.vx_sign}, vy[{self.vy_index}]*{self.vy_sign}; '
            f'deadband={self.deadband_mps:.3f} m/s; frame={self.base_frame}'
        )

    def _on_micro(self, msg: Float32MultiArray):
        data = msg.data
        yaw, oky = _read(data, self.head_yaw_index, 0.0)
        vx_c, okvx = _read(data, self.vx_index, 0.0)
        vy_c, okvy = _read(data, self.vy_index, 0.0)
        if not (okvx and okvy):
            return

        vx_c *= self.vx_sign
        vy_c *= self.vy_sign

        # Deadband BEFORE rotation so pure-rotation noise is removed.
        if abs(vx_c) < self.deadband_mps:
            vx_c = 0.0
        if abs(vy_c) < self.deadband_mps:
            vy_c = 0.0

        # Rotate chassis velocity into the head (base_link) frame.
        # head is rotated by 'yaw' relative to chassis -> express chassis
        # vector in head frame with R(-yaw). head_yaw_sign folds the
        # firmware convention; flip it if validation shows mirrored motion.
        ang = -self.head_yaw_sign * yaw
        c = math.cos(ang)
        s = math.sin(ang)
        self.vx_head = vx_c * c - vy_c * s
        self.vy_head = vx_c * s + vy_c * c

        self.last_micro_time = self.get_clock().now()

    def _fresh(self) -> bool:
        if self.last_micro_time is None:
            return False
        age = (self.get_clock().now() - self.last_micro_time).nanoseconds * 1e-9
        return age <= self.micro_timeout_sec

    def _on_timer(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame

        fresh = self._fresh()

        if fresh:
            msg.twist.twist.linear.x = self.vx_head
            msg.twist.twist.linear.y = self.vy_head
            var = self.vxy_variance
        else:
            if not self.publish_zero_on_timeout:
                return
            msg.twist.twist.linear.x = 0.0
            msg.twist.twist.linear.y = 0.0
            var = 1e6  # tell the EKF to ignore this

        # Pose is meaningless here (velocity-only source) -> huge pose covariance.
        # Twist covariance: only vx, vy meaningful.
        big = 1e9
        cov = [0.0] * 36
        # pose covariance diagonal = huge (not used)
        for i in (0, 7, 14, 21, 28, 35):
            cov[i] = big
        msg.pose.covariance = list(cov)

        tcov = [0.0] * 36
        for i in (0, 7, 14, 21, 28, 35):
            tcov[i] = big
        tcov[0] = var   # vx
        tcov[7] = var   # vy
        msg.twist.covariance = list(tcov)

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ChassisOdomPublisher()
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
