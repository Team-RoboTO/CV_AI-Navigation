"""
turret_yaw_mux – Multiplex CV yaw and chassis yaw for the turret micro.

PROBLEM:
  The CV pipeline publishes aim commands for the turret (yaw/pitch) when it
  detects an enemy. While the robot navigates, we also need to send *something*
  to the turret — if the CV stays silent, the micro has nothing to follow and
  the barrel can drift.

SOLUTION:
  This node is the single authoritative publisher of /turret/cmd.
  • If CV has published a target within `cv_timeout` seconds → forward it.
  • Otherwise → publish chassis yaw (from odom) and idle pitch, so the barrel
    stays aligned with the front of the robot.

Subscribes:
  /cv/target     geometry_msgs/Vector3Stamped  (x=yaw_rad, y=pitch_rad, z=confidence)
  /cv/detection  std_msgs/Bool                 (optional — enables CV output)
  /odom          nav_msgs/Odometry             (chassis heading)

Publishes:
  /turret/cmd    geometry_msgs/Vector3Stamped  (x=yaw, y=pitch, z=mode)
                 z=0.0  → chassis mode (no target)
                 z=1.0  → CV mode     (actively tracking)
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


def quat_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_angle(a):
    """Wrap angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class TurretYawMux(Node):
    def __init__(self):
        super().__init__('turret_yaw_mux')

        self.declare_parameter('cv_timeout', 0.3)
        self.declare_parameter('cv_target_topic', '/cv/target')
        self.declare_parameter('cv_detection_topic', '/cv/detection')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('turret_cmd_topic', '/turret/cmd')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('cv_yaw_frame', 'chassis')     # "chassis" or "world"
        self.declare_parameter('idle_pitch', 0.0)

        self.cv_timeout = float(self.get_parameter('cv_timeout').value)
        self.cv_frame = self.get_parameter('cv_yaw_frame').value
        self.idle_pitch = float(self.get_parameter('idle_pitch').value)
        publish_rate = float(self.get_parameter('publish_rate').value)

        # State
        self.chassis_yaw = 0.0
        self.cv_yaw = 0.0
        self.cv_pitch = 0.0
        self.cv_confidence = 0.0
        self.last_cv_time = 0.0
        self.detection_enabled = True   # default true — CV timeout alone decides

        # Best-effort QoS for high-rate topics
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(
            Vector3Stamped,
            self.get_parameter('cv_target_topic').value,
            self._on_cv_target, qos)

        self.create_subscription(
            Bool,
            self.get_parameter('cv_detection_topic').value,
            self._on_detection, 10)

        self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self._on_odom, qos)

        self.pub = self.create_publisher(
            Vector3Stamped,
            self.get_parameter('turret_cmd_topic').value,
            10)

        # Publish at fixed rate
        self.create_timer(1.0 / publish_rate, self._publish)

        self.get_logger().info(
            f'Turret mux started. cv_timeout={self.cv_timeout}s  '
            f'cv_frame={self.cv_frame}  rate={publish_rate}Hz')

    # ─── Callbacks ───────────────────────────────────────────────────────────

    def _on_cv_target(self, msg: Vector3Stamped):
        self.cv_yaw = float(msg.vector.x)
        self.cv_pitch = float(msg.vector.y)
        self.cv_confidence = float(msg.vector.z)
        self.last_cv_time = time.monotonic()

    def _on_detection(self, msg: Bool):
        self.detection_enabled = bool(msg.data)

    def _on_odom(self, msg: Odometry):
        self.chassis_yaw = quat_to_yaw(msg.pose.pose.orientation)

    # ─── Main publish loop ───────────────────────────────────────────────────

    def _publish(self):
        now = time.monotonic()
        cv_fresh = (now - self.last_cv_time) < self.cv_timeout
        use_cv = cv_fresh and self.detection_enabled

        out = Vector3Stamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'

        if use_cv:
            # Forward CV aim
            if self.cv_frame == 'chassis':
                # CV yaw is already relative to robot front → pass as-is
                out.vector.x = self.cv_yaw
            else:
                # CV yaw is absolute in world frame → convert to chassis-relative
                out.vector.x = wrap_angle(self.cv_yaw - self.chassis_yaw)
            out.vector.y = self.cv_pitch
            out.vector.z = 1.0   # mode = CV
        else:
            # No detection → align barrel with chassis front
            # In chassis-relative coords, that's yaw=0.
            # If the micro expects absolute world yaw, swap out.vector.x for chassis_yaw.
            out.vector.x = 0.0
            out.vector.y = self.idle_pitch
            out.vector.z = 0.0   # mode = chassis

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = TurretYawMux()
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
