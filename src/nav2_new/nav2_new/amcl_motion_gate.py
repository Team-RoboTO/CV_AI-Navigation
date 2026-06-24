"""
amcl_motion_gate.py  (Approach A — safety gate)

Freezes AMCL filter updates when the CHASSIS is stationary, by raising
update_min_d / update_min_a on /amcl, and restores them when the robot moves.

WHY (on top of Approach B):
    Approach B already stops head rotation from triggering spurious AMCL
    updates. This gate is a second, independent safety: when the robot is
    truly parked (vx,vy ~ 0 from the micro), AMCL should not update at all,
    so it can never wander on the symmetric map while sitting still. The
    instant the robot starts moving, full updates resume.

    Unfreeze is immediate (no delay) so AMCL is live as soon as the robot
    moves; freeze has a short hold to avoid flapping on velocity noise.

Requires the AMCL parameters update_min_d / update_min_a to be declared
(they are, in nav2_params.yaml).
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


def _read(data, idx):
    if idx < 0 or idx >= len(data):
        return 0.0, False
    try:
        v = float(data[idx])
    except Exception:
        return 0.0, False
    if not math.isfinite(v):
        return 0.0, False
    return v, True


class AmclMotionGate(Node):
    def __init__(self):
        super().__init__('amcl_motion_gate')

        self.declare_parameter('micro_status_topic', '/micro_status')
        self.declare_parameter('amcl_node_name', 'amcl')
        self.declare_parameter('vx_index', 2)
        self.declare_parameter('vy_index', 3)
        self.declare_parameter('vx_sign', 1.0)
        self.declare_parameter('vy_sign', -1.0)

        # Above this speed -> moving. Set a bit above the rotation-noise floor.
        self.declare_parameter('move_threshold_mps', 0.12)
        # Must be stationary this long before freezing (anti-flap).
        self.declare_parameter('freeze_after_sec', 0.5)
        self.declare_parameter('micro_timeout_sec', 0.30)

        # Values applied when MOVING (normal AMCL) and when FROZEN.
        self.declare_parameter('move_update_min_d', 0.10)
        self.declare_parameter('move_update_min_a', 0.1)
        self.declare_parameter('freeze_update_min_d', 1000.0)
        self.declare_parameter('freeze_update_min_a', 1000.0)

        self.micro_topic = self.get_parameter('micro_status_topic').value
        self.amcl_name = self.get_parameter('amcl_node_name').value
        self.vx_index = int(self.get_parameter('vx_index').value)
        self.vy_index = int(self.get_parameter('vy_index').value)
        self.vx_sign = float(self.get_parameter('vx_sign').value)
        self.vy_sign = float(self.get_parameter('vy_sign').value)
        self.move_th = float(self.get_parameter('move_threshold_mps').value)
        self.freeze_after = float(self.get_parameter('freeze_after_sec').value)
        self.micro_timeout = float(self.get_parameter('micro_timeout_sec').value)
        self.move_d = float(self.get_parameter('move_update_min_d').value)
        self.move_a = float(self.get_parameter('move_update_min_a').value)
        self.freeze_d = float(self.get_parameter('freeze_update_min_d').value)
        self.freeze_a = float(self.get_parameter('freeze_update_min_a').value)

        self.cli = self.create_client(
            SetParameters, f'/{self.amcl_name}/set_parameters')

        self.last_micro_time = None
        self.speed = 0.0
        self.stationary_since = None
        self.state = None  # 'moving' or 'frozen'

        self.sub = self.create_subscription(
            Float32MultiArray, self.micro_topic, self._on_micro, 50)
        self.timer = self.create_timer(0.1, self._tick)

        self.get_logger().info(
            f'amcl_motion_gate active: gating /{self.amcl_name} on {self.micro_topic}; '
            f'move_threshold={self.move_th:.3f} m/s, freeze_after={self.freeze_after:.2f}s'
        )

    def _on_micro(self, msg: Float32MultiArray):
        vx, okx = _read(msg.data, self.vx_index)
        vy, oky = _read(msg.data, self.vy_index)
        if okx and oky:
            self.speed = math.hypot(self.vx_sign * vx, self.vy_sign * vy)
            self.last_micro_time = self.get_clock().now()

    def _fresh(self):
        if self.last_micro_time is None:
            return False
        age = (self.get_clock().now() - self.last_micro_time).nanoseconds * 1e-9
        return age <= self.micro_timeout

    def _tick(self):
        now = self.get_clock().now()

        # If micro is stale, behave as "moving" (keep AMCL fully active) — safer
        # than freezing on missing data.
        moving = (not self._fresh()) or (self.speed > self.move_th)

        desired = 'moving'
        if not moving:
            if self.stationary_since is None:
                self.stationary_since = now
            held = (now - self.stationary_since).nanoseconds * 1e-9
            if held >= self.freeze_after:
                desired = 'frozen'
            else:
                desired = self.state or 'moving'
        else:
            self.stationary_since = None
            desired = 'moving'

        if desired != self.state:
            if desired == 'frozen':
                self._set_amcl(self.freeze_d, self.freeze_a)
            else:
                self._set_amcl(self.move_d, self.move_a)
            self.state = desired

    def _set_amcl(self, dmin, amin):
        if not self.cli.service_is_ready():
            # try to (re)connect silently; will apply on next state change
            self.cli.wait_for_service(timeout_sec=0.0)
            if not self.cli.service_is_ready():
                self.get_logger().warn('amcl set_parameters service not ready yet')
                return
        req = SetParameters.Request()
        p1 = Parameter()
        p1.name = 'update_min_d'
        p1.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=dmin)
        p2 = Parameter()
        p2.name = 'update_min_a'
        p2.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=amin)
        req.parameters = [p1, p2]
        self.cli.call_async(req)
        self.get_logger().info(
            f'AMCL gate -> {self.state if self.state else "init"}: '
            f'update_min_d={dmin}, update_min_a={amin}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AmclMotionGate()
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
