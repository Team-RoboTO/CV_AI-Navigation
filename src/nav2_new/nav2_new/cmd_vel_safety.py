"""
cmd_vel_safety – Safety layer between Nav2 and the motor driver.

Sits between Nav2's /cmd_vel output and the motor controller. Purpose:

  1. WATCHDOG: if Nav2 stops publishing for longer than `watchdog_timeout`,
     forward zero velocity so the robot doesn't keep coasting on the last
     command.

  2. EMERGENCY STOP: if /emergency_stop (Bool) is True, forward zero
     velocity regardless of what Nav2 says. Great for a big-red-button
     or for pausing the robot via game state (e.g. match timeout).

  3. VELOCITY CLAMPING: enforce hard max limits, independent of Nav2's own
     limits. Belt-and-braces safety.

Topology:
   /cmd_vel_nav  →  cmd_vel_safety  →  /cmd_vel  →  motor driver

In nav2_params.yaml, remap Nav2's controller output to /cmd_vel_nav:
   controller_server:
     ros__parameters:
       controller_plugins: ["FollowPath"]
       # ... add a remapping in the launch file, OR let this node subscribe
       # to /cmd_vel_smoothed and republish on /cmd_vel_safe

The default wiring subscribes to /cmd_vel_nav and publishes on /cmd_vel.
If you don't want to remap Nav2, just change the input topic below to
match Nav2's actual output.
"""
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class CmdVelSafety(Node):
    def __init__(self):
        super().__init__('cmd_vel_safety')

        # Default wiring: watchdog/clamp BEFORE Nav2's output reaches the
        # motor-facing topic. If you don't want this node at all, just don't
        # launch it and the Nav2 remapping publishes directly to /cmd_vel_NAV.
        #
        # If you do use it, place it BETWEEN Nav2 and /cmd_vel_NAV:
        #   • Remove the Nav2 remapping (set cmd_vel_topic:=/cmd_vel_nav_raw)
        #   • This node subscribes to /cmd_vel_nav_raw and publishes to /cmd_vel_NAV
        self.declare_parameter('input_topic', '/cmd_vel_nav_raw')
        self.declare_parameter('output_topic', '/cmd_vel_NAV')
        self.declare_parameter('estop_topic', '/emergency_stop')
        self.declare_parameter('watchdog_timeout', 0.5)     # seconds
        self.declare_parameter('max_linear_x', 1.5)         # m/s (hard clamp)
        self.declare_parameter('max_angular_z', 3.0)        # rad/s (hard clamp)
        self.declare_parameter('publish_rate', 30.0)        # Hz

        self.watchdog_timeout = float(self.get_parameter('watchdog_timeout').value)
        self.max_lin = float(self.get_parameter('max_linear_x').value)
        self.max_ang = float(self.get_parameter('max_angular_z').value)
        publish_rate = float(self.get_parameter('publish_rate').value)

        # State
        self.last_cmd = Twist()
        self.last_cmd_time = 0.0
        self.estop = False
        self._last_was_zero = False
        self._warned_watchdog = False

        # Subscribers
        self.create_subscription(
            Twist, self.get_parameter('input_topic').value,
            self._on_cmd, 10)
        self.create_subscription(
            Bool, self.get_parameter('estop_topic').value,
            self._on_estop, 10)

        # Publisher
        self.pub = self.create_publisher(
            Twist, self.get_parameter('output_topic').value, 10)

        # Periodic publisher
        self.create_timer(1.0 / publish_rate, self._publish)

        self.get_logger().info(
            f'cmd_vel_safety: {self.get_parameter("input_topic").value}'
            f' → {self.get_parameter("output_topic").value}  '
            f'watchdog={self.watchdog_timeout}s  max_lin={self.max_lin}  max_ang={self.max_ang}')

    def _on_cmd(self, msg: Twist):
        self.last_cmd = msg
        self.last_cmd_time = time.monotonic()
        self._warned_watchdog = False

    def _on_estop(self, msg: Bool):
        was = self.estop
        self.estop = bool(msg.data)
        if self.estop and not was:
            self.get_logger().warn('EMERGENCY STOP engaged')
        elif not self.estop and was:
            self.get_logger().info('Emergency stop released')

    def _publish(self):
        out = Twist()

        # Reason to stop
        now = time.monotonic()
        stale = (now - self.last_cmd_time) > self.watchdog_timeout

        if self.estop:
            pass  # keep out as zero
        elif stale:
            if not self._warned_watchdog and self.last_cmd_time > 0.0:
                self.get_logger().warn(
                    f'Watchdog: no cmd_vel for >{self.watchdog_timeout}s — zeroing output')
                self._warned_watchdog = True
        else:
            # Clamp
            out.linear.x = max(-self.max_lin, min(self.max_lin, self.last_cmd.linear.x))
            out.linear.y = max(-self.max_lin, min(self.max_lin, self.last_cmd.linear.y))
            out.angular.z = max(-self.max_ang, min(self.max_ang, self.last_cmd.angular.z))

        # Avoid spamming zeros on the bus if we were already zero
        is_zero = (out.linear.x == 0.0 and out.linear.y == 0.0 and out.angular.z == 0.0)
        if is_zero and self._last_was_zero:
            # still publish at ~5 Hz as a heartbeat
            if int(now * 5) % 5 != 0:
                return
        self._last_was_zero = is_zero
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CmdVelSafety()
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
