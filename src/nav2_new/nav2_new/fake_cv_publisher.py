"""
fake_cv_publisher – Simulate the CV pipeline for testing turret_yaw_mux.

Publishes synthetic /cv/target messages so you can verify the mux behavior
without the real vision pipeline running.

Three modes (parameter `pattern`):
  • static    — constant yaw/pitch
  • sweep     — yaw sweeps ±sweep_amplitude sinusoidally
  • burst     — bursts of fake detections with gaps (tests mux timeout)

Parameters:
  rate              publish rate in Hz       (default 30)
  pattern           static | sweep | burst   (default sweep)
  yaw               yaw value for static (rad)
  pitch             pitch for all modes (rad)
  sweep_amplitude   ± yaw range for sweep (rad)
  sweep_period      sweep period (s)
  burst_on          on-duration for burst mode (s)
  burst_off         off-duration for burst mode (s)

Usage:
  ros2 run nav2_new fake_cv_publisher
  ros2 run nav2_new fake_cv_publisher --ros-args -p pattern:=burst
"""
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped


class FakeCVPublisher(Node):
    def __init__(self):
        super().__init__('fake_cv_publisher')

        self.declare_parameter('rate', 30.0)
        self.declare_parameter('pattern', 'sweep')
        self.declare_parameter('yaw', 0.3)
        self.declare_parameter('pitch', 0.05)
        self.declare_parameter('sweep_amplitude', 0.5)   # ±0.5 rad ≈ ±28°
        self.declare_parameter('sweep_period', 4.0)
        self.declare_parameter('burst_on', 1.5)
        self.declare_parameter('burst_off', 1.0)
        self.declare_parameter('topic', '/cv/target')

        self.pattern = self.get_parameter('pattern').value
        self.yaw_static = float(self.get_parameter('yaw').value)
        self.pitch = float(self.get_parameter('pitch').value)
        self.amp = float(self.get_parameter('sweep_amplitude').value)
        self.period = float(self.get_parameter('sweep_period').value)
        self.on = float(self.get_parameter('burst_on').value)
        self.off = float(self.get_parameter('burst_off').value)
        rate = float(self.get_parameter('rate').value)

        self.pub = self.create_publisher(
            Vector3Stamped, self.get_parameter('topic').value, 10)

        self.t0 = time.monotonic()
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f'fake_cv_publisher started (pattern={self.pattern}, '
            f'rate={rate}Hz, topic={self.get_parameter("topic").value})')

    def _tick(self):
        t = time.monotonic() - self.t0

        if self.pattern == 'static':
            yaw = self.yaw_static
            publish = True

        elif self.pattern == 'sweep':
            yaw = self.amp * math.sin(2.0 * math.pi * t / self.period)
            publish = True

        elif self.pattern == 'burst':
            cycle = self.on + self.off
            phase = t % cycle
            publish = phase < self.on
            yaw = self.amp * math.sin(2.0 * math.pi * t / self.period)

        else:
            self.get_logger().warn_once(f'Unknown pattern: {self.pattern}')
            return

        if not publish:
            return

        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.vector.x = yaw
        msg.vector.y = self.pitch
        msg.vector.z = 0.95   # fake confidence
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = FakeCVPublisher()
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
