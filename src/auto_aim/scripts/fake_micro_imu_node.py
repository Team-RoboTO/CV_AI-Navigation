#!/usr/bin/env python3
"""Static testing-only /micro_imu publisher.

This node intentionally does not estimate yaw/pitch from video frames. It is a
bench/debug pose source only: by default it publishes yaw=0 and pitch=0 forever.
Use the real microcontroller feedback on the robot.
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class FakeMicroImuNode(Node):
    def __init__(self):
        super().__init__("fake_micro_imu")

        self.declare_parameter("mode", "static")
        self.declare_parameter("imu_topic", "/micro_imu")
        self.declare_parameter("rate_hz", 120.0)
        self.declare_parameter("yaw_rad", 0.0)
        self.declare_parameter("pitch_rad", 0.0)
        self.declare_parameter("publish_log_period_s", 5.0)

        mode = str(self.get_parameter("mode").value)
        if mode != "static":
            raise RuntimeError(
                "fake_micro_imu only supports mode='static'. "
                "Video-derived fake IMU was removed because it creates hidden TF drift."
            )

        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.rate_hz = max(1.0, float(self.get_parameter("rate_hz").value))
        self.yaw = float(self.get_parameter("yaw_rad").value)
        self.pitch = float(self.get_parameter("pitch_rad").value)
        self.publish_log_period = float(self.get_parameter("publish_log_period_s").value)

        if not (math.isfinite(self.yaw) and math.isfinite(self.pitch)):
            raise RuntimeError("fake_micro_imu yaw_rad and pitch_rad must be finite")

        self.pub = self.create_publisher(Float32MultiArray, self.imu_topic, 10)
        self.last_log_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.rate_hz, self._timer_cb)
        self.get_logger().info(
            "Static fake micro IMU active: yaw=%.3f pitch=%.3f topic=%s rate=%.1f Hz. "
            "Testing only; use real /micro_imu for live firing."
            % (self.yaw, self.pitch, self.imu_topic, self.rate_hz)
        )

    def _timer_cb(self):
        msg = Float32MultiArray()
        msg.data = [float(self.yaw), float(self.pitch)]
        self.pub.publish(msg)

        if self.publish_log_period > 0.0:
            now = self.get_clock().now()
            age = (now - self.last_log_time).nanoseconds * 1e-9
            if age >= self.publish_log_period:
                self.last_log_time = now
                self.get_logger().info(
                    "static fake imu yaw=%.3f pitch=%.3f" % (self.yaw, self.pitch)
                )


def main():
    rclpy.init()
    node = FakeMicroImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
