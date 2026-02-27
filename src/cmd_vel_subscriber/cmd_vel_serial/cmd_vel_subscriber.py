import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import serial
import time
import threading
import struct
import numpy as np
from rclpy.qos import qos_profile_system_default


PACKET_SIZE = 20  # 5 floats * 4 bytes


def format_float(array):
    bts = []
    for num in array:
        for byte in struct.pack('<f', num):
            bts.append(byte)
    return bytearray(bts)


class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_vel_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            qos_profile_system_default
        )

        self.shoot_frequency = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.latest_cmd_vel = None
        self.buffer_lock = threading.Lock()

        self.timer = self.create_timer(0.01, self.publish_to_serial)
        self.serial_port = serial.Serial(
            port="/dev/ttyUSB0",
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        )
        time.sleep(1)

        self.micro_publisher = self.create_publisher(
            PoseStamped,
            "/micro_pose",
            10)

        self.get_logger().info("Serial port opened")

    def listener_callback(self, msg):
        with self.buffer_lock:
            self.latest_cmd_vel = msg

    def publish_to_serial(self):
        with self.buffer_lock:
            if self.latest_cmd_vel is not None:
                # angular.x = fire command (0/1), angular.y = pitch (deg), angular.z = yaw (deg)
                self.shoot_frequency = self.latest_cmd_vel.angular.x
                self.pitch = self.latest_cmd_vel.angular.y
                self.yaw = -(self.latest_cmd_vel.angular.z)
                self.latest_cmd_vel = None

        # TX: [yaw, pitch, shoot_frequency, 0.0, 0.0]
        formatted_data = format_float(
            np.array([self.yaw, self.pitch, self.shoot_frequency, 0.0, 0.0],
                     dtype=np.float32))
        self.serial_port.write(formatted_data)

        # RX: read feedback from lower computer
        in_waiting = self.serial_port.in_waiting
        if in_waiting >= PACKET_SIZE:
            var = self.serial_port.read(PACKET_SIZE)
            # Discard any extra stale bytes
            leftover = self.serial_port.in_waiting
            if leftover > 0:
                self.serial_port.read(leftover)

            try:
                yaw = struct.unpack('<f', bytes(var[:4]))[0]
                pitch = struct.unpack('<f', bytes(var[4:8]))[0]
            except struct.error:
                return

            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose.position.x = -pitch
            pose_stamped.pose.position.y = -yaw
            pose_stamped.pose.position.z = 0.0
            self.micro_publisher.publish(pose_stamped)

    def destroy_node(self):
        if self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    cmd_vel_subscriber = CmdVelSubscriber()
    rclpy.spin(cmd_vel_subscriber)
    cmd_vel_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
