#"""
#cmd_vel_xy_only - pass through only linear x/y from Nav2 and force angular z = 0.
#
#Purpose:
#- Keep translation control from Nav2 in a holonomic base.
#- Prevent Nav2 from rotating the robot to align with the path or goal.
#- Leave yaw control to the combat / CV stack.
#
#Typical wiring:
#    Nav2 -> /cmd_vel_nav_raw -> cmd_vel_xy_only -> /cmd_vel_NAV
#"""
#
#import rclpy
#from rclpy.node import Node
#from geometry_msgs.msg import Twist
#import math
#
#
#class CmdVelXYOnly(Node):
#    def __init__(self):
#        super().__init__('cmd_vel_xy_only')
#
#        self.declare_parameter('input_topic', '/cmd_vel_nav_raw')
#        self.declare_parameter('output_topic', '/cmd_vel_NAV')
#        self.declare_parameter('max_linear_x', 2.0)
#        self.declare_parameter('max_linear_y', 2.0)
#        # Rotate Nav2 velocity from base_link/head frame into robot/barrel frame.
#        # Use +90 or -90 depending on motor convention.
#        self.declare_parameter('rotate_yaw_deg', 90.0)
#
#        input_topic = self.get_parameter('input_topic').value
#        output_topic = self.get_parameter('output_topic').value
#        self.max_x = float(self.get_parameter('max_linear_x').value)
#        self.max_y = float(self.get_parameter('max_linear_y').value)
#        self.rotate_yaw = math.radians(float(self.get_parameter('rotate_yaw_deg').value))
#        self.cos_yaw = math.cos(self.rotate_yaw)
#        self.sin_yaw = math.sin(self.rotate_yaw)
#
#        self.sub = self.create_subscription(Twist, input_topic, self._on_cmd, 50)
#        self.pub = self.create_publisher(Twist, output_topic, 50)
#
#        self.get_logger().info(
#            f'cmd_vel_xy_only active: {input_topic} -> {output_topic}; rotate_yaw_deg={math.degrees(self.rotate_yaw):.1f}; angular.z forced to 0'
#        )
#
#    def _on_cmd(self, msg: Twist):
#        out = Twist()
#
#        rx = -msg.linear.y
#        ry = -msg.linear.x
#        out.linear.x = max(-self.max_x, min(self.max_x, rx))
#        out.linear.y = max(-self.max_y, min(self.max_y, ry))
#        
#        out.linear.z = 0.0
#        out.angular.x = 0.0
#        out.angular.y = 0.0
#        out.angular.z = 0.0
#        self.pub.publish(out)
#
#
#def main(args=None):
#    rclpy.init(args=args)
#    node = None
#    try:
#        node = CmdVelXYOnly()
#        rclpy.spin(node)
#    except KeyboardInterrupt:
#        pass
#    finally:
#        if node is not None:
#            node.destroy_node()
#        try:
#            rclpy.shutdown()
#        except Exception:
#            pass
#
#
#if __name__ == '__main__':
#    main()
"""
cmd_vel_xy_only - pass through only linear x/y from Nav2 and force angular z = 0.

Purpose:
- Keep translation control from Nav2 in a holonomic base.
- Prevent Nav2 from rotating the robot to align with the path or goal.
- Leave yaw control to the combat / CV stack.

Typical wiring:
    Nav2 -> /cmd_vel_nav -> cmd_vel_xy_only -> /cmd_vel_NAV
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelXYOnly(Node):
    def __init__(self):
        super().__init__('cmd_vel_xy_only')

        self.declare_parameter('input_topic', '/cmd_vel_nav')
        self.declare_parameter('output_topic', '/cmd_vel_NAV')

        self.declare_parameter('max_linear_x', 2.0)
        self.declare_parameter('max_linear_y', 2.0)

        # Minimum speed compensation for motor deadzone
        self.declare_parameter('min_linear_speed', 0.00)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.max_x = float(self.get_parameter('max_linear_x').value)
        self.max_y = float(self.get_parameter('max_linear_y').value)

        self.min_linear_speed = float(
            self.get_parameter('min_linear_speed').value
        )

        self.sub = self.create_subscription(
            Twist,
            input_topic,
            self._on_cmd,
            50
        )

        self.pub = self.create_publisher(
            Twist,
            output_topic,
            50
        )

        self.get_logger().info(
            f'cmd_vel_xy_only active: '
            f'{input_topic} -> {output_topic}; '
            f'min_linear_speed={self.min_linear_speed:.2f}; '
            f'angular.z forced to 0'
        )

    def _on_cmd(self, msg: Twist):
        out = Twist()

        # ------------------------------------------------------------
        # NAV2 FRAME -> ROBOT FRAME CONVERSION
        #
        # Nav2/base_link:
        #   +X = forward
        #   +Y = left
        #
        # Robot/motor frame:
        #   adjusted for your 90° rotated lidar mounting
        # ------------------------------------------------------------

        rx = msg.linear.x
        ry = -msg.linear.y

        # Clamp max velocity
        out.linear.x = max(-self.max_x, min(self.max_x, rx))
        out.linear.y = max(-self.max_y, min(self.max_y, ry))

        # ------------------------------------------------------------
        # MINIMUM SPEED COMPENSATION
        #
        # Prevent tiny Nav2 speeds (0.02–0.05) that cannot move
        # the real motors.
        # ------------------------------------------------------------

        speed = math.hypot(out.linear.x, out.linear.y)

        if 0.0 < speed < self.min_linear_speed:
            scale = self.min_linear_speed / speed
            out.linear.x *= scale
            out.linear.y *= scale

        # Disable all rotations
        out.linear.z = 0.0

        out.angular.x = 0.0
        out.angular.y = 0.0
        out.angular.z = 0.0

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = CmdVelXYOnly()
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