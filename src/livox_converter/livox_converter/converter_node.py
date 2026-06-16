"""
livox_converter – Convert Livox CustomMsg → sensor_msgs/PointCloud2.

The Livox Mid-360 ROS 2 driver publishes point clouds in a proprietary format
called livox_ros_driver2/CustomMsg. This node subscribes to that topic and
republishes a standard PointCloud2 message that the rest of the ROS stack
(pointcloud_to_laserscan, RViz, costmaps, etc.) can consume.

Subscribes: /livox/lidar      (livox_ros_driver2/CustomMsg)
Publishes:  /livox/lidar_pc2  (sensor_msgs/PointCloud2)
"""
import struct

import rclpy
from rclpy.node import Node
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import PointCloud2, PointField


class LivoxCustomToPC2(Node):
    def __init__(self):
        super().__init__('livox_custom_to_pc2_node')

        self.declare_parameter('input_topic', '/livox/lidar')
        self.declare_parameter('output_topic', '/livox/lidar_pc2')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.subscription = self.create_subscription(
            CustomMsg, input_topic, self._on_custom_msg, 10)

        self.publisher = self.create_publisher(
            PointCloud2, output_topic, 10)

        self.get_logger().info(
            f'Livox converter ready: {input_topic} → {output_topic}')

    def _on_custom_msg(self, msg: CustomMsg):
        pc2 = PointCloud2()
        pc2.header = msg.header
        pc2.height = 1
        pc2.width = msg.point_num

        pc2.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        pc2.is_bigendian = False
        pc2.point_step = 16
        pc2.row_step = pc2.point_step * pc2.width
        pc2.is_dense = True

        # Pack all points at once — much faster than per-point pack
        buffer = bytearray(pc2.row_step)
        offset = 0
        for p in msg.points:
            struct.pack_into('ffff', buffer, offset,
                             p.x, p.y, p.z, float(p.reflectivity))
            offset += 16

        pc2.data = bytes(buffer)
        self.publisher.publish(pc2)


def main(args=None):
    rclpy.init(args=args)
    node = LivoxCustomToPC2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
