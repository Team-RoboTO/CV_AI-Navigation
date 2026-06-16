#!/usr/bin/env python3
import os
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


class SavePCDOnce(Node):
    def __init__(self):
        super().__init__('save_pcd_once')
        self.declare_parameter('topic', '/Laser_map')
        self.declare_parameter('out_dir', '/root/nav2_ws/maps')
        self.declare_parameter('prefix', 'fastlio_map')
        self.topic = self.get_parameter('topic').value
        self.out_dir = self.get_parameter('out_dir').value
        self.prefix = self.get_parameter('prefix').value
        os.makedirs(self.out_dir, exist_ok=True)
        self.sub = self.create_subscription(PointCloud2, self.topic, self._cb, 10)
        self.get_logger().info(f'Waiting for PointCloud2 on {self.topic}')

    def _cb(self, msg):
        pts = []
        for p in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            pts.append([float(p[0]), float(p[1]), float(p[2])])
        if not pts:
            self.get_logger().warn('Received empty cloud, waiting...')
            return
        arr = np.asarray(pts, dtype=np.float32)
        path = os.path.join(self.out_dir, datetime.now().strftime(f'{self.prefix}_%Y%m%d_%H%M%S.pcd'))
        with open(path, 'w') as f:
            f.write('# .PCD v0.7 - Point Cloud Data file format\n')
            f.write('VERSION 0.7\n')
            f.write('FIELDS x y z\n')
            f.write('SIZE 4 4 4\n')
            f.write('TYPE F F F\n')
            f.write('COUNT 1 1 1\n')
            f.write(f'WIDTH {len(arr)}\n')
            f.write('HEIGHT 1\n')
            f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
            f.write(f'POINTS {len(arr)}\n')
            f.write('DATA ascii\n')
            for x, y, z in arr:
                f.write(f'{x:.6f} {y:.6f} {z:.6f}\n')
        self.get_logger().info(f'Saved {len(arr)} points to {path}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SavePCDOnce()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
