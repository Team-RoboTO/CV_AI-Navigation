#!/usr/bin/env python3

# Thin wrapper that launches the viewer node with flipped=True.
# Kept for backward compatibility with existing launch files.

import rclpy
from .viewer_node import BboxWithDistance


def main():
    rclpy.init()
    node = BboxWithDistance()
    # Override parameter to flipped mode
    node.flipped = True
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
