#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        
        self.declare_parameter('video_path', 'media/output2.mp4')
        video_path = self.get_parameter('video_path').value
        
        self.image_pub = self.create_publisher(Image, '/camera/camera/color/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera/color/camera_info', 10)
        
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(video_path)
        
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open video file: {video_path}")
            return
            
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        if fps <= 0:
            fps = 30.0
            
        self.timer = self.create_timer(1.0 / fps, self.timer_callback)
        self.get_logger().info(f"Publishing video {video_path} at {fps} FPS")

    def timer_callback(self):
        ret, frame = self.cap.read()
        
        if not ret:
            self.get_logger().info("End of video reached. Restarting...")
            # Re-initialize the capture object to guarantee it restarts
            # (cv2.CAP_PROP_POS_FRAMES doesn't always work for all video formats)
            video_path = self.get_parameter('video_path').value
            self.cap.release()
            self.cap = cv2.VideoCapture(video_path)
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error("Failed to restart video!")
                return

        now = self.get_clock().now().to_msg()
        
        # Publish Image
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        img_msg.header.stamp = now
        img_msg.header.frame_id = "camera_color_optical_frame"
        self.image_pub.publish(img_msg)
        
        # Publish basic CameraInfo
        info_msg = CameraInfo()
        info_msg.header = img_msg.header
        # Add basic dummy values so nodes don't crash
        info_msg.height = frame.shape[0]
        info_msg.width = frame.shape[1]
        info_msg.distortion_model = "plumb_bob"
        info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Typical dummy intrinsics matrix for 640x480
        fx, fy = 600.0, 600.0
        cx, cy = getattr(info_msg, 'width') / 2.0, getattr(info_msg, 'height') / 2.0
        info_msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info_msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        
        self.info_pub.publish(info_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'cap'):
            node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
