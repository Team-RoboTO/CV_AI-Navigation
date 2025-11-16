import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import numpy as np
import time

class BoundingBoxDepthNode(Node):

    def __init__(self):
        super().__init__('bounding_box_depth_node')

        self.bridge = CvBridge()
        self.detections = None
        self.camera_info = None
        self.depth_image = None
        self.prev_time = time.time()
        # Subscribers
        self.detection_sub = self.create_subscription(Detection2DArray, '/detections',self.detections_callback, qos_profile=qos_profile_system_default)
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw',self.depth_callback, qos_profile=qos_profile_system_default)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.camera_info_callback, qos_profile=qos_profile_system_default)
        self.publisher= self.create_publisher(Detection2DArray, '/detections_output/with_pose', qos_profile=qos_profile_system_default)
        # Synchronize messages
        self.timer = self.create_timer(0.05, self.publish_callback)

        self.get_logger().info('Bounding Box Depth Node has been started.')
    def detections_callback(self, msg):
        self.detections = msg
    def depth_callback(self, msg):
        try:
        # Convert the depth image to an OpenCV format using the correct encoding
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

    def camera_info_callback(self, msg):
        self.camera_info = msg
    
    def publish_callback(self):

        detections = self.detections 
        camera_info = self.camera_info
        depth_image_cv = self.depth_image
        start = time.time()
        self.get_logger().info(f'time from previous execution{self.prev_time - start}')
        
        if self.detections is None or self.camera_info is None or self.depth_image is None:
            return
        
        for detection in self.detections.detections:
            # Check if the detection frame ID matches the depth image frame ID
            bbox = detection.bbox
            x = int(bbox.center.position.x - bbox.size_x / 2)
            y = int(bbox.center.position.y - bbox.size_y / 2)
            width = int(bbox.size_x*0.7)
            height = int(bbox.size_y*0.7)

            # Ensure the bounding box is within the image bounds
            x = max(0, x)
            y = max(0, y)
            width = min(width, depth_image_cv.shape[1] - x)
            height = min(height, depth_image_cv.shape[0] - y)

            if width <= 0 or height <= 0:
                continue

            depth_roi = depth_image_cv[y:y+height, x:x+width]

            # Filter out invalid depth values (e.g., those with zero distance)
            valid_depths = depth_roi[depth_roi > 0]
            if valid_depths.size == 0:
                continue

            mean_distance = np.mean(valid_depths) / 1000.0  # Convert from millimeters to meters

            #self.get_logger().info(f'Mean distance for bbox (x={x}, y={y}, w={width}, h={height}): {mean_distance} meters')

            # Compute the 3D coordinates in the camera frame
            center_x = bbox.center.position.x
            center_y = bbox.center.position.y
            fx = self.camera_info.k[0]
            fy = self.camera_info.k[4]
            cx = self.camera_info.k[2]
            cy = self.camera_info.k[5]

            relative_x = (center_x - cx) * mean_distance / fx
            relative_y = (center_y - cy) * mean_distance / fy
            relative_z = mean_distance
            detection.results[0].pose.pose.position.x = relative_x
            detection.results[0].pose.pose.position.y = relative_y
            detection.results[0].pose.pose.position.z = relative_z

        self.publisher.publish(detections)
        end = time.time()
        self.prev_time = end
        self.get_logger().info(f'Processing time: {end - start}')
        self.detections = None 
        self.depth_image = None
def main(args=None):
    rclpy.init(args=args)
    node = BoundingBoxDepthNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
