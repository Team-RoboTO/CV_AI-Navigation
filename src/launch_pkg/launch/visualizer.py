import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2

import tf2_ros
import tf2_geometry_msgs

class VisualizerNode(Node):
    def __init__(self):
        super().__init__('visualizer_node')
        self.bridge = CvBridge()
        
        self.latest_image = None
        self.latest_detections = None
        self.latest_marker = None
        self.camera_info = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # sottoscrizioni
        self.create_subscription(Image, '/image', self.img_cb, 10)
        self.create_subscription(CameraInfo, '/camera_info', self.info_cb, 10)
        self.create_subscription(Detection2DArray, '/detections_output', self.bbox_cb, 10)
        self.create_subscription(Marker, '/trajectory/marker', self.marker_cb, 10)
        
        # publisher
        self.pub = self.create_publisher(Image, '/annotated_image', 10)
        self.get_logger().info("VisualizerNode started")

    def info_cb(self, msg):
        self.camera_info = msg

    def marker_cb(self, msg):
        if msg.ns == "impact_point":
            self.latest_marker = msg

    def bbox_cb(self, msg):
        self.latest_detections = msg

    def img_cb(self, msg):
        self.latest_image = msg
        self.draw_and_publish()

    def draw_and_publish(self):
        if self.latest_image is None or self.camera_info is None:
            return

        cv_img = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding="bgr8")
        
        #        bounding box yolo
        if self.latest_detections is not None:
            for det in self.latest_detections.detections:
                cx = int(det.bbox.center.position.x)
                cy = int(det.bbox.center.position.y)
                w = int(det.bbox.size_x)
                h = int(det.bbox.size_y)
                
                pt1 = (cx - w//2, cy - h//2)
                pt2 = (cx + w//2, cy + h//2)
                cv2.rectangle(cv_img, pt1, pt2, (0, 255, 0), 2)

        if self.latest_marker is not None:
            try:
                pt = PointStamped()
                pt.header = self.latest_marker.header
                pt.point = self.latest_marker.pose.position

                camera_frame = self.camera_info.header.frame_id
                
                transformed_pt = self.tf_buffer.transform(pt, camera_frame, rclpy.time.Duration(seconds=0.1))

                x = transformed_pt.point.x
                y = transformed_pt.point.y
                z = transformed_pt.point.z

                if z > 0.05:
                    K = self.camera_info.k
                    fx, cx_cam = K[0], K[2]
                    fy, cy_cam = K[4], K[5]

                    u = int((x / z) * fx + cx_cam)
                    v = int((y / z) * fy + cy_cam)

                    h_img, w_img, _ = cv_img.shape
                    if 0 <= u < w_img and 0 <= v < h_img:
                        cv2.circle(cv_img, (u, v), 6, (0, 0, 255), -1) 
                        cv2.line(cv_img, (u - 20, v), (u + 20, v), (0, 0, 255), 2)
                        cv2.line(cv_img, (u, v - 20), (u, v + 20), (0, 0, 255), 2)
                        cv2.putText(cv_img, "IMPACT", (u + 10, v - 10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                        
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn(f"TF Error: {e}")

        out_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
        out_msg.header = self.latest_image.header
        self.pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()