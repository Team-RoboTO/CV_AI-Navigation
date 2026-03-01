import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection2D
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import cv2

import tf2_ros
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs  # noqa: F401  — registers PointStamped transform

class VisualizerNode(Node):
    def __init__(self):
        super().__init__('visualizer_node')
        self.bridge = CvBridge()
        
        self.latest_image = None
        self.latest_detections = None
        self.optimal_target = None
        self.latest_marker = None
        self.camera_info = None
        self.tracking_info = None

        self.create_subscription(Image, '/camera/camera/color/image_raw', self.img_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.info_cb, qos_profile_sensor_data)
        self.create_subscription(Detection2DArray, '/detections_output', self.bbox_cb, qos_profile_sensor_data)
        self.create_subscription(Detection2D, '/detections_output/optimal_target', self.optimal_cb, qos_profile_sensor_data)
        self.create_subscription(Marker, '/trajectory/marker', self.marker_cb, qos_profile_sensor_data)

        # Try subscribing to tracker target for distance/tracking info
        try:
            from auto_aim_interfaces.msg import Target
            self.create_subscription(Target, '/tracker/target', self.target_cb, qos_profile_sensor_data)
        except ImportError:
            self.get_logger().warn("auto_aim_interfaces not found — run: source install/setup.bash")

        self.pub = self.create_publisher(Image, '/annotated_image', 10)

        # TF2 for transforming impact point from odom → camera frame
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info("Visualizer Node started!")

    def info_cb(self, msg):
        self.camera_info = msg

    def optimal_cb(self, msg):
        self.optimal_target = msg

    def marker_cb(self, msg):
        if msg.ns == 'impact_point':
            if msg.action == Marker.DELETE:
                self.latest_marker = None
            else:
                self.latest_marker = msg

    def target_cb(self, msg):
        self.tracking_info = msg

    def bbox_cb(self, msg):
        self.latest_detections = msg

    def img_cb(self, msg):
        self.latest_image = msg
        self.draw_and_publish()

    def bbox_to_rect(self, det, cv_img):
        """Convert a Detection2D bbox to pixel rectangle.
        The YOLO decoder un-letterboxes coordinates back to camera-native
        pixel space (matching camera_info K matrix), so no pad correction needed."""
        h_img, w_img, _ = cv_img.shape
        pad_y = 80.0
        x_scale = w_img / 640.0
        y_scale = h_img / 480.0

        cx = int(det.bbox.center.position.x * x_scale)
        w  = int(det.bbox.size_x * x_scale)
        cy = int((det.bbox.center.position.y - pad_y) * y_scale)
        h  = int(det.bbox.size_y * y_scale)

        pt1 = (cx - w//2, cy - h//2)
        pt2 = (cx + w//2, cy + h//2)
        return pt1, pt2, (cx, cy)

    def draw_and_publish(self):
        if self.latest_image is None or self.camera_info is None:
            return

        cv_img = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding="bgr8")
        h_img, w_img, _ = cv_img.shape
        
        # 1. YOLO bounding boxes (green)
        if self.latest_detections is not None:
            for det in self.latest_detections.detections:
                pt1, pt2, _ = self.bbox_to_rect(det, cv_img)
                cv2.rectangle(cv_img, pt1, pt2, (0, 255, 0), 2)
                
                if det.results:
                    class_id = det.results[0].hypothesis.class_id
                    cv2.putText(cv_img, f"ID: {class_id}", (pt1[0], pt1[1] - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # 2. Tracked armor (cyan) — from optimal_target bbox
        if (self.optimal_target is not None and 
            self.optimal_target.bbox.size_x > 0):
            det = self.optimal_target
            pt1, pt2, center = self.bbox_to_rect(det, cv_img)

            # Thicker cyan bbox
            cv2.rectangle(cv_img, pt1, pt2, (0, 255, 255), 3)

            # Crosshair at center
            cx, cy = center
            cv2.line(cv_img, (cx - 20, cy), (cx + 20, cy), (0, 255, 255), 2)
            cv2.line(cv_img, (cx, cy - 20), (cx, cy + 20), (0, 255, 255), 2)

            # Tracking info overlay
            if self.tracking_info is not None and self.tracking_info.tracking:
                tgt = self.tracking_info
                dist = (tgt.position.x**2 + tgt.position.y**2 + tgt.position.z**2) ** 0.5
                label = f"d={dist:.2f}m yaw={tgt.yaw:.1f} v={tgt.v_yaw:.1f}r/s"
                cv2.putText(cv_img, label, (pt1[0], pt1[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        # 3. Impact point (red) — from trajectory solver marker
        if self.latest_marker is not None:
            # Transform from marker frame (odom) → camera optical frame
            # Use latest TF (Time(0)) instead of marker stamp to avoid
            # ExtrapolationException when TF and marker timestamps differ.
            pt_odom = PointStamped()
            pt_odom.header.frame_id = self.latest_marker.header.frame_id
            pt_odom.header.stamp = rclpy.time.Time().to_msg()  # latest TF
            pt_odom.point = self.latest_marker.pose.position
            try:
                pt_cam = self.tf_buffer.transform(
                    pt_odom, 'camera_color_optical_frame',
                    timeout=rclpy.duration.Duration(seconds=0.05))
                x = pt_cam.point.x
                y = pt_cam.point.y
                z = pt_cam.point.z
                # Project (x,y,z) in camera optical frame → pixel
                if z > 0.05:
                    K = self.camera_info.k
                    fx, cx_cam = K[0], K[2]
                    fy, cy_cam = K[4], K[5]
                    u = int((x / z) * fx + cx_cam)
                    v = int((y / z) * fy + cy_cam)
                    if 0 <= u < w_img and 0 <= v < h_img:
                        cv2.circle(cv_img, (u, v), 6, (0, 0, 255), -1)
                        cv2.line(cv_img, (u - 20, v), (u + 20, v), (0, 0, 255), 2)
                        cv2.line(cv_img, (u, v - 20), (u, v + 20), (0, 0, 255), 2)
                        cv2.putText(cv_img, "IMPACT", (u + 10, v - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn(f"Impact TF failed: {e}", throttle_duration_sec=2.0)

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