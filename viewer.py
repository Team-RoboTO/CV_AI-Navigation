import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection2D
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import cv2
import numpy as np

from geometry_msgs.msg import PoseStamped

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
        self.gimbal_cmd = None

        # Gimbal state from /micro_pose (updated at ~100 Hz)
        self._gimbal_yaw = 0.0
        self._gimbal_pitch = 0.0
        self._yaw_sign = 1.0     # matches gimbal.yaw_sign in debug.launch.py
        self._pitch_sign = 1.0   # matches gimbal.pitch_sign in debug.launch.py
        self._gimbal_height = 0.325  # [m] matches gimbal.height in debug.launch.py

        # Precompute RPY(-π/2, 0, -π/2) convention rotation matrix
        # This maps from gimbal mechanical convention to camera optical frame
        # and matches the tracker's broadcastGimbalTF exactly.
        self._R_convention = np.array([
            [ 0,  0,  1],
            [-1,  0,  0],
            [ 0, -1,  0]], dtype=np.float64)

        self.create_subscription(Image, '/camera/camera/color/image_raw', self.img_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.info_cb, qos_profile_sensor_data)
        self.create_subscription(Detection2DArray, '/detections_output', self.bbox_cb, qos_profile_sensor_data)
        self.create_subscription(Detection2D, '/detections_output/optimal_target', self.optimal_cb, qos_profile_sensor_data)
        self.create_subscription(Marker, '/trajectory/marker', self.marker_cb, qos_profile_sensor_data)

        # Gimbal feedback — fresh angles for impact projection
        self.create_subscription(PoseStamped, '/micro_pose', self.micro_pose_cb, qos_profile_sensor_data)

        # Try subscribing to tracker target and gimbal commands
        try:
            from auto_aim_interfaces.msg import Target, GimbalCmd
            self.create_subscription(Target, '/tracker/target', self.target_cb, qos_profile_sensor_data)
            self.create_subscription(GimbalCmd, '/tracker/cmd_gimbal', self.gimbal_cmd_cb, qos_profile_sensor_data)
        except ImportError:
            self.get_logger().warn("auto_aim_interfaces not found — run: source install/setup.bash")

        self.pub = self.create_publisher(Image, '/annotated_image', 10)

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

    def gimbal_cmd_cb(self, msg):
        self.gimbal_cmd = msg

    def micro_pose_cb(self, msg):
        p = self._pitch_sign * msg.pose.position.x
        y = self._yaw_sign * msg.pose.position.y
        # Reject garbage serial data (gimbal angles can't exceed ~90°)
        if abs(p) < 1.6 and abs(y) < 1.6:
            self._gimbal_pitch = p
            self._gimbal_yaw = y

    def bbox_cb(self, msg):
        self.latest_detections = msg

    def img_cb(self, msg):
        self.latest_image = msg
        self.draw_and_publish()

    def bbox_to_rect(self, det, cv_img):
        """Convert a Detection2D bbox to pixel rectangle.
        Scales from decoder output space (camera_info resolution) to actual image."""
        h_img, w_img, _ = cv_img.shape
        
        pad_y = 80  # letterbox padding: (640 - 480) / 2
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
        #    Project using latest /micro_pose gimbal angles instead of TF to
        #    avoid timing mismatch (TF is broadcast at detection time, but
        #    the image arrives before YOLO finishes → stale TF while gimbal slews).
        if self.latest_marker is not None and self.camera_info is not None:
            p_odom = np.array([
                self.latest_marker.pose.position.x,
                self.latest_marker.pose.position.y,
                self.latest_marker.pose.position.z])

            # odom→camera: p_cam = R_total.T @ (p_odom - translation)
            yaw = self._gimbal_yaw
            pitch = self._gimbal_pitch
            cos_y, sin_y = np.cos(yaw), np.sin(yaw)
            cos_p, sin_p = np.cos(pitch), np.sin(pitch)
            R_yaw = np.array([[ cos_y, -sin_y, 0],
                               [ sin_y,  cos_y, 0],
                               [ 0,      0,     1]])
            R_pitch = np.array([[ cos_p, 0, sin_p],
                                 [ 0,     1, 0    ],
                                 [-sin_p, 0, cos_p]])
            R_total = R_yaw @ R_pitch @ self._R_convention
            p_translated = p_odom - np.array([0.0, 0.0, self._gimbal_height])
            p_cam = R_total.T @ p_translated

            x, y, z = p_cam
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

        # 4. Aim direction (cyan) — where the barrel is pointing
        if self.gimbal_cmd is not None and self.gimbal_cmd.distance > 0.1:
            import math
            cmd = self.gimbal_cmd
            pitch_deg = cmd.pitch
            yaw_deg = cmd.yaw
            dist = cmd.distance
            
            pitch = math.radians(pitch_deg)
            yaw = math.radians(yaw_deg)

            # Option 2: Direct Projection in Camera Optical Frame
            # Z is forward, X is right, Y is down
            # Relative yaw moves barrel left -> negative X
            # Relative pitch moves barrel up -> negative Y
            z = dist * math.cos(pitch) * math.cos(yaw)
            x = -dist * math.cos(pitch) * math.sin(yaw)
            y = -dist * math.sin(pitch)
            
            if z > 0.05:
                K = self.camera_info.k
                fx, cx_cam = K[0], K[2]
                fy, cy_cam = K[4], K[5]
                # Project onto image plane
                au = int((x / z) * fx + cx_cam)
                av = int((y / z) * fy + cy_cam)
                
                if 0 <= au < w_img and 0 <= av < h_img:
                    # Cyan diamond crosshair for aim
                    sz = 15
                    pts = [(au, av-sz), (au+sz, av), (au, av+sz), (au-sz, av)]
                    for i in range(4):
                        cv2.line(cv_img, pts[i], pts[(i+1)%4], (255, 255, 0), 2)
                    cv2.putText(cv_img, "AIM", (au + 10, av - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)


            # Angle text overlay (top-left)
            fire_str = "FIRE" if cmd.fire_cmd else "hold"
            cv2.putText(cv_img, f"Pitch: {pitch_deg:+.1f} deg", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(cv_img, f"Yaw:   {yaw_deg:+.1f} deg", (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(cv_img, f"Dist:  {dist:.2f}m  [{fire_str}]", (10, 75),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

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