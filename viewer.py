import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2

try:
    from auto_aim.msg import ArmorKeypointArray, AutoAimDebug
except ImportError:
    ArmorKeypointArray = None
    AutoAimDebug = None

class VisualizerNode(Node):
    def __init__(self):
        super().__init__('visualizer_node')
        self.bridge = CvBridge()

        self.latest_image = None
        self.latest_detections = None
        self.latest_keypoints = None
        self.latest_debug = None
        self.camera_info = None
        self.gimbal_cmd = None
        self.aim_pixels = None
        self.image_time = None
        self.camera_info_time = None
        self.detections_time = None
        self.keypoints_time = None
        self.debug_time = None
        self.gimbal_cmd_time = None
        self.aim_pixels_time = None

        self.image_topic = self.declare_parameter(
            'image_topic', '/camera/camera/color/image_raw').value
        self.camera_info_topic = self.declare_parameter(
            'camera_info_topic', '/camera/camera/color/camera_info').value
        self.detection_topic = self.declare_parameter(
            'detection_topic', '/detector/armors').value
        self.keypoint_topic = self.declare_parameter(
            'keypoint_topic', '/detector/armors_keypoints').value
        self.debug_topic = self.declare_parameter(
            'debug_topic', '/auto_aim/debug').value
        self.cmd_topic = self.declare_parameter(
            'cmd_topic', '/tracker/cmd_gimbal').value
        self.aim_pixels_topic = self.declare_parameter(
            'aim_pixels_topic', '/tracker/aim_pixels').value

        self.create_subscription(Image, self.image_topic, self.img_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.info_cb, qos_profile_sensor_data)
        self.create_subscription(Detection2DArray, self.detection_topic, self.bbox_cb, qos_profile_sensor_data)
        self.create_subscription(Twist, self.cmd_topic, self.gimbal_cmd_cb, qos_profile_sensor_data)
        self.create_subscription(Twist, self.aim_pixels_topic, self.aim_pixels_cb, qos_profile_sensor_data)

        if ArmorKeypointArray is not None:
            self.create_subscription(
                ArmorKeypointArray, self.keypoint_topic,
                self.keypoints_cb, qos_profile_sensor_data)
        else:
            self.get_logger().warn(
                "auto_aim messages not found; keypoint overlay disabled. "
                "Run: source install/setup.bash")

        if AutoAimDebug is not None:
            self.create_subscription(
                AutoAimDebug, self.debug_topic,
                self.debug_cb, qos_profile_sensor_data)

        self.pub = self.create_publisher(Image, '/annotated_image', 10)

        self.get_logger().info(
            f"Visualizer started: image={self.image_topic}, "
            f"keypoints={self.keypoint_topic}, cmd={self.cmd_topic}")

    def info_cb(self, msg):
        self.camera_info = msg
        self.camera_info_time = self.get_clock().now()

    def gimbal_cmd_cb(self, msg):
        self.gimbal_cmd = msg
        self.gimbal_cmd_time = self.get_clock().now()

    def aim_pixels_cb(self, msg):
        self.aim_pixels = msg
        self.aim_pixels_time = self.get_clock().now()

    def keypoints_cb(self, msg):
        self.latest_keypoints = msg
        self.keypoints_time = self.get_clock().now()

    def debug_cb(self, msg):
        self.latest_debug = msg
        self.debug_time = self.get_clock().now()

    def bbox_cb(self, msg):
        self.latest_detections = msg
        self.detections_time = self.get_clock().now()

    def img_cb(self, msg):
        self.latest_image = msg
        self.image_time = self.get_clock().now()
        self.draw_and_publish()

    def bbox_fields_to_rect(self, cx, cy, box_w, box_h, cv_img):
        h_img, w_img, _ = cv_img.shape
        cx = int(round(cx))
        cy = int(round(cy))
        w = int(round(box_w))
        h = int(round(box_h))
        x1 = max(0, min(w_img - 1, cx - w // 2))
        y1 = max(0, min(h_img - 1, cy - h // 2))
        x2 = max(0, min(w_img - 1, cx + w // 2))
        y2 = max(0, min(h_img - 1, cy + h // 2))
        return (x1, y1), (x2, y2), (cx, cy)

    def bbox_to_rect(self, det, cv_img):
        """Convert a Detection2D bbox to native image pixels.

        The new YOLOv26 detector publishes bbox coordinates in the incoming
        RealSense image frame, so no old 640x640 letterbox de-padding is needed.
        """
        return self.bbox_fields_to_rect(
            det.bbox.center.position.x,
            det.bbox.center.position.y,
            det.bbox.size_x,
            det.bbox.size_y,
            cv_img,
        )

    def draw_label(self, cv_img, text, org, color, scale=0.55):
        x, y = org
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 2)
        cv2.rectangle(cv_img, (x - 2, y - th - 5), (x + tw + 4, y + 4), (0, 0, 0), -1)
        cv2.putText(cv_img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 2)

    def topic_age_s(self, stamp):
        if stamp is None:
            return None
        return (self.get_clock().now() - stamp).nanoseconds * 1.0e-9

    def topic_status(self, label, stamp, ok_detail="", warn_after_s=1.0):
        age = self.topic_age_s(stamp)
        if age is None:
            return f"{label}: NO DATA", (0, 0, 255)
        if age > warn_after_s:
            return f"{label}: STALE {age:.1f}s", (0, 128, 255)
        detail = f" {ok_detail}" if ok_detail else ""
        return f"{label}: OK{detail}", (0, 255, 0)

    def topic_is_fresh(self, stamp, max_age_s=0.35):
        age = self.topic_age_s(stamp)
        return age is not None and age <= max_age_s

    def draw_keypoints(self, cv_img):
        if self.latest_keypoints is None or not self.topic_is_fresh(self.keypoints_time):
            return False

        msg = self.latest_keypoints
        h_img, w_img, _ = cv_img.shape
        sx = float(w_img) / float(msg.image_width) if msg.image_width else 1.0
        sy = float(h_img) / float(msg.image_height) if msg.image_height else 1.0

        drew = False
        names = list(msg.keypoint_order) if msg.keypoint_order else ["TL", "TR", "BR", "BL"]
        for det in msg.detections:
            cx = det.bbox_cx * sx
            cy = det.bbox_cy * sy
            bw = det.bbox_w * sx
            bh = det.bbox_h * sy
            pt1, pt2, _ = self.bbox_fields_to_rect(cx, cy, bw, bh, cv_img)

            color = (0, 220, 0)
            cv2.rectangle(cv_img, pt1, pt2, color, 2)

            pts = []
            for i in range(4):
                x = int(round(det.keypoints_xy[2 * i] * sx))
                y = int(round(det.keypoints_xy[2 * i + 1] * sy))
                valid = bool(det.keypoint_valid[i])
                score = float(det.keypoint_scores[i])
                pts.append((x, y, valid, score))

            for i in range(4):
                x1, y1, v1, _ = pts[i]
                x2, y2, v2, _ = pts[(i + 1) % 4]
                line_color = (0, 255, 0) if v1 and v2 else (0, 128, 255)
                cv2.line(cv_img, (x1, y1), (x2, y2), line_color, 2)

            for i, (x, y, valid, score) in enumerate(pts):
                point_color = (255, 255, 255) if valid else (0, 0, 255)
                cv2.circle(cv_img, (x, y), 4, point_color, -1)
                cv2.putText(cv_img, names[i] if i < len(names) else str(i),
                            (x + 4, y - 4), cv2.FONT_HERSHEY_SIMPLEX,
                            0.4, point_color, 1)
                if not valid:
                    cv2.putText(cv_img, f"{score:.2f}", (x + 4, y + 12),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.35, point_color, 1)

            label = f"{det.class_name or det.class_id} {det.confidence:.2f}"
            self.draw_label(cv_img, label, (pt1[0], max(18, pt1[1] - 8)), color)
            drew = True

        return drew

    def draw_debug_measurement(self, cv_img):
        dbg = self.latest_debug
        if dbg is None or not self.topic_is_fresh(self.debug_time, 0.5):
            return

        status_color = (0, 255, 255) if dbg.pnp_ok else (0, 128, 255)
        if dbg.detection_present and dbg.bbox_w > 1.0 and dbg.bbox_h > 1.0:
            pt1, pt2, _ = self.bbox_fields_to_rect(
                dbg.bbox_cx, dbg.bbox_cy, dbg.bbox_w, dbg.bbox_h, cv_img)
            cv2.rectangle(cv_img, pt1, pt2, status_color, 3)
            self.draw_label(
                cv_img,
                f"MEAS cls={dbg.class_id} pnp={dbg.pnp_reproj_err:.1f}",
                (pt1[0], max(38, pt1[1] - 28)),
                status_color,
                scale=0.5,
            )

        if dbg.pnp_ok:
            pts = []
            for i in range(4):
                x = int(round(dbg.pnp_image_points[2 * i]))
                y = int(round(dbg.pnp_image_points[2 * i + 1]))
                pts.append((x, y))
            for i in range(4):
                cv2.line(cv_img, pts[i], pts[(i + 1) % 4], (255, 0, 255), 1)
                cv2.circle(cv_img, pts[i], 3, (255, 0, 255), -1)

    def draw_aim_pixels(self, cv_img):
        if self.aim_pixels is None or not self.topic_is_fresh(self.aim_pixels_time, 0.25):
            return

        h_img, w_img, _ = cv_img.shape
        aim_x = int(round(self.aim_pixels.linear.x))
        aim_y = int(round(self.aim_pixels.linear.y))
        impact_x = int(round(self.aim_pixels.angular.x))
        impact_y = int(round(self.aim_pixels.angular.y))
        fire = self.aim_pixels.angular.z > 0.5

        if 0 <= impact_x < w_img and 0 <= impact_y < h_img:
            color = (0, 0, 255) if fire else (0, 120, 255)
            cv2.circle(cv_img, (impact_x, impact_y), 5, color, -1)
            cv2.line(cv_img, (impact_x - 18, impact_y), (impact_x + 18, impact_y), color, 2)
            cv2.line(cv_img, (impact_x, impact_y - 18), (impact_x, impact_y + 18), color, 2)
            self.draw_label(cv_img, "IMPACT", (impact_x + 10, max(18, impact_y - 10)), color, 0.45)

        if 0 <= aim_x < w_img and 0 <= aim_y < h_img:
            color = (255, 255, 0)
            sz = 13
            pts = [(aim_x, aim_y - sz), (aim_x + sz, aim_y),
                   (aim_x, aim_y + sz), (aim_x - sz, aim_y)]
            for i in range(4):
                cv2.line(cv_img, pts[i], pts[(i + 1) % 4], color, 2)
            self.draw_label(cv_img, "AIM", (aim_x + 10, max(18, aim_y - 10)), color, 0.45)

    def draw_status_panel(self, cv_img):
        lines = []
        h_img, w_img, _ = cv_img.shape
        lines.append((f"image: OK {w_img}x{h_img}", (0, 255, 0)))

        text, color = self.topic_status("camera_info", self.camera_info_time)
        lines.append((text, color))

        kpt_count = len(self.latest_keypoints.detections) if self.latest_keypoints is not None else 0
        text, color = self.topic_status(
            "keypoints", self.keypoints_time, f"{kpt_count} det")
        lines.append((text, color))

        bbox_count = len(self.latest_detections.detections) if self.latest_detections is not None else 0
        text, color = self.topic_status(
            "bbox fallback", self.detections_time, f"{bbox_count} det")
        lines.append((text, color))

        text, color = self.topic_status("auto_aim/debug", self.debug_time)
        lines.append((text, color))

        text, color = self.topic_status("cmd_gimbal", self.gimbal_cmd_time)
        lines.append((text, color))

        text, color = self.topic_status("aim_pixels", self.aim_pixels_time)
        lines.append((text, color))

        if self.keypoints_time is None and self.detections_time is None:
            lines.append(("detector silent: check yolo26 process, cupy, engine", (0, 0, 255)))
        elif self.keypoints_time is None:
            lines.append(("no keypoints: auto_aim will not track with use_keypoints=true", (0, 128, 255)))

        if self.gimbal_cmd is not None and self.topic_is_fresh(self.gimbal_cmd_time, 1.0):
            cmd = self.gimbal_cmd
            fire_str = "FIRE" if cmd.angular.x > 0.5 else "hold"
            lines.extend([
                (f"cmd yaw {math.degrees(cmd.angular.z):+.1f} deg", (255, 255, 0)),
                (f"cmd pitch {math.degrees(cmd.angular.y):+.1f} deg", (255, 255, 0)),
                (f"dist {cmd.linear.x:.2f} m [{fire_str}]", (255, 255, 0)),
            ])

        if self.latest_debug is not None and self.topic_is_fresh(self.debug_time, 1.0):
            dbg = self.latest_debug
            tracker_states = ["LOST", "DETECT", "TRACK", "TEMP_LOST"]
            state = tracker_states[dbg.tracker_state] if dbg.tracker_state < len(tracker_states) else str(dbg.tracker_state)
            lines.extend([
                (f"state {state} fire {dbg.fire_blocker_reason or 'n/a'}", (255, 255, 0)),
                (f"pnp {'OK' if dbg.pnp_ok else 'BAD'} err {dbg.pnp_reproj_err:.1f}", (255, 255, 0)),
                (f"lat {1000.0 * dbg.latency_total_s:.1f} ms", (255, 255, 0)),
            ])

        for i, (line, color) in enumerate(lines[:12]):
            self.draw_label(cv_img, line, (10, 24 + i * 24), color, 0.55)

    def draw_and_publish(self):
        if self.latest_image is None:
            return

        cv_img = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding="bgr8")

        # 1. YOLOv26-pose detections. Prefer the keypoint topic because this
        # is what auto_aim consumes when use_keypoints=true. Fall back to the
        # compatibility bbox topic if keypoint messages are not available.
        drew_keypoints = self.draw_keypoints(cv_img)
        if (not drew_keypoints and self.latest_detections is not None and
                self.topic_is_fresh(self.detections_time)):
            for det in self.latest_detections.detections:
                pt1, pt2, _ = self.bbox_to_rect(det, cv_img)
                cv2.rectangle(cv_img, pt1, pt2, (0, 255, 0), 2)

                if det.results:
                    class_id = det.results[0].hypothesis.class_id
                    score = det.results[0].hypothesis.score
                    self.draw_label(
                        cv_img, f"ID {class_id} {score:.2f}",
                        (pt1[0], max(18, pt1[1] - 8)), (0, 255, 0))

        # 2. Auto-aim's accepted/diagnostic measurement and PnP corners.
        self.draw_debug_measurement(cv_img)

        # 3. Commanded aim pixel and selected impact pixel. These come from
        # /tracker/aim_pixels, so no TF lookup is required for the new code.
        self.draw_aim_pixels(cv_img)

        # 4. Command/debug text overlay.
        self.draw_status_panel(cv_img)

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
