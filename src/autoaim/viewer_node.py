#!/usr/bin/env python3
"""
viewer_node.py — Debug overlay for the auto-aim pipeline.

Subscribes to:
  /yolo/debug_image       (Image from test.py)
  /detector/armors        (Detection2DArray — YOLO bounding boxes)
  /cmd_vel_AI             (Twist — ABSOLUTE pitch/yaw target in radians + fire command)
  /tracker/aim_pixels     (Twist — aim/impact as 2D pixel coordinates from C++ node)
  /micro_status           (Float32MultiArray — [yaw, pitch, vx, vy, ...])
  /camera_info            (CameraInfo — for fallback projection)

Publishes:
  /tracker/debug_image    (Image — annotated for RViz2)

Important:
  /cmd_vel_AI contains ABSOLUTE target angles.
  The image overlay must use the RELATIVE camera error:
      error = absolute_target - current_micro_angle
  Otherwise an absolute target like +1.2 rad would project offscreen even when
  the target is visible in the camera.
"""

import math
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge

# Optional import: only present once autoaim package is built.
try:
    from auto_aim_3.msg import ArmorKeypointArray
    HAVE_KEYPOINTS = True
except ImportError:
    HAVE_KEYPOINTS = False


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


class ViewerNode(Node):
    def __init__(self):
        super().__init__('auto_aim_viewer')
        self.bridge = CvBridge()

        self.detections = []
        self.keypoint_detections = []  # ArmorKeypoint list, drawn as 4-point polygons

        # Absolute target command from /cmd_vel_AI, in micro convention [rad]
        self.cmd_yaw = 0.0
        self.cmd_pitch = 0.0
        self.cmd_fire = False
        self.cmd_distance = 0.0
        self.tracking = False

        # Current measured micro angles from /micro_status [rad]
        self.micro_yaw = None
        self.micro_pitch = None
        self.micro_vx = 0.0
        self.micro_vy = 0.0
        self.micro_pitch_feedback_opposite_sign = self.declare_parameter("micro_pitch_feedback_opposite_sign", True).value

        # Pixel overlay from C++ node. If missing/stale, viewer computes a fallback
        # from cmd absolute target minus current micro angle.
        self.aim_px = None
        self.imp_px = None
        self.fire_px = False
        self.last_aim_px_time = 0.0
        self.aim_source = "none"

        # Camera intrinsics for fallback projection
        self.cam_fx = None
        self.cam_fy = None
        self.cam_cx = None
        self.cam_cy = None
        self.img_w = 0
        self.img_h = 0

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)

        self.create_subscription(Image, '/yolo/debug_image', self.image_cb, sensor_qos)
        self.create_subscription(Detection2DArray, '/detector/armors', self.det_cb, sensor_qos)
        self.create_subscription(Twist, '/cmd_vel_AI', self.cmd_cb, sensor_qos)
        self.create_subscription(Twist, '/tracker/aim_pixels', self.aim_px_cb, sensor_qos)
        self.create_subscription(Float32MultiArray, '/micro_status', self.micro_status_cb, sensor_qos)
        self.create_subscription(CameraInfo, '/camera_info', self.camera_info_cb, sensor_qos)

        # Keypoint detections — used when the detector publishes 4-point armor
        # keypoints (PnP-from-keypoints mode). Without this the viewer would
        # only show bbox detections from /detector/armors.
        if HAVE_KEYPOINTS:
            self.create_subscription(
                ArmorKeypointArray,
                '/detector/armors_keypoints',
                self.keypoint_cb,
                sensor_qos)
        else:
            self.get_logger().warn(
                "auto_aim_3.msg.ArmorKeypointArray not available — keypoint overlay disabled.")

        self.pub = self.create_publisher(Image, '/tracker/debug_image', 1)
        self.get_logger().info('Viewer started — view /tracker/debug_image in RViz2')

    def camera_info_cb(self, msg):
        self.img_w = int(msg.width)
        self.img_h = int(msg.height)
        if len(msg.k) >= 9:
            self.cam_fx = float(msg.k[0])
            self.cam_fy = float(msg.k[4])
            self.cam_cx = float(msg.k[2])
            self.cam_cy = float(msg.k[5])

    def det_cb(self, msg):
        self.detections = msg.detections

    def keypoint_cb(self, msg):
        self.keypoint_detections = list(msg.detections)

    def micro_status_cb(self, msg):
        if len(msg.data) >= 2:
            self.micro_yaw = float(msg.data[0])
            self.micro_pitch = float(msg.data[1])
        if len(msg.data) >= 4:
            self.micro_vx = float(msg.data[2])
            self.micro_vy = float(msg.data[3])

    def cmd_cb(self, msg):
        self.cmd_fire = msg.angular.x > 0.5
        self.cmd_pitch = float(msg.angular.y)
        self.cmd_yaw = float(msg.angular.z)
        self.cmd_distance = float(msg.linear.x)
        self.tracking = (
            abs(msg.angular.y) > 0.001 or
            abs(msg.angular.z) > 0.001 or
            msg.linear.x > 0.01
        )

    def aim_px_cb(self, msg):
        vals = [msg.linear.x, msg.linear.y, msg.angular.x, msg.angular.y]
        if not all(math.isfinite(v) for v in vals):
            return
        self.aim_px = (int(msg.linear.x), int(msg.linear.y))
        self.imp_px = (int(msg.angular.x), int(msg.angular.y))
        self.fire_px = msg.angular.z > 0.5
        self.last_aim_px_time = time.monotonic()

    def _fallback_aim_px_from_absolute_command(self):
        """Project absolute target into the camera image using target-current.

        This is only a viewer fallback. The micro still receives absolute target angles.
        """
        if self.micro_yaw is None or self.micro_pitch is None:
            return None
        if self.cam_fx is None or self.cam_fy is None or self.cam_cx is None or self.cam_cy is None:
            return None

        yaw_err = wrap_pi(self.cmd_yaw - self.micro_yaw)
        micro_pitch_for_lock = -self.micro_pitch if self.micro_pitch_feedback_opposite_sign else self.micro_pitch
        pitch_err = self.cmd_pitch - micro_pitch_for_lock

        # Camera image convention: positive image x is to the right, while positive
        # gimbal yaw usually means target left in ROS/micro convention, so negate yaw.
        aim_cam_yaw = -yaw_err
        aim_cam_pitch = pitch_err

        # Avoid tan explosion near 90 deg.
        max_angle = math.radians(85.0)
        aim_cam_yaw = max(-max_angle, min(max_angle, aim_cam_yaw))
        aim_cam_pitch = max(-max_angle, min(max_angle, aim_cam_pitch))

        px = self.cam_cx + self.cam_fx * math.tan(aim_cam_yaw)
        py = self.cam_cy - self.cam_fy * math.tan(aim_cam_pitch)
        return int(px), int(py)

    def _draw_aim_marker(self, frame, ax, ay, w, h, source):
        in_frame = 0 <= ax < w and 0 <= ay < h
        dax, day = ax, ay
        if not in_frame:
            dax = int(np.clip(ax, 4, max(4, w - 5)))
            day = int(np.clip(ay, 4, max(4, h - 5)))

        s = 18
        color = (255, 255, 0)
        cv2.line(frame, (dax-s, day), (dax+s, day), color, 2)
        cv2.line(frame, (dax, day-s), (dax, day+s), color, 2)
        d = 12
        pts = np.array([[dax, day-d], [dax+d, day], [dax, day+d], [dax-d, day]], dtype=np.int32)
        cv2.polylines(frame, [pts], True, color, 2)
        label = "AIM" if in_frame else "AIM OFFSCREEN"
        if source:
            label += f" ({source})"
        cv2.putText(frame, label, (min(dax + 16, w - 185), max(day - 8, 18)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    def image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, 'bgra8')
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            except Exception as e:
                self.get_logger().warn(f'Image conversion failed: {e}', throttle_duration_sec=5.0)
                return

        h, w = frame.shape[:2]

        # YOLO bounding boxes
        for det in self.detections:
            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            bw = det.bbox.size_x
            bh = det.bbox.size_y
            x1, y1 = int(cx - bw/2), int(cy - bh/2)
            x2, y2 = int(cx + bw/2), int(cy + bh/2)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
            if det.results:
                label = f"c{det.results[0].hypothesis.class_id} {det.results[0].hypothesis.score:.2f}"
                cv2.putText(frame, label, (x1, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        # 4-point armor keypoints from the keypoint detector.
        # Convention from msg: TL, TR, BR, BL — draw as quadrilateral.
        for kpd in self.keypoint_detections:
            try:
                kp = kpd.keypoints_xy  # length 8
                valid = kpd.keypoint_valid  # length 4
                pts = []
                for i in range(4):
                    if valid[i]:
                        pts.append((int(kp[2*i]), int(kp[2*i + 1])))
                if len(pts) >= 2:
                    # Draw polygon (connect available points)
                    for i in range(len(pts)):
                        p1 = pts[i]
                        p2 = pts[(i + 1) % len(pts)]
                        cv2.line(frame, p1, p2, (255, 0, 255), 2)
                    for p in pts:
                        cv2.circle(frame, p, 3, (255, 255, 255), -1)
                    label = f"kp c{kpd.class_id} {kpd.confidence:.2f}"
                    cv2.putText(frame, label, (pts[0][0], pts[0][1] - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)
            except (AttributeError, IndexError):
                continue

        # Aim point. Prefer C++ /tracker/aim_pixels, but if it is missing/stale,
        # compute it from absolute target minus current micro angle.
        aim_to_draw = None
        aim_source = None
        if self.tracking:
            topic_fresh = self.aim_px is not None and (time.monotonic() - self.last_aim_px_time) < 0.30
            if topic_fresh:
                aim_to_draw = self.aim_px
                aim_source = "topic"
            else:
                fallback = self._fallback_aim_px_from_absolute_command()
                if fallback is not None:
                    aim_to_draw = fallback
                    aim_source = "cmd-now"

        if aim_to_draw is not None:
            ax, ay = aim_to_draw
            self._draw_aim_marker(frame, ax, ay, w, h, aim_source)

        # Impact point from C++: selected armor center / desired bullet impact point.
        if self.tracking and self.imp_px is not None:
            ix, iy = self.imp_px
            if 0 <= ix < w and 0 <= iy < h:
                color = (0, 255, 0) if self.fire_px else (0, 100, 255)
                cv2.circle(frame, (ix, iy), 22, color, 3)
                cv2.drawMarker(frame, (ix, iy), color, cv2.MARKER_TILTED_CROSS, 30, 2)
                label = "FIRE" if self.fire_px else "HOLD"
                cv2.putText(frame, label, (ix+25, iy+5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        # HUD
        lines = []
        if self.tracking:
            lines.append("STATE: TRACKING")
            lines.append(f"Pitch tgt: {self.cmd_pitch:+.3f} rad")
            lines.append(f"Yaw tgt:   {self.cmd_yaw:+.3f} rad")
            if self.micro_yaw is not None and self.micro_pitch is not None:
                yaw_err = wrap_pi(self.cmd_yaw - self.micro_yaw)
                micro_pitch_for_lock = -self.micro_pitch if self.micro_pitch_feedback_opposite_sign else self.micro_pitch
                pitch_err = self.cmd_pitch - micro_pitch_for_lock
                lines.append(f"Yaw now:   {self.micro_yaw:+.3f} rad")
                lines.append(f"Pitch raw: {self.micro_pitch:+.3f} rad")
                lines.append(f"Yaw err:   {yaw_err:+.3f} rad")
                lines.append(f"Pitch err: {pitch_err:+.3f} rad")
                lines.append(f"vx/vy:     {self.micro_vx:+.2f}, {self.micro_vy:+.2f} m/s")
            else:
                lines.append("Micro status: missing")
            if aim_to_draw is None:
                lines.append("AIM: missing")
            lines.append(f"Dist:      {self.cmd_distance:.2f}m")
            if self.cmd_fire:
                lines.append(">>> FIRE <<<")
        else:
            lines.append("STATE: SEARCHING")

        y_off = 25
        for i, line in enumerate(lines):
            y = y_off + i * 22
            (tw, th), _ = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
            cv2.rectangle(frame, (8, y-th-4), (18+tw, y+6), (0, 0, 0), -1)
            color = (0, 255, 0) if 'FIRE' in line else (0, 255, 255)
            cv2.putText(frame, line, (12, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)

        out_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        out_msg.header = msg.header
        self.pub.publish(out_msg)


def main():
    rclpy.init()
    node = ViewerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
