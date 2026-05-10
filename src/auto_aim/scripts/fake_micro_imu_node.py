#!/usr/bin/env python3
"""Fake /micro_imu publisher for static bench and video replay."""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class FakeMicroImuNode(Node):
    def __init__(self):
        super().__init__("fake_micro_imu")

        self.declare_parameter("mode", "video_motion")
        self.declare_parameter("imu_topic", "/micro_imu")
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("rate_hz", 120.0)
        self.declare_parameter("yaw_rad", 0.0)
        self.declare_parameter("pitch_rad", 0.0)
        self.declare_parameter("publish_log_period_s", 5.0)
        self.declare_parameter("video_hfov_deg", 90.0)
        self.declare_parameter("video_vfov_deg", 58.0)
        self.declare_parameter("video_resize_width", 640)
        self.declare_parameter("video_top_mask_px", 110)
        self.declare_parameter("video_bottom_mask_px", 95)
        self.declare_parameter("video_left_mask_px", 0)
        self.declare_parameter("video_right_mask_px", 0)
        self.declare_parameter("video_max_corners", 300)
        self.declare_parameter("video_process_every_n", 2)
        self.declare_parameter("video_min_inliers", 40)
        self.declare_parameter("video_min_inlier_ratio", 0.35)
        self.declare_parameter("video_max_step_rad", 0.20)
        self.declare_parameter("video_smoothing_alpha", 0.85)
        self.declare_parameter("video_yaw_sign", -1.0)
        self.declare_parameter("video_pitch_sign", 1.0)

        self.mode = str(self.get_parameter("mode").value)
        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.rate_hz = max(1.0, float(self.get_parameter("rate_hz").value))
        self.yaw = float(self.get_parameter("yaw_rad").value)
        self.pitch = float(self.get_parameter("pitch_rad").value)
        self.publish_log_period = float(self.get_parameter("publish_log_period_s").value)
        self.video_hfov = math.radians(float(self.get_parameter("video_hfov_deg").value))
        self.video_vfov = math.radians(float(self.get_parameter("video_vfov_deg").value))
        self.video_resize_width = int(self.get_parameter("video_resize_width").value)
        self.video_top_mask_px = int(self.get_parameter("video_top_mask_px").value)
        self.video_bottom_mask_px = int(self.get_parameter("video_bottom_mask_px").value)
        self.video_left_mask_px = int(self.get_parameter("video_left_mask_px").value)
        self.video_right_mask_px = int(self.get_parameter("video_right_mask_px").value)
        self.video_max_corners = int(self.get_parameter("video_max_corners").value)
        self.video_process_every_n = max(
            1, int(self.get_parameter("video_process_every_n").value)
        )
        self.video_min_inliers = int(self.get_parameter("video_min_inliers").value)
        self.video_min_inlier_ratio = float(self.get_parameter("video_min_inlier_ratio").value)
        self.video_max_step_rad = float(self.get_parameter("video_max_step_rad").value)
        self.video_smoothing_alpha = max(
            0.0, min(1.0, float(self.get_parameter("video_smoothing_alpha").value))
        )
        self.video_yaw_sign = float(self.get_parameter("video_yaw_sign").value)
        self.video_pitch_sign = float(self.get_parameter("video_pitch_sign").value)

        if self.mode not in ("static", "video_motion"):
            raise RuntimeError(
                "fake_micro_imu mode must be 'static' or 'video_motion'"
            )

        self.pub = self.create_publisher(Float32MultiArray, self.imu_topic, 10)
        self.last_timer_time = self.get_clock().now()
        self.last_log_time = self.last_timer_time
        self.prev_gray = None
        self.frame_index = 0
        self.last_video_conf = 0.0

        if self.mode == "video_motion":
            import cv2  # pylint: disable=import-outside-toplevel
            from cv_bridge import CvBridge  # pylint: disable=import-outside-toplevel
            from sensor_msgs.msg import Image  # pylint: disable=import-outside-toplevel

            self.cv2 = cv2
            self.bridge = CvBridge()
            self.image_sub = self.create_subscription(
                Image, self.image_topic, self._image_cb, 10
            )
        else:
            self.cv2 = None
            self.bridge = None
            self.image_sub = None

        self.timer = self.create_timer(1.0 / self.rate_hz, self._timer_cb)
        self.get_logger().info(
            "Fake micro IMU: mode=%s yaw=%.3f pitch=%.3f imu=%s image=%s"
            % (
                self.mode,
                self.yaw,
                self.pitch,
                self.imu_topic,
                self.image_topic,
            )
        )

    def _timer_cb(self):
        now = self.get_clock().now()
        self.last_timer_time = now

        msg = Float32MultiArray()
        msg.data = [float(self.yaw), float(self.pitch)]
        self.pub.publish(msg)

        if self.publish_log_period > 0.0:
            age = (now - self.last_log_time).nanoseconds * 1e-9
            if age >= self.publish_log_period:
                self.last_log_time = now
                self.get_logger().info(
                    "fake imu yaw=%.3f pitch=%.3f video_conf=%.2f"
                    % (
                        self.yaw,
                        self.pitch,
                        self.last_video_conf,
                    )
                )

    def _image_cb(self, msg):
        if self.mode != "video_motion":
            return
        try:
            if self.frame_index % self.video_process_every_n != 0:
                self.frame_index += 1
                return
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            gray = self._prepare_gray(frame)
            if self.prev_gray is not None:
                self._integrate_video_motion(self.prev_gray, gray)
            self.prev_gray = gray
            self.frame_index += 1
            out = Float32MultiArray()
            out.data = [float(self.yaw), float(self.pitch)]
            self.pub.publish(out)
        except Exception as exc:  # pragma: no cover - runtime ROS diagnostic
            self.get_logger().warn(
                "video_motion fake IMU failed: %s" % exc,
                throttle_duration_sec=2.0,
            )

    def _prepare_gray(self, frame):
        cv2 = self.cv2
        h, w = frame.shape[:2]
        if self.video_resize_width > 0 and w > self.video_resize_width:
            scale = float(self.video_resize_width) / float(w)
            frame = cv2.resize(
                frame,
                (self.video_resize_width, int(round(h * scale))),
                interpolation=cv2.INTER_AREA,
            )
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return cv2.GaussianBlur(gray, (5, 5), 0)

    def _video_mask(self, shape):
        import numpy as np  # pylint: disable=import-outside-toplevel

        h, w = shape[:2]
        mask = np.full((h, w), 255, dtype=np.uint8)
        sx = w / 1280.0
        sy = h / 720.0
        top = int(max(0, self.video_top_mask_px) * sy)
        bottom = int(max(0, self.video_bottom_mask_px) * sy)
        left = int(max(0, self.video_left_mask_px) * sx)
        right = int(max(0, self.video_right_mask_px) * sx)
        if top > 0:
            mask[: min(top, h), :] = 0
        if bottom > 0:
            mask[max(0, h - bottom):, :] = 0
        if left > 0:
            mask[:, : min(left, w)] = 0
        if right > 0:
            mask[:, max(0, w - right):] = 0
        return mask

    def _integrate_video_motion(self, prev_gray, gray):
        cv2 = self.cv2
        import numpy as np  # pylint: disable=import-outside-toplevel

        mask = self._video_mask(prev_gray.shape)
        pts0 = cv2.goodFeaturesToTrack(
            prev_gray,
            maxCorners=max(50, self.video_max_corners),
            qualityLevel=0.01,
            minDistance=9,
            blockSize=7,
            mask=mask,
        )
        if pts0 is None or len(pts0) < 20:
            self.last_video_conf = 0.0
            return

        pts1, status, _ = cv2.calcOpticalFlowPyrLK(
            prev_gray,
            gray,
            pts0,
            None,
            winSize=(21, 21),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
        )
        if pts1 is None or status is None:
            self.last_video_conf = 0.0
            return
        good = status.reshape(-1) == 1
        p0 = pts0.reshape(-1, 2)[good]
        p1 = pts1.reshape(-1, 2)[good]
        if len(p0) < 20:
            self.last_video_conf = 0.0
            return

        affine, inliers = cv2.estimateAffinePartial2D(
            p0,
            p1,
            method=cv2.RANSAC,
            ransacReprojThreshold=3.0,
            maxIters=1200,
            confidence=0.995,
            refineIters=10,
        )
        if affine is None or inliers is None:
            self.last_video_conf = 0.0
            return
        inlier_count = int(np.sum(inliers))
        inlier_ratio = float(inlier_count) / float(max(1, len(p0)))
        self.last_video_conf = inlier_ratio
        if (
            inlier_count < self.video_min_inliers or
            inlier_ratio < self.video_min_inlier_ratio
        ):
            return

        h, w = gray.shape[:2]
        fx = w / (2.0 * math.tan(max(self.video_hfov, 1e-3) * 0.5))
        fy = h / (2.0 * math.tan(max(self.video_vfov, 1e-3) * 0.5))
        dx = float(affine[0, 2])
        dy = float(affine[1, 2])
        raw_dyaw = self.video_yaw_sign * dx / max(fx, 1e-6)
        raw_dpitch = self.video_pitch_sign * dy / max(fy, 1e-6)
        if not (math.isfinite(raw_dyaw) and math.isfinite(raw_dpitch)):
            return

        limit = max(1e-4, self.video_max_step_rad)
        raw_dyaw = max(-limit, min(limit, raw_dyaw))
        raw_dpitch = max(-limit, min(limit, raw_dpitch))
        a = self.video_smoothing_alpha
        self.yaw = math.atan2(
            math.sin(self.yaw + a * raw_dyaw),
            math.cos(self.yaw + a * raw_dyaw),
        )
        self.pitch += a * raw_dpitch


def main():
    rclpy.init()
    node = FakeMicroImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
