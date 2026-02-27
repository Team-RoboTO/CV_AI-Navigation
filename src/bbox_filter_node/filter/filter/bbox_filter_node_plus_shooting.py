import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray, Detection2D
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist
import numpy as np
from .library.score_functions import _get_centered, _get_close, _get_wide, _get_fract_sizes

names = {
    0: 'blue_armor',
    1: 'grey_armor',
    2: 'purple_armor',
    3: 'red_armor',
}


class FilterPublisher(Node):

    def __init__(self):
        super().__init__('filter_publisher')
        QUEUE_SIZE = 40

        self.declare_parameter('color_to_shoot', 'blue_armor')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.color_to_shoot = self.get_parameter('color_to_shoot').get_parameter_value().string_value
        self.image_width = self.get_parameter('image_width').get_parameter_value().integer_value
        self.image_height = self.get_parameter('image_height').get_parameter_value().integer_value
        self.IMPROVEMENT = 0.25

        self.prev_score = None
        self.prev_point = None
        self.path = Path()
        self.path.poses = []
        self.path.header.frame_id = 'camera_color_optical_frame'

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.SYSTEM_DEFAULT,
            depth=QUEUE_SIZE
        )

        self._pose_detection_subscriber = self.create_subscription(
            Detection2DArray,
            "/detections_output/with_pose",
            self.detections_callback,
            10
        )

        self._tracking_pose_publisher = self.create_publisher(
            Path,
            "/tracking_pose",
            qos_profile=qos_profile
        )

        self._optimal_bbox_publisher = self.create_publisher(
            Detection2D,
            "/detections_output/optimal_target",
            qos_profile=qos_profile
        )

        self._twist_publisher = self.create_publisher(
            Twist,
            "/cmd_vel",
            qos_profile=qos_profile
        )

    def detections_callback(self, detections_msg: Detection2DArray):
        best_bbox = None
        best_bbox_index = []
        for i in range(len(detections_msg.detections)):
            detection = detections_msg.detections[i]
            score = 0

            if not detection.results:
                continue
            class_id = int(detection.results[0].hypothesis.class_id)
            if class_id not in names:
                continue
            color = names[class_id]

            if color == self.color_to_shoot:
                score += _get_centered(detection, self.image_width, self.image_height)
                score += _get_wide(detection, self.image_width, self.image_height)
                score += _get_fract_sizes(detection)
                best_bbox_index.append((i, score))
                if best_bbox is None or score > best_bbox[0]:
                    best_bbox = (score, detection)

        if len(best_bbox_index) == 0:
            self.prev_score = None
            self.prev_point = None
            if len(self.path.poses) > 5:
                self.path.poses.pop(0)
            empty = Detection2D()
            empty.header.frame_id = 'camera_color_optical_frame'

            self._tracking_pose_publisher.publish(self.path)
            self._optimal_bbox_publisher.publish(empty)

            # Stop gimbal when no target
            twist = Twist()
            self._twist_publisher.publish(twist)
            return

        if self.prev_score is None:
            self.prev_score = best_bbox[0]
            self.prev_point = best_bbox[1].results[0].pose.pose.position
            best_bbox = best_bbox[1]
        else:
            max_score = max(abs(self.prev_score), abs(best_bbox[0]), 1e-6)
            if (self.prev_score - best_bbox[0]) / max_score < self.IMPROVEMENT:
                p1 = np.array([self.prev_point.x, self.prev_point.y, self.prev_point.z])
                dist = 1e9
                box = None
                for i, sc in best_bbox_index:
                    point = detections_msg.detections[i].results[0].pose.pose.position
                    p2 = np.array([point.x, point.y, point.z])
                    new_dist = np.linalg.norm(p1 - p2)

                    if new_dist < dist or box is None:
                        dist = new_dist
                        box = detections_msg.detections[i]
                        self.prev_score = sc

                self.prev_point = box.results[0].pose.pose.position
                best_bbox = box
            else:
                self.prev_score = best_bbox[0]
                self.prev_point = best_bbox[1].results[0].pose.pose.position
                best_bbox = best_bbox[1]

        newpoint = PoseStamped()
        newpoint.header = best_bbox.header
        newpoint.header.frame_id = 'camera_color_optical_frame'
        newpoint.pose.position = best_bbox.results[0].pose.pose.position
        if newpoint.pose.position.x != 0 and newpoint.pose.position.y != 0 and newpoint.pose.position.z != 0:
            self.path.poses.append(newpoint)
        if len(self.path.poses) > 30:
            self.path.poses.pop(0)

        self._optimal_bbox_publisher.publish(best_bbox)
        self._tracking_pose_publisher.publish(self.path)

        # Gimbal proportional controller
        twist = self.calculate_gimbal_command(best_bbox)
        self._twist_publisher.publish(twist)

    def calculate_gimbal_command(self, best_bbox):
        K_YAW = 0.8
        K_PITCH = 0.8
        PITCH_OFFSET = 0.2
        SHOOT_THRESHOLD = 0.15
        DEAD_ZONE_X = 20  # pixels
        DEAD_ZONE_Y = 20  # pixels

        x = best_bbox.bbox.center.position.x
        y = best_bbox.bbox.center.position.y

        center_x = self.image_width / 2.0
        center_y = self.image_height / 2.0

        error_x = x - center_x
        error_y = y - center_y

        if abs(error_x) < DEAD_ZONE_X:
            error_x = 0.0
        if abs(error_y) < DEAD_ZONE_Y:
            error_y = 0.0

        normalized_error_x = error_x / center_x
        normalized_error_y = error_y / center_y

        twist = Twist()
        twist.angular.z = -K_YAW * normalized_error_x
        twist.angular.y = -K_PITCH * normalized_error_y - PITCH_OFFSET
        twist.angular.x = 1.0 if (abs(twist.angular.z) <= SHOOT_THRESHOLD and abs(twist.angular.y) <= SHOOT_THRESHOLD) else 0.0

        return twist


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(FilterPublisher())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
