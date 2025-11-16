import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D
import message_filters
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist
import numpy as np
from .library.score_functions import _get_centered, _get_close, _get_wide, _get_fract_sizes
# import time

names = {
        0: 'blue_armor',
        1: 'grey_armor',
        2: 'purple_armor',
        3: 'red_armor',
        #4: 'robot',
}

path = Path()
path.poses = []
path.header.frame_id = 'camera_color_optical_frame'

prev_score = None
prev_point = None 

class FilterPublisher(Node):
    
    def __init__(self):
        super().__init__('filter_publisher')
        QUEUE_SIZE = 40

        self.declare_parameter('color_to_shoot', 'blue_armor')
        self.color_to_shoot = self.get_parameter('color_to_shoot').get_parameter_value().string_value

        # TODO THESE VALUES MUST BY PASSED AS PARAMETERS WHEN
        # CALLING ALL THIS WITH 'ros2 run filter talker' + params
        self.image_width = 640
        self.image_height = 480
        self.confidence_threshold = -2
        self.IMPROVEMENT = 0.25

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.SYSTEM_DEFAULT,
            depth=QUEUE_SIZE
        )
 
        # SUBSCRIPTION input - image (Image)
        # self._image_subscriber = message_filters.Subscriber(
        #     self,
        #     Image,
        #     "/image", 
        # )

        # SUBSCRIPTION input - array of bbox (Detection2DArray)
        self._pose_detection_subscriber = self.create_subscription(
            Detection2DArray,
            "/detections_output/with_pose",
            self.detections_callback,
            10
        )
                
        # PUBLISHER - best bbox (PoseWithCovariance)
        self._tracking_pose_publisher = self.create_publisher(
            Path, 
            "/tracking_pose",
            qos_profile=qos_profile
        )

        # PUBLISHER - best bbox (PoseWithCovariance)
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

        # self.time_synchronizer = message_filters.TimeSynchronizer(
        #     [self._pose_detection_subscriber],
        #     QUEUE_SIZE
        # )
        
        # self.time_synchronizer.registerCallback(self.detections_callback)
    


    def detections_callback(self, detections_msg: Detection2DArray):
        global path, prev_score, prev_point

        # ##################################
        # time_now = time.time()

        best_bbox = None
        best_bbox_index = []
        for i in range(len(detections_msg.detections)):
            detection = detections_msg.detections[i]
            score = 0
            
            # color should be only BLUE or RED
            color = names[int(detection.results[0].hypothesis.class_id)]

            if color == self.color_to_shoot:
                score += _get_close(detection)
                score += _get_centered(detection)
                score += _get_wide(detection)
                score += _get_fract_sizes(detection)
                best_bbox_index.append((i,score))
                if best_bbox is None or score > best_bbox[0]:
                    best_bbox = (score, detection)


        if len(best_bbox_index) == 0:
            # print("no bbox found")
            prev_score = None
            prev_point = None
            if len(path.poses) > 5: path.poses.pop(0)
            best_bbox = Detection2D()
            best_bbox.header.frame_id = 'camera_color_optical_frame'
            
            self._tracking_pose_publisher.publish(path)
            self._optimal_bbox_publisher.publish(best_bbox)
            return
        
        else:
            if prev_score is None:
                prev_score = best_bbox[0]
                prev_point = best_bbox[1].results[0].pose.pose.position
                best_bbox = best_bbox[1]
            else:
                if (prev_score - best_bbox[0]) / max(prev_score, best_bbox[0]) < self.IMPROVEMENT:
                    p1 = np.array([prev_point.x, prev_point.y, prev_point.z])
                    dist = 1e9
                    box = None
                    for i, sc in best_bbox_index:
                        point = detections_msg.detections[i].results[0].pose.pose.position
                        p2 = np.array([point.x, point.y, point.z])
                        squared_dist = np.sum((p1-p2)**2, axis=0)
                        new_dist = np.sqrt(squared_dist)

                        if new_dist < dist or box is None:
                            dist = new_dist
                            box = detections_msg.detections[i]
                            prev_score = sc
                    
                    prev_point = box.results[0].pose.pose.position
                    best_bbox = box

                else:
                    prev_score = best_bbox[0]
                    prev_point = best_bbox[1].results[0].pose.pose.position
                    best_bbox = best_bbox[1]
                
                

            newpoint = PoseStamped()
            newpoint.header = best_bbox.header
            newpoint.header.frame_id = 'camera_color_optical_frame'
            newpoint.pose.position = best_bbox.results[0].pose.pose.position
            if newpoint.pose.position.x != 0 and newpoint.pose.position.y != 0 and newpoint.pose.position.z != 0:
                path.poses.append(newpoint)
            if len(path.poses) > 30: path.poses.pop(0)


            self._optimal_bbox_publisher.publish(best_bbox)
            self._tracking_pose_publisher.publish(path)

        #Calculations for the angles to be returned to the gimbal
        if best_bbox is not None:
            # Calculate and publish twist command
            twist = self.calculate_gimbal_command(best_bbox)
            self._twist_publisher.publish(twist)
        else:
            # No target detected - stop movement
            twist = Twist()
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self._twist_publisher.publish(twist)
        # ###################################################
        # self.get_logger().info(f'{time.time() - time_now}')

        # GIMBAL calculation for the angles to center the armor, this value is returned to the micro

    def calculate_gimbal_command(self, best_bbox):
        """
        Convert Detection2D bbox to Twist command for gimbal control
        """
        # Camera parameters
        IMAGE_WIDTH = 640
        IMAGE_HEIGHT = 480
        
        # Control gains (tune these for your system)
        K_YAW = 0.8     # Horizontal movement sensitivity
        K_PITCH = 0.8   # Vertical movement sensitivity
        PITCH_OFFSET = 0.2
        SHOOT_THRESHOLD = 0.15
        
        # Dead zone (don't move if target is close enough to center)
        DEAD_ZONE_X = 20  # pixels
        DEAD_ZONE_Y = 20  # pixels
        
        # Extract bbox data
        x = best_bbox.bbox.center.position.x
        y = best_bbox.bbox.center.position.y
        width = best_bbox.bbox.size_x
        height = best_bbox.bbox.size_y
        
        # Calculate center of image
        center_x = IMAGE_WIDTH / 2.0
        center_y = IMAGE_HEIGHT / 2.0
        
        # Calculate errors
        error_x = x - center_x
        error_y = y - center_y
        
        # Apply dead zone
        if abs(error_x) < DEAD_ZONE_X:
            error_x = 0.0
        if abs(error_y) < DEAD_ZONE_Y:
            error_y = 0.0
        
        # Normalize errors to [-1, 1]
        normalized_error_x = error_x / center_x
        normalized_error_y = error_y / center_y
        
        # Create Twist message
        twist = Twist()
        twist.angular.z = -K_YAW * normalized_error_x      # Yaw
        twist.angular.y = -K_PITCH * normalized_error_y - PITCH_OFFSET   # Pitch
        twist.angular.x = 1.0 if (abs(twist.angular.z ) <= SHOOT_THRESHOLD and abs(twist.angular.y ) <= SHOOT_THRESHOLD) else 0.0  # Shoot when centered

        
        return twist


def main(args=None):
    # print("Sto ascoltando...")
    
    rclpy.init(args=args)
    rclpy.spin(FilterPublisher())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
