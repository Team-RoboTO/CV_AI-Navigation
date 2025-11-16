import rclpy
from math import pi 
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.node import Node
from vision_msgs.msg import Detection2D
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu
import numpy as np
from scipy.optimize import minimize
from .library.constants_prediction import g
from .library.prediction_without import PredictionWithout
import time
from .library.score_functions import _get_centered, _get_close, _get_wide, _get_fract_sizes
import queue

# TODO set as input
v_p = 17.0
lag_time = 0.025  # Time delay before shooting (seconds)
num_refs = 1 # number of references for the angles

path = Path()
path.poses = []
path.header.frame_id = 'camera_color_optical_frame'


class ShootPrediction(Node):

    def __init__(self):
        super().__init__('shoot_prediction_without_drag')

        QUEUE_SIZE = 10
        self.theta_0 = None
        self.x0 = self.y0 = self.z0 = None
        self.ax = self.ay = self.az = 0.0
        
        self.wx = 0
        self.wy = 0
        self.wz = 0
        # launch file parameters
        self.declare_parameter('camera_x_displacement', 0.17)
        self.declare_parameter('camera_z_displacement', 0.065)
        self.declare_parameter('camera_y_displacement', 0.00)

        self.camera_x_displacement = self.get_parameter('camera_x_displacement').get_parameter_value().double_value
        self.camera_z_displacement = self.get_parameter('camera_z_displacement').get_parameter_value().double_value
        self.camera_y_displacement = self.get_parameter('camera_y_displacement').get_parameter_value().double_value

        self.predictionWithout = PredictionWithout(v_p, lag_time, num_refs)
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.SYSTEM_DEFAULT,
            depth=QUEUE_SIZE
        )

        # SUBSCRIPTION input - bbox to shoot (Detection2D)
        self._box_to_shoot = self.create_subscription(
            Detection2D, 
            "/detections_output/optimal_target",
            self.detections_callback,
            10)
        
        self.subscription = self.create_subscription(
            Imu,
            '/imu/filtered',

            self.imu_callback,
            10)
            
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            "/micro_pose",
            self.micro_callback,
            10
        )
        # PUBLISHER - best bbox (PoseWithCovariance)
        self._predicted_shoot_publisher = self.create_publisher(
            Path, 
            "/predicted_shoot",
            qos_profile=qos_profile
        )

        # PUBLISHER - angular references (Twist)
        self._twist_publisher = self.create_publisher(
            Twist, 
            "/cmd_vel",
            qos_profile=qos_profile
        )

        self.prev_time = time.time()
        self.twist_queue = queue.Queue()
        # self.timer = self.create_timer(0.01, self.timer_publisher)

    def imu_callback(self, msg: Imu):
        #self.theta_0 = (np.arctan2(-msg.linear_acceleration.z, msg.linear_acceleration.y))
        #if self.theta_0 >= 0 : self.theta_0=-(self.theta_0-pi)
        #elif self.theta_0 < 0 : self.theta_0=-(self.theta_0+pi)
        self.wx = msg.angular_velocity.z
        self.wy = - msg.angular_velocity.x
        self.wz = - msg.angular_velocity.y
        #print(f"theta imu_callback {self.theta_0}")
        

    
    def micro_callback(self, msg: PoseStamped):
        self.theta_0 =  msg.pose.position.y
        #self.get_logger().info(f"theta {self.theta_0}")   
    ##    #print(f"theta {self.theta_0}")

        
    # def timer_publisher(self):
    #     if self.twist_queue.qsize() > 0:
    #         twist = self.twist_queue.get()
    #         self._twist_publisher.publish(twist)

        
    def detections_callback(self, detection: Detection2D):
        
        x = detection.bbox.center.position.x
        y = detection.bbox.center.position.y
        width = detection.bbox.size_x  # width
        height = detection.bbox.size_y # height
        th = detection.bbox.center.theta
        if (x == 0.0 and y == 0.0 and width == 0.0 and height == 0.0 and th == 0.0):
            twist: Twist = Twist()
            twist.angular.x = 0.0 # SHOOT FREQUENCY
            twist.angular.z = 0.0
            twist.angular.y = 0.0
            self._twist_publisher.publish(twist)
            #print(f'ciao1')
            return
        
        # start_node = time.time()
        # self.get_logger().info(f"Time since last detection: {start_node - self.prev_time}")

        if len(detection.results) == 0 or self.theta_0 is None:
            # self.get_logger().info("no detection")
            return

        self.x0 =  detection.results[0].pose.pose.position.z + self.camera_x_displacement # x
        self.y0 =  detection.results[0].pose.pose.position.x  + self.camera_y_displacement  # y
        self.z0 = detection.results[0].pose.pose.position.y + self.camera_z_displacement # z

        # x is the front direction, y towards right, and z below
        # rotate by -theta_0 in the y-axis. This makes gravity work properly on the vertical dimension
        # theta_0 is the rotation which is received from the imu
        tempx0 = self.x0
        tempz0 = self.z0
        self.x0 = tempx0 * np.cos(self.theta_0) - tempz0 * np.sin(self.theta_0)
        self.z0 = + tempx0 * np.sin(self.theta_0) + tempz0 * np.cos(self.theta_0)
        #print(f"theta {self.theta_0}")
        # self.get_logger().info(f"theta{self.theta_0}")  
        # print(f"theta{self.theta_0}")
        # self.get_logger().info(f"original accelleration AX: {self.ax}, AY: {self.ay}, , AZ: {self.az}")  
        # acceleration transformation

        # self.get_logger().info(f"AX: {self.ax}, AY: {self.ay}, , AZ: {self.az}")        


        # ts = detection.header.stamp_nsec
        ts = detection.header.stamp.nanosec

        wht_avg_t, wht_avg_p, t_hit, _ = self.predictionWithout.prediction_without(self.x0, (self.y0), (self.z0),self.wz, ts)
        # I guess theta_ref should be brought back to the camera frame (theta_0) for absolute angles

        if wht_avg_t == 0 and wht_avg_p == 0 and t_hit == 0: return

        bbox_distance = _get_close(detection) * (-7)
        bbox_fract = _get_fract_sizes(detection) * 5

        SHOOT_GAIN = 1.05
        hz_shoot_dist = 3 - 3/13.6*bbox_distance
        hz_shoot_dist = max(hz_shoot_dist, 0.0)
        hz_shoot = hz_shoot_dist * min(1, bbox_fract * SHOOT_GAIN)
        print(f'shoot {hz_shoot}')
        if (detection.header.frame_id == 'NO_DETECTION'): hz_shoot = 0.0

        twist: Twist = Twist()
        twist.angular.x = 1.0 #hz_shoot #SHOOT FREQUENCY
        twist.angular.z = -wht_avg_p 
        twist.angular.y = (wht_avg_t - self.theta_0)

        self._twist_publisher.publish(twist)
        print(f'x: {twist.angular.x} y: {twist.angular.y} z: {twist.angular.z}')


        position = self.predictionWithout.projectile_position_without(self.predictionWithout.wth_avg_t, self.predictionWithout.wth_avg_p, t_hit, self.theta_0)

        newpoint = PoseStamped()
        newpoint.header.frame_id = 'camera_color_optical_frame'
        newpoint.pose.position.z = position[0]
        newpoint.pose.position.x = - position[1]
        newpoint.pose.position.y = - position[2]
    
        path.poses.append(newpoint)
        if len(path.poses) > 30: 
            # self.get_logger().info("len path poses > 30")
            path.poses.pop(0)

        self._predicted_shoot_publisher.publish(path)
        

        end_node = time.time()
        self.prev_time = end_node
        # self.get_logger().info(f"Node execution time: {end_node - start_node}")





def main(args=None):
    # print("Sto ascoltando...")
    
    rclpy.init(args=args)
    rclpy.spin(ShootPrediction())
    rclpy.shutdown()


if __name__ == '__main__':
    main()

