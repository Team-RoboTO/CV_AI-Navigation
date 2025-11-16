import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np



class IMUFilter:

    def __init__(self, accelerometer_noise_density, accelerometer_random_walk, gyroscope_noise_density, gyroscope_random_walk, sampling_frequency):
        self.accelerometer_noise_density = accelerometer_noise_density
        self.accelerometer_random_walk = accelerometer_random_walk
        self.gyroscope_noise_density = gyroscope_noise_density
        self.gyroscope_random_walk = gyroscope_random_walk
        self.dt = 1/sampling_frequency
        self.accelerometer_bias_prev = np.zeros(3)
        self.gyroscope_bias_prev = np.zeros(3)

    def filter(self, lin_acc_noisy: np.ndarray, ang_vel_noisy: np.ndarray):
        # Linear acceleration filtering
        accelerometer_noise = lin_acc_noisy * self.accelerometer_noise_density / np.sqrt(self.dt)
        accelerometer_bias = self.accelerometer_bias_prev + lin_acc_noisy * self.accelerometer_random_walk * np.sqrt(self.dt)
        lin_acc_filtered = lin_acc_noisy - accelerometer_bias - accelerometer_noise
        self.accelerometer_bias_prev = accelerometer_bias

        # Angular velocity filtering
        gyroscope_noise = ang_vel_noisy * self.gyroscope_noise_density / np.sqrt(self.dt)
        gyroscope_bias = self.gyroscope_bias_prev + ang_vel_noisy * self.gyroscope_random_walk * np.sqrt(self.dt)
        ang_vel_filtered = ang_vel_noisy - gyroscope_bias - gyroscope_noise
        self.gyroscope_bias_prev = gyroscope_bias

        return lin_acc_filtered, ang_vel_filtered

class IMUFilterNode(Node):

    def __init__(self):
        super().__init__('imu_filter_node')

        self.declare_parameter('accelerometer_noise_density', 0.02)
        self.declare_parameter('accelerometer_random_walk', 0.001)
        self.declare_parameter('gyroscope_noise_density', 0.01)
        self.declare_parameter('gyroscope_random_walk', 0.0001)
        self.declare_parameter('sampling_frequency', 200.0)

        accelerometer_noise_density = self.get_parameter('accelerometer_noise_density').get_parameter_value().double_value
        accelerometer_random_walk = self.get_parameter('accelerometer_random_walk').get_parameter_value().double_value
        gyroscope_noise_density = self.get_parameter('gyroscope_noise_density').get_parameter_value().double_value
        gyroscope_random_walk = self.get_parameter('gyroscope_random_walk').get_parameter_value().double_value
        sampling_frequency = self.get_parameter('sampling_frequency').get_parameter_value().double_value

        self.filter = IMUFilter(accelerometer_noise_density, accelerometer_random_walk, gyroscope_noise_density, gyroscope_random_walk, sampling_frequency)
        self.subscription = self.create_subscription(
            Imu,
            '/imu',
            self.listener_callback,
            10) 
        self.publisher = self.create_publisher(Imu, '/imu/filtered', 10)

    def listener_callback(self, msg):
        lin_acc_noisy = np.array([-msg.linear_acceleration.x, -msg.linear_acceleration.y, -msg.linear_acceleration.z])
        ang_vel_noisy = np.array([-msg.angular_velocity.x, -msg.angular_velocity.y, -msg.angular_velocity.z])
        
        lin_acc_filtered, ang_vel_filtered = self.filter.filter(lin_acc_noisy, ang_vel_noisy)

        filtered_msg = Imu()
        filtered_msg.header = msg.header
        filtered_msg.header.frame_id = 'camera_color_optical_frame'
        # filtered_msg.linear_acceleration.x = lin_acc_filtered[0]
        # filtered_msg.linear_acceleration.y = lin_acc_filtered[1]
        # filtered_msg.linear_acceleration.z = lin_acc_filtered[2]
        # filtered_msg.angular_velocity.x = ang_vel_filtered[0]
        # filtered_msg.angular_velocity.y = ang_vel_filtered[1]
        # filtered_msg.angular_velocity.z = ang_vel_filtered[2]
        filtered_msg.linear_acceleration.x = - msg.linear_acceleration.x
        filtered_msg.linear_acceleration.y = - msg.linear_acceleration.y
        filtered_msg.linear_acceleration.z = - msg.linear_acceleration.z
        filtered_msg.angular_velocity.x = - msg.angular_velocity.x
        filtered_msg.angular_velocity.y = - msg.angular_velocity.y
        filtered_msg.angular_velocity.z = - msg.angular_velocity.z
        filtered_msg.orientation = msg.orientation  # assuming orientation is not filtered

        self.publisher.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IMUFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
