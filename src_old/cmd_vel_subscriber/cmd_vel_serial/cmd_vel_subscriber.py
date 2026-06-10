import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import serial
import time
import threading
import csv
import struct
import numpy as np
from rclpy.qos import qos_profile_system_default


# def format_message(cmd_array):
#     formatted_numbers = []
#     for num in cmd_array:
#         if num >= 100:
#             num = 99.999
#         if num <= -100:
#             num = -99.99
    
#         if num >= 0 and num < 10:
#                 formatted_numbers.append(f"{num:.4f} ")

#         elif num >= 10:
#                 formatted_numbers.append(f"{num:.3f} ")

#         elif num < 0 and num > -10:
#             formatted_numbers.append(f"{num:.3f} ")

#         elif num <= -10:
#             formatted_numbers.append(f"{num:.2f} ")

#     float_string = "".join(formatted_numbers).rstrip()
#     byte_array = float_string.encode('utf-8')
#     return byte_array


def format_float(array):
    
    bts = []
    for num in array:
        for byte in struct.pack('<f',num):
            bts.append(byte)
    
    b=bytearray(bts)

    return b

    
def format_message_to_print(cmd_array):
    formatted_numbers = []
    for num in cmd_array:
        if num >= 100:
            num = 99.999
        if num <= -100:
            num = -99.99
    
        if num >= 0 and num < 10:
                tmp_num=(f"{num:.4f} ")

        elif num >= 10:
                tmp_num=(f"{num:.3f} ")

        elif num < 0 and num > -10:
            tmp_num=(f"{num:.3f} ")

        elif num <= -10:
            tmp_num=(f"{num:.2f} ")
        formatted_numbers.append(float(tmp_num))
    
    return formatted_numbers



class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_vel_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            qos_profile_system_default
        )
        # self.subscription  # prevent unused variable warning
        # file_path = '/workspaces/isaac_ros-dev/data.csv'
        # f = open(file_path, 'a')
        # self.writer = csv.writer(f)
        # self.writer.writerow(['Angular Y', 'Angular Z'])
        self.prev_pitch = 0.0
        self.prev_yaw = 0.0
        self.shoot_frequency = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.x = 0.0
        self.rotation_chassy = 0.0
        self.y = 0.0
        self.prev_time = 0.0
        self.pitch_disp = 0.0
        self.yaw_disp = 0.0
        self.latest_cmd_vel = None
        self.buffer_lock = threading.Lock()
        self.timer = self.create_timer(0.01, self.publish_to_serial)
        self.serial_port = serial.Serial(
            port="/dev/ttyUSB0", #it was "/dev/ttyUSB0"
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        )
        time.sleep(1)
        
        self.micro_publisher = self.create_publisher(
            PoseStamped,
            "/micro_pose",
            10)

        self.get_logger().info("Serial port opened")

    def publish_to_serial(self):

        with self.buffer_lock:
            if self.latest_cmd_vel is not None:
                # Extract values from the buffered cmd_vel message

                # shoot frequency
                self.shoot_frequency = self.latest_cmd_vel.angular.x
                self.pitch =  (self.latest_cmd_vel.angular.y)
                self.yaw =  - (self.latest_cmd_vel.angular.z )
                self.x = self.latest_cmd_vel.angular.y
                self.rotation_chassy = self.latest_cmd_vel.angular.z
                self.y = self.latest_cmd_vel.angular.y
          #      self.writer.writerow(format_message_to_print([angular_y, angular_z]))
                self.latest_cmd_vel = None
            # else:
            #     self.angular_y = 0.0
            #     self.angular_z = 0.0
        # Print a message to indicate that the values have been saved
        # self.get_logger().info("Values saved to CSV file")
        
        formatted_data = format_float(np.array([self.yaw, self.pitch, self.shoot_frequency, 0.0, 0.0], dtype=np.float32))
        #                                        yaw       pitch     shoot_frequency         rotation_chassy         x       y      
        
        self.serial_port.write(formatted_data)
        #self.get_logger().info(f"send pitch: {self.pitch}, yaw: {self.yaw}")
        # self.get_logger().info(f"formatted data {formatted_data}")
            
        var = self.serial_port.read(self.serial_port.in_waiting)
        if len(var) == 20:
            # self.get_logger().info(f"Received: {var}")

            yaw = struct.unpack('<f', bytes(var[:4]))[0]
            pitch = struct.unpack('<f', bytes(var[4:8]))[0]
            yaw2 = struct.unpack('<f', bytes(var[8:12]))[0]
            pitch2 = struct.unpack('<f', bytes(var[12:16]))[0]
            pitch3 = struct.unpack('<f', bytes(var[16:20]))[0]
            
            pose_stamped = PoseStamped()
            pose_stamped.pose.position.x = - pitch
            pose_stamped.pose.position.y = - yaw
            pose_stamped.pose.position.z = 0.0
            self.micro_publisher.publish(pose_stamped)
            #self.get_logger().info(f"Received: pitch {pitch}, yaw {yaw}")

 
        #     self.pitch_disp = float(pitch) + float(self.angular_y)
        #     self.yaw_disp = float(yaw)+(self.angular_z)
            #formatted_data = format_float(np.array([self.yaw_disp, self.pitch_disp], dtype=np.float32))
    

    def listener_callback(self, msg):
        with self.buffer_lock:
            self.latest_cmd_vel = msg
    
            if self.latest_cmd_vel is not None:
                # Extract values from the buffered cmd_vel message

                # shoot frequency
                self.shoot_frequency = self.latest_cmd_vel.angular.x
                self.pitch =  (self.latest_cmd_vel.angular.y)
                self.yaw =  - (self.latest_cmd_vel.angular.z )
                self.x = self.latest_cmd_vel.angular.y
                self.rotation_chassy = self.latest_cmd_vel.angular.z
                self.y = self.latest_cmd_vel.angular.y
          #      self.writer.writerow(format_message_to_print([angular_y, angular_z]))
                self.latest_cmd_vel = None
            # else:
            #     self.angular_y = 0.0
            #     self.angular_z = 0.0
        # Print a message to indicate that the values have been saved
        # self.get_logger().info("Values saved to CSV file")
        
        formatted_data = format_float(np.array([self.yaw, self.pitch, self.shoot_frequency, 0.0, 0.0], dtype=np.float32))
        #                                        yaw       pitch     shoot_frequency         rotation_chassy         x       y      
        
        self.serial_port.write(formatted_data)
        #self.get_logger().info(f"send pitch: {self.pitch}, yaw: {self.yaw}")
        # self.get_logger().info(f"formatted data {formatted_data}")
            
        var = self.serial_port.read(self.serial_port.in_waiting)
        if len(var) == 20:
            # self.get_logger().info(f"Received: {var}")

            yaw = struct.unpack('<f', bytes(var[:4]))[0]
            pitch = struct.unpack('<f', bytes(var[4:8]))[0]
            yaw2 = struct.unpack('<f', bytes(var[8:12]))[0]
            pitch2 = struct.unpack('<f', bytes(var[12:16]))[0]
            pitch3 = struct.unpack('<f', bytes(var[16:20]))[0]
            
            pose_stamped = PoseStamped()
            pose_stamped.pose.position.x = - yaw
            pose_stamped.pose.position.y = - pitch
            pose_stamped.pose.position.z = 0.0
            self.micro_publisher.publish(pose_stamped)
            #self.get_logger().info(f"Received: pitch {pitch}, yaw {yaw}")

 
        #     self.pitch_disp = float(pitch) + float(self.angular_y)
        #     self.yaw_disp = float(yaw)+(self.angular_z)
            #formatted_data = format_float(np.array([self.yaw_disp, self.pitch_disp], dtype=np.float32))



                
    
         
    def vel_integrator(self, w_pitch, w_yaw):
        # Initialize the variables
        # Calculate the time difference
        current_time = time.time()
        time_diff = current_time - self.prev_time
        # Calculate the displacement
        self.pitch_disp += (w_pitch)
        self.yaw_disp += (w_yaw)
        self.prev_time = current_time
        

    def destroy_node(self):
        self.serial_port.close()
        self.serial_port.__del__()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    cmd_vel_subscriber = CmdVelSubscriber()
    rclpy.spin(cmd_vel_subscriber)
    # Shutdown and close serial port properly
    cmd_vel_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
