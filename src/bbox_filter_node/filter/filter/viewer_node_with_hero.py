import cv2
import cv_bridge
import message_filters
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D
# Assuming these imports exist in your library structure
from .library.score_functions import _get_centered, _get_close, _get_wide, _get_fract_sizes

names = {
        0: 'blue_armor',
        1: 'grey_armor',    
        2: 'purple_armor',
        3: 'red_armor',
        4: 'robot',
}

class BboxWithDistance(Node):
    QUEUE_SIZE = 10
    color = (0, 255, 0)      # Green (All)
    color_opt = (0, 0, 255)  # Red (Optimal)
    color_dot_aim = (255, 255, 0) # Cyan (Hero Aim)
    color_dot_hit = (0, 165, 255) # Orange (Hero Impact)
    bbox_thickness = 2

    # Hero Ballistics Parameters
    BULLET_SPEED = 18.0 # m/s
    GRAVITY = 9.81
    PIXEL_SCALE_FACTOR = 800.0 # Tune this value!

    def __init__(self):
        super().__init__('bbox_viewer')
        self._bridge = cv_bridge.CvBridge()

        # 1 PUBLISHER: Every box detected is visible
        self._processed_image_all = self.create_publisher(
            Image,
            '/yolov8_processed/all', 
            self.QUEUE_SIZE)

        # 2 PUBLISHER: Only the optimal target is visible
        self._processed_image_optimal = self.create_publisher(
            Image,
            '/yolov8_processed/optimal',
            self.QUEUE_SIZE)
        
        # 3 PUBLISHER: Center and hit point publisher for Hero
        self._processed_image_ballistic = self.create_publisher(
            Image, 
            '/yolov8_processed/ballistics', 
            self.QUEUE_SIZE)

        # 1 SUBSCRIBER: Takes the informations from the detections
        self._detections_subscription = message_filters.Subscriber(
            self,
            Detection2DArray,
            '/detections_output/with_pose')
        
        # 2 SUBSCRIBER: Takes the image
        self._image_subscription = message_filters.Subscriber(
            self,
            Image,
            '/image')

        # 3 SUBSCRIBER: Takes the Optimal Target
        self._optimal_subscription = message_filters.Subscriber(
            self,
            Detection2D, 
            '/detections_output/optimal_target'
        )

        # Synchronizer of the 3 subscribers
        self.time_synchronizer = message_filters.TimeSynchronizer(
            [self._detections_subscription, self._image_subscription, self._optimal_subscription],
            self.QUEUE_SIZE)

        self.time_synchronizer.registerCallback(self.detections_callback)

    def calculate_pixel_drop(self, distance_z):
        if distance_z <= 0: return 0
        
        # Time to target
        t = distance_z / self.BULLET_SPEED
        
        # Drop in meters (Real world vertical drop)
        drop_meters = 0.5 * self.GRAVITY * (t ** 2)
        
        # Convert Drop (meters) to Drop (pixels)
        drop_pixels = (drop_meters * self.PIXEL_SCALE_FACTOR) / distance_z
        
        return int(drop_pixels)

    def detections_callback(self, detections_msg, img_msg, optimal_msg):
        # Color Setup
        color_txt = (255, 0, 255)

        # Image conversion (from ROS to OpenCV)
        # REMOVED: cv2.flip logic
        cv2_img = self._bridge.imgmsg_to_cv2(img_msg)
        
        img_height = cv2_img.shape[0]  # Altezza in pixel 
        img_width = cv2_img.shape[1]   # Larghezza in pixel 

        # Generation of three "canvas" for the three "channels"
        # Using cv2_img directly instead of the flipped version
        img_all = cv2_img.copy()
        img_optimal = cv2_img.copy()
        img_hit_point = cv2_img.copy() # The "Hero" View

        # Optimal Target raw coordinates
        opt_x = optimal_msg.bbox.center.position.x
        opt_y = optimal_msg.bbox.center.position.y

        for detection in detections_msg.detections:

            # Single detection raw coordinates
            center_x = detection.bbox.center.position.x
            center_y = detection.bbox.center.position.y

            width = detection.bbox.size_x
            height = detection.bbox.size_y
            
            # Distance Z from Pose
            dist_z = detection.results[0].pose.pose.position.z

            # CHANGED: Use raw coordinates directly (No inversion needed)
            draw_center_x = center_x
            draw_center_y = center_y

            s_x = int(width*0.7/2)
            s_y = int(height*0.7/2)

            label_id = int(detection.results[0].hypothesis.class_id)
            label = names.get(label_id, str(label_id))
            conf_score = detection.results[0].hypothesis.score
            label_text = f'{label} {conf_score:.2f}'
            coordinates = f'x:{center_x:.2f} y:{center_y:.2f} z:{dist_z:.2f}'
            
            # Computation of the rectangle vertices using standard coordinates
            min_pt = (round(draw_center_x - (width / 2.0)), round(draw_center_y - (height / 2.0)))
            max_pt = (round(draw_center_x + (width / 2.0)), round(draw_center_y + (height / 2.0)))
        
            new_min_pt = (round(draw_center_x - s_x), round(draw_center_y - s_y))
            new_max_pt = (round(draw_center_x + s_x), round(draw_center_y + s_y))

            lw = max(round((img_height + img_width) / 2 * 0.003), 2)
            tf = max(lw - 1, 1)
            
            # Drawing on the "All" channel (every detected box is green)
            cv2.rectangle(img_all, min_pt, max_pt, self.color, self.bbox_thickness)
            cv2.rectangle(img_all, new_min_pt, new_max_pt, self.color, self.bbox_thickness)
            cv2.putText(img_all, label_text, (min_pt[0], min_pt[1]-2), 0, lw / 3, color_txt, thickness=tf, lineType=cv2.LINE_AA)
            cv2.putText(img_all, coordinates, max_pt, 0, lw / 3, color_txt, thickness=tf, lineType=cv2.LINE_AA)

            # Check if Optimal
            is_optimal = abs(center_x - opt_x) < 0.001 and abs(center_y - opt_y) < 0.001

            if is_optimal:
                # --- Drawing on "Optimal" Channel ---
                cv2.rectangle(img_optimal, min_pt, max_pt, self.color_opt, 3)
                cv2.putText(img_optimal, f"TARGET {label_text}", (min_pt[0], min_pt[1]-2), 
                            0, lw / 3, self.color_opt, thickness=tf, lineType=cv2.LINE_AA)
                cv2.putText(img_optimal, coordinates, max_pt, 
                            0, lw / 3, color_txt, thickness=tf, lineType=cv2.LINE_AA)

                # --- Drawing on "Hero Ballistics" Channel ---
                # 1. Draw the box of the target
                cv2.rectangle(img_hit_point, min_pt, max_pt, self.color_opt, 2)
                
                # 2. Draw Center Point (Aim)
                center_pt = (int(draw_center_x), int(draw_center_y))
                cv2.circle(img_hit_point, center_pt, 5, self.color_dot_aim, -1)
                
                # 3. Calculate Drop
                pixel_drop = self.calculate_pixel_drop(dist_z)
                
                # 4. Draw Impact Point
                # In standard image coords (Top-Left origin), +Y moves DOWN.
                # So we ADD pixel_drop to move the impact point lower (gravity pulls down).
                impact_pt = (int(draw_center_x), int(draw_center_y) + pixel_drop)
                
                cv2.circle(img_hit_point, impact_pt, 5, self.color_dot_hit, -1)
                
                # 5. Text Info
                drop_text = f"DROP: {pixel_drop}px | Dist: {dist_z:.2f}m"
                cv2.putText(img_hit_point, drop_text, (min_pt[0], min_pt[1]-20), 
                            0, 0.6, self.color_dot_hit, 2)

        # Publishing part
        processed_img_all = self._bridge.cv2_to_imgmsg(img_all, encoding=img_msg.encoding)
        self._processed_image_all.publish(processed_img_all)

        processed_img_opt = self._bridge.cv2_to_imgmsg(img_optimal, encoding=img_msg.encoding)
        self._processed_image_optimal.publish(processed_img_opt)
        
        processed_img_hero = self._bridge.cv2_to_imgmsg(img_hit_point, encoding=img_msg.encoding)
        self._processed_image_ballistic.publish(processed_img_hero)


def main():
    rclpy.init()
    rclpy.spin(BboxWithDistance())
    rclpy.shutdown()


if __name__ == '__main__':
    main()