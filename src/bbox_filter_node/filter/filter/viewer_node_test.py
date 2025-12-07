import cv2
import cv_bridge
import message_filters
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
# Commented out to run standalone
# from .library.score_functions import _get_centered, _get_close, _get_wide, _get_fract_sizes

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
    PIXEL_SCALE_FACTOR = 800.0 

    def __init__(self):
        super().__init__('bbox_viewer')
        self._bridge = cv_bridge.CvBridge()

        # --- PUBLISHERS ---
        # 1. All detections
        self._processed_image_all = self.create_publisher(
            Image, '/yolov8_processed/all', self.QUEUE_SIZE)

        # 2. Optimal target only
        self._processed_image_optimal = self.create_publisher(
            Image, '/yolov8_processed/optimal', self.QUEUE_SIZE)
        
        # 3. Ballistics view
        self._processed_image_ballistic = self.create_publisher(
            Image, '/yolov8_processed/ballistics', self.QUEUE_SIZE)

        # --- SUBSCRIBERS ---
        # 1. Detections
        self._detections_subscription = message_filters.Subscriber(
            self, Detection2DArray, '/output_detections_with_pose')
        
        # 2. Image
        self._image_subscription = message_filters.Subscriber(
            self, Image, '/image')

        # --- SYNCHRONIZER ---
        # Use ApproximateTimeSynchronizer to handle slight timestamp differences in rosbag
        self.time_synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self._detections_subscription, self._image_subscription],
            queue_size=self.QUEUE_SIZE, 
            slop=0.5) # 0.5s tolerance for bag playback

        self.time_synchronizer.registerCallback(self.detections_callback)

    def calculate_pixel_drop(self, distance_z):
        if distance_z <= 0: return 0
        t = distance_z / self.BULLET_SPEED
        drop_meters = 0.5 * self.GRAVITY * (t ** 2)
        drop_pixels = (drop_meters * self.PIXEL_SCALE_FACTOR) / distance_z
        return int(drop_pixels)

    def detections_callback(self, detections_msg, img_msg):
        # Color Setup
        color_txt = (255, 0, 255)

        cv2_img = self._bridge.imgmsg_to_cv2(img_msg)
        cv2_img_flip = cv2.flip(cv2_img, -1)
        img_height = cv2_img.shape[0]  # Altezza in pixel 
        img_width = cv2_img.shape[1]   # Larghezza in pixel 

        # Canvases
        img_all = cv2_img_flip.copy()
        img_optimal = cv2_img_flip.copy()
        img_hit_point = cv2_img_flip.copy()
        
        # Center of the image (in raw coordinates, not flipped yet)
        raw_center_x = img_width / 2
        raw_center_y = img_height / 2

        #-------------Unnecessary Part-------------#

        opt_x = -9999
        opt_y = -9999
        min_distance = float('inf')


        for detection in detections_msg.detections:
            cx = detection.bbox.center.position.x
            cy = detection.bbox.center.position.y
            
            # Pythagorean theorem to find distance to center
            dist = math.sqrt((cx - raw_center_x)**2 + (cy - raw_center_y)**2)
            
            if dist < min_distance:
                min_distance = dist
                opt_x = cx
                opt_y = cy

        #-------------Unnecessary Part-------------#

        # --- DRAWING LOOP ---
        for detection in detections_msg.detections:

            # Single detection raw coordinates
            center_x = detection.bbox.center.position.x
            center_y = detection.bbox.center.position.y

            width = detection.bbox.size_x
            height = detection.bbox.size_y
            
            # Distance Z from Pose
            dist_z = detection.results[0].pose.pose.position.z

            # Single detection flipped coordinates
            flipped_center_x = img_width - center_x
            flipped_center_y = img_height - center_y

            s_x = int(width*0.7/2)
            s_y = int(height*0.7/2)

            label_id = int(detection.results[0].hypothesis.class_id)
            label = names.get(label_id, str(label_id))
            conf_score = detection.results[0].hypothesis.score
            label_text = f'{label} {conf_score:.2f}'
            coordinates = f'x:{center_x:.2f} y:{center_y:.2f} z:{dist_z:.2f}'
            
            # Computation of the rectangle vertices
            min_pt = (round(flipped_center_x - (width / 2.0)), round(flipped_center_y - (height / 2.0)))
            max_pt = (round(flipped_center_x + (width / 2.0)), round(flipped_center_y + (height / 2.0)))
        
            new_min_pt = (round(flipped_center_x - s_x), round(flipped_center_y - s_y))
            new_max_pt = (round(flipped_center_x + s_x), round(flipped_center_y + s_y))

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
                center_pt = (int(flipped_center_x), int(flipped_center_y))
                cv2.circle(img_hit_point, center_pt, 5, self.color_dot_aim, -1)
                
                # 3. Calculate Drop
                pixel_drop = self.calculate_pixel_drop(dist_z)
                
                # 4. Draw Impact Point
                # NOTE: Since image is FLIPPED (Upside Down), gravity pulls UP in image coordinates
                # So we ADD pixel_drop to Y (which moves it down in pixel coords, but check visual)
                impact_pt = (int(flipped_center_x), int(flipped_center_y) + pixel_drop)
                
                cv2.circle(img_hit_point, impact_pt, 5, self.color_dot_hit, -1)
                
                # 5. Text Info
                drop_text = f"DROP: {pixel_drop}px | Dist: {dist_z:.2f}m"
                cv2.putText(img_hit_point, drop_text, (min_pt[0], min_pt[1]-20), 
                            0, 0.6, self.color_dot_hit, 2)

        # Publish
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