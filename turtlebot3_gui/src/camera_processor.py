#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Header, Int32, String, UInt8
from threading import Lock

class CameraProcessor:
    def __init__(self, camera_name="camera"):
        self.camera_name = camera_name
        self.bridge = CvBridge()

        # Subscribers
        image_topic = f'/{camera_name}/image_raw'
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback)
        rospy.Subscriber(f'/{camera_name}/visualization_mode', Int32, self.mode_callback)
        rospy.Subscriber(f'/{camera_name}/text_overlay', String, self.text_callback)
        rospy.Subscriber(f'/{camera_name}/perspective_angle', Int32, self.perspective_angle_callback)
        rospy.Subscriber(f'/{camera_name}/perspective_correction_enabled', Int32, self.perspective_correction_callback)
        rospy.Subscriber(f'/{camera_name}/lane_detection_enabled', Int32, self.lane_detection_callback)
        
        # Publisher
        processed_topic = f'/{camera_name}/processed'
        self.processed_pub = rospy.Publisher(processed_topic, Image, queue_size=10)


        # Parameters
        self.white_hue_l, self.white_hue_h = 0, 179
        self.white_sat_l, self.white_sat_h = 0, 70
        self.white_light_l, self.white_light_h = 105, 255
        self.yellow_hue_l, self.yellow_hue_h = 10, 127
        self.yellow_sat_l, self.yellow_sat_h = 70, 255
        self.yellow_light_l, self.yellow_light_h = 95, 255
        self.kernel_size = 5
        self.perspective_angle = 180.0
        self.crop_fraction = 0.6
        self.top_x, self.top_y = 72, 4
        self.bottom_x, self.bottom_y = 115, 120

        # State flags
        self.perspective_correction_enabled = False
        self.lane_detection_enabled = False
        self.visualization_mode = 0
        self.text_overlay = ""
        self.text_lock = Lock()

        self.update_homography_parameters()
        rospy.loginfo(f"{camera_name} Processor started")

    # All callback methods
    def mode_callback(self, msg):
        with self.text_lock: self.visualization_mode = msg.data
    def text_callback(self, msg):
        with self.text_lock: self.text_overlay = msg.data
    def perspective_angle_callback(self, msg):
        with self.text_lock: self.perspective_angle = float(msg.data)
        self.update_homography_parameters()
    def perspective_correction_callback(self, msg):
        with self.text_lock: self.perspective_correction_enabled = bool(msg.data)
    def lane_detection_callback(self, msg):
        with self.text_lock: self.lane_detection_enabled = bool(msg.data)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            processed = self.process_image(cv_image)
            processed_msg = self.bridge.cv2_to_imgmsg(processed, "bgr8")
            processed_msg.header = msg.header
            self.processed_pub.publish(processed_msg)
        except Exception as e:
            rospy.logerr(f"Error processing {self.camera_name}: {e}")

    def process_image(self, image):
        with self.text_lock:
            mode = self.visualization_mode
            text = self.text_overlay
            perspective_enabled = self.perspective_correction_enabled
            lane_detection_enabled = self.lane_detection_enabled

        result = image.copy()
        height, width = result.shape[:2]

        if perspective_enabled:
            result = self.apply_perspective_correction(result)
        
        if mode == 1:
            gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
            _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
            result = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)

        if lane_detection_enabled:
            hsv = cv2.cvtColor(result, cv2.COLOR_BGR2HSV)
            lower_white = np.array([self.white_hue_l, self.white_sat_l, self.white_light_l])
            upper_white = np.array([self.white_hue_h, self.white_sat_h, self.white_light_h])
            mask_white = cv2.inRange(hsv, lower_white, upper_white)
            lower_yellow = np.array([self.yellow_hue_l, self.yellow_sat_l, self.yellow_light_l])
            upper_yellow = np.array([self.yellow_hue_h, self.yellow_sat_h, self.yellow_light_h])
            mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
            mask = cv2.bitwise_or(mask_white, mask_yellow)
            kernel = np.ones((self.kernel_size, self.kernel_size), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(result, contours, -1, (0, 255, 0), 2)

        font = cv2.FONT_HERSHEY_SIMPLEX
        if text:
            lines = text.split('\n')
            y0, dy = 30, 30
            for i, line in enumerate(lines):
                y = y0 + i * dy
                cv2.putText(result, line, (10, y), font, 0.7, (0, 0, 255), 2, cv2.LINE_AA)


        return result

    def apply_perspective_correction(self, image):
        height, width = image.shape[:2]
        crop_pixels = int(height * self.crop_fraction)
        cropped_image = image[crop_pixels:, :]
        h, w = cropped_image.shape[:2]
        
        scale_x, scale_y = w / 320.0, h / 120.0
        pts_src = np.array([
            [w//2 - int(self.top_x * scale_x), int(self.top_y * scale_y)],
            [w//2 + int(self.top_x * scale_x), int(self.top_y * scale_y)],
            [w//2 + int(self.bottom_x * scale_x), h - int(self.bottom_y * scale_y)],
            [w//2 - int(self.bottom_x * scale_x), h - int(self.bottom_y * scale_y)]
        ], dtype=np.float32)
        
        pts_dst = np.array([[0, 0], [w, 0], [w, h], [0, h]], dtype=np.float32)
        h_matrix, _ = cv2.findHomography(pts_src, pts_dst)
        if h_matrix is None: return cropped_image
        
        corrected_image = cv2.warpPerspective(cropped_image, h_matrix, (w, h))
        return corrected_image

    def update_homography_parameters(self):
        base_angle = 30.0
        base_top_x, base_top_y = 72, 4
        base_bottom_x, base_bottom_y = 115, 120
        angle_ratio = self.perspective_angle / base_angle
        self.top_x = base_top_x * angle_ratio
        self.top_y = base_top_y / angle_ratio
        self.bottom_x = base_bottom_x * angle_ratio
        self.bottom_y = base_bottom_y / angle_ratio

if __name__ == '__main__':
    try:
        rospy.init_node('camera_processor_node', anonymous=True)
        camera_name = rospy.get_param('~camera_name', 'lane_camera')
        processor = CameraProcessor(camera_name)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass