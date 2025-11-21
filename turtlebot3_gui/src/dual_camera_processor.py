#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Header

class DualCameraProcessor:
    def __init__(self):
        rospy.init_node('dual_camera_processor')
        self.bridge = CvBridge()

        # Инициализация переменных для хранения изображений
        self.front_image = None
        self.top_image = None

        # Подписка на камеры
        rospy.Subscriber('/front_camera/image_raw', Image, self.front_camera_callback)
        rospy.Subscriber('/top_camera/image_raw', Image, self.top_camera_callback)

        # Публикация обработанных изображений
        self.front_processed_pub = rospy.Publisher('/front_camera/processed', Image, queue_size=10)
        self.top_processed_pub = rospy.Publisher('/top_camera/processed', Image, queue_size=10)

        # Параметры обработки
        self.canny_low = rospy.get_param('~canny_low', 50)
        self.canny_high = rospy.get_param('~canny_high', 150)

        rospy.loginfo("Dual Camera Processor started")

    def front_camera_callback(self, msg):
        try:
            # Преобразование изображения ROS в OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.front_image = cv_image

            # Обработка изображения
            processed = self.process_image(cv_image)

            # Публикация обработанного изображения
            processed_msg = self.bridge.cv2_to_imgmsg(processed, "bgr8")
            processed_msg.header = msg.header
            self.front_processed_pub.publish(processed_msg)

        except Exception as e:
            rospy.logerr(f"Error processing front camera: {e}")

    def top_camera_callback(self, msg):
        try:
            # Преобразование изображения ROS в OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.top_image = cv_image

            # Обработка изображения
            processed = self.process_image(cv_image)

            # Публикация обработанного изображения
            processed_msg = self.bridge.cv2_to_imgmsg(processed, "bgr8")
            processed_msg.header = msg.header
            self.top_processed_pub.publish(processed_msg)

        except Exception as e:
            rospy.logerr(f"Error processing top camera: {e}")

    def process_image(self, image):
        """Обработка изображения с помощью OpenCV"""
        # Преобразование в оттенки серого
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Применение фильтра Canny для обнаружения краев
        edges = cv2.Canny(gray, self.canny_low, self.canny_high)

        # Преобразование обратно в BGR для отображения
        edges_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

        # Создание комбинированного изображения (оригинал + края)
        combined = np.hstack((image, edges_bgr))

        return combined

    def get_front_image(self):
        """Возвращает текущее изображение с передней камеры"""
        return self.front_image

    def get_top_image(self):
        """Возвращает текущее изображение с верхней камеры"""
        return self.top_image

if __name__ == '__main__':
    try:
        processor = DualCameraProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
