#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Header

class CameraProcessor:
    def __init__(self, camera_name="camera"):
        rospy.init_node(f'{camera_name}_processor', anonymous=True)
        self.camera_name = camera_name
        self.bridge = CvBridge()

        # Подписка на исходные изображения
        image_topic = f'/{camera_name}/image_raw'
        rospy.Subscriber(image_topic, Image, self.image_callback)

        # Публикация обработанных изображений
        processed_topic = f'/{camera_name}/processed'
        self.processed_pub = rospy.Publisher(processed_topic, Image, queue_size=10)

        # Параметры обработки
        self.canny_low = rospy.get_param('~canny_low', 50)
        self.canny_high = rospy.get_param('~canny_high', 150)

        rospy.loginfo(f"{camera_name} Processor started")

    def image_callback(self, msg):
        try:
            # Преобразование изображения ROS в OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Обработка изображения
            processed = self.process_image(cv_image)

            # Публикация обработанного изображения
            processed_msg = self.bridge.cv2_to_imgmsg(processed, "bgr8")
            processed_msg.header = msg.header
            self.processed_pub.publish(processed_msg)

        except Exception as e:
            rospy.logerr(f"Error processing {self.camera_name}: {e}")

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

if __name__ == '__main__':
    try:
        # Проверяем, какая камера указана в качестве параметра
        camera_name = rospy.get_param('~camera_name', 'front_camera')
        processor = CameraProcessor(camera_name)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
