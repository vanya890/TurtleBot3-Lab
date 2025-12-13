#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Header, Int32
from std_msgs.msg import String
from threading import Lock

class CameraProcessor:
    def __init__(self, camera_name="camera"):
        self.camera_name = camera_name
        self.bridge = CvBridge()

        # Подписка на исходные изображения
        image_topic = f'/{camera_name}/image_raw'
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback)

        # Подписка на команды управления визуализацией
        rospy.Subscriber(f'/{camera_name}/visualization_mode', Int32, self.mode_callback)
        rospy.Subscriber(f'/{camera_name}/text_overlay', String, self.text_callback)

        # Публикация обработанных изображений
        processed_topic = f'/{camera_name}/processed'
        self.processed_pub = rospy.Publisher(processed_topic, Image, queue_size=10)

        # Параметры обработки для детекта разметки (из lane.yaml)
        self.white_hue_l = 0
        self.white_hue_h = 179
        self.white_sat_l = 0
        self.white_sat_h = 70
        self.white_light_l = 105
        self.white_light_h = 255

        self.yellow_hue_l = 10
        self.yellow_hue_h = 127
        self.yellow_sat_l = 70
        self.yellow_sat_h = 255
        self.yellow_light_l = 95
        self.yellow_light_h = 255

        # Параметры для морфологических операций
        self.kernel_size = 5

        # Режим визуализации (0: исходное, 1: бинаризованное, 2: с линиями)
        self.visualization_mode = 0
        self.text_overlay = ""
        self.text_lock = Lock()

        rospy.loginfo(f"{camera_name} Lane Detection Processor started")

    def mode_callback(self, msg):
        """Обработка изменения режима визуализации"""
        with self.text_lock:
            self.visualization_mode = msg.data
            rospy.loginfo(f"{self.camera_name} visualization mode set to {self.visualization_mode}")

    def text_callback(self, msg):
        """Обработка текстового оверлея"""
        with self.text_lock:
            self.text_overlay = msg.data

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
        """Обработка изображения в зависимости от режима визуализации"""
        with self.text_lock:
            mode = self.visualization_mode
            text = self.text_overlay

        if mode == 0:  # Исходное изображение
            result = image.copy()
        elif mode == 1:  # Бинаризованное изображение
            # Преобразование в оттенки серого
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            # Бинаризация
            _, binary = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY)
            # Преобразование обратно в BGR
            result = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
        elif mode == 2:  # Изображение с детектированными линиями
            # Преобразование в HSV
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # Маска для белой разметки
            lower_white = np.array([self.white_hue_l, self.white_sat_l, self.white_light_l])
            upper_white = np.array([self.white_hue_h, self.white_sat_h, self.white_light_h])
            mask_white = cv2.inRange(hsv, lower_white, upper_white)

            # Маска для желтой разметки
            lower_yellow = np.array([self.yellow_hue_l, self.yellow_sat_l, self.yellow_light_l])
            upper_yellow = np.array([self.yellow_hue_h, self.yellow_sat_h, self.yellow_light_h])
            mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

            # Объединение масок
            mask = cv2.bitwise_or(mask_white, mask_yellow)

            # Морфологические операции для очистки
            kernel = np.ones((self.kernel_size, self.kernel_size), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

            # Найти контуры
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Рисуем контуры на оригинальном изображении
            result = image.copy()
            cv2.drawContours(result, contours, -1, (0, 255, 0), 2)

            # Опционально, нарисовать ограничивающие прямоугольники
            for cnt in contours:
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(result, (x, y), (x + w, y + h), (255, 0, 0), 2)
        else:  # Неизвестный режим - возвращаем исходное изображение
            result = image.copy()

        # Добавляем текстовый оверлей, если он есть
        if text:
            # Рисуем текст в верхнем левом углу
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.7
            font_color = (0, 0, 255)  # Красный цвет
            thickness = 2
            line_type = cv2.LINE_AA

            # Разбиваем текст на строки
            lines = text.split('\n')
            y0, dy = 30, 30  # Начальная позиция и шаг по Y

            for i, line in enumerate(lines):
                y = y0 + i * dy
                cv2.putText(result, line, (10, y), font, font_scale, font_color, thickness, line_type)

        return result

if __name__ == '__main__':
    try:
        # Initialize ROS node
        rospy.init_node('camera_processor_node', anonymous=True)

        # Проверяем, какая камера указана в качестве параметра
        camera_name = rospy.get_param('~camera_name', 'front_camera')
        processor = CameraProcessor(camera_name)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
