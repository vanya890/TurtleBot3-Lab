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
        rospy.Subscriber(f'/{camera_name}/perspective_angle', Int32, self.perspective_angle_callback)
        from std_msgs.msg import Float32
        rospy.Subscriber(f'/{camera_name}/crop_fraction', Float32, self.crop_fraction_callback)

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

        # Параметры для коррекции перспективы (режим 3)
        # Угол наклона для коррекции перспективы (в градусах)
        self.perspective_angle = 180.0  # Начальное значение угла наклона
        # Доля изображения для обрезки сверху (0.0 - 1.0)
        self.crop_fraction = 0.6  # Обрезать половину (50%) сверху по умолчанию
        # Точки для гомографии (по умолчанию для изображения 320x240)
        self.top_x = 72
        self.top_y = 4
        self.bottom_x = 115
        self.bottom_y = 120

        # Режим визуализации (0: исходное, 1: бинаризованное, 2: с линиями, 3: коррекция перспективы)
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

    def perspective_angle_callback(self, msg):
        """Обработка изменения угла наклона для коррекции перспективы"""
        with self.text_lock:
            self.perspective_angle = float(msg.data)
            rospy.loginfo(f"{self.camera_name} perspective angle set to {self.perspective_angle}°")
            # Обновляем параметры гомографии на основе нового угла наклона
            self.update_homography_parameters()

    def crop_fraction_callback(self, msg):
        """Обработка изменения доли обрезки изображения"""
        with self.text_lock:
            self.crop_fraction = float(msg.data)
            # Ограничиваем значение от 0.0 до 1.0
            self.crop_fraction = max(0.0, min(1.0, self.crop_fraction))
            rospy.loginfo(f"{self.camera_name} crop fraction set to {self.crop_fraction:.2f} ({self.crop_fraction*100:.1f}%)")

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
        elif mode == 3:  # Коррекция перспективы с детектированием линий
            # Сначала применяем коррекцию перспективы
            result = self.apply_perspective_correction(image)

            # Затем выполняем детектирование линий на скорректированном изображении
            # Преобразование в HSV
            hsv = cv2.cvtColor(result, cv2.COLOR_BGR2HSV)

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

            # Рисуем контуры на скорректированном изображении
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

    def apply_perspective_correction(self, image):
        """
        Применяет коррекцию перспективы к изображению.
        1. Обрезает заданную долю изображения сверху
        2. Применяет коррекцию перспективы с использованием гомографии
        """
        # Шаг 1: Обрезаем заданную долю изображения сверху
        height, width = image.shape[:2]
        crop_pixels = int(height * self.crop_fraction)
        cropped_image = image[crop_pixels:, :]  # Берем нижнюю часть после обрезки

        # Шаг 2: Применяем коррекцию перспективы
        # Получаем размеры обрезанного изображения
        h, w = cropped_image.shape[:2]

        # Определяем точки источника (исходные координаты)
        # Используем параметры для настройки угла наклона
        top_x = self.top_x
        top_y = self.top_y
        bottom_x = self.bottom_x
        bottom_y = self.bottom_y

        # Адаптируем точки под текущий размер изображения
        # Для изображения 320x240, после обрезки получаем 120x320
        # Масштабируем точки пропорционально
        scale_x = w / 320.0
        scale_y = h / 120.0

        pts_src = np.array([
            [w//2 - int(top_x * scale_x), int(top_y * scale_y)],  # Верхний левый
            [w//2 + int(top_x * scale_x), int(top_y * scale_y)],  # Верхний правый
            [w//2 + int(bottom_x * scale_x), h - int(bottom_y * scale_y)],  # Нижний правый
            [w//2 - int(bottom_x * scale_x), h - int(bottom_y * scale_y)]   # Нижний левый
        ], dtype=np.float32)

        # Определяем точки назначения (целевые координаты)
        # Создаем прямоугольное изображение
        dst_width = w
        dst_height = h
        pts_dst = np.array([
            [0, 0],  # Верхний левый
            [dst_width, 0],  # Верхний правый
            [dst_width, dst_height],  # Нижний правый
            [0, dst_height]  # Нижний левый
        ], dtype=np.float32)

        # Вычисляем матрицу гомографии
        h_matrix, status = cv2.findHomography(pts_src, pts_dst)

        # Проверяем, что матрица гомографии допустима
        if h_matrix is None or h_matrix.shape != (3, 3):
            rospy.logerr(f"{self.camera_name} Invalid homography matrix, returning original cropped image")
            return cropped_image

        # Применяем преобразование перспективы
        try:
            corrected_image = cv2.warpPerspective(cropped_image, h_matrix, (dst_width, dst_height))

            # Проверяем, что результат является допустимым изображением
            if corrected_image is None or corrected_image.size == 0:
                rospy.logerr(f"{self.camera_name} Perspective correction failed, returning original cropped image")
                return cropped_image

            return corrected_image
        except Exception as e:
            rospy.logerr(f"{self.camera_name} Error applying perspective correction: {e}, returning original cropped image")
            return cropped_image

    def update_homography_parameters(self):
        """
        Обновляет параметры гомографии на основе текущего угла наклона.
        Преобразует угол наклона в соответствующие параметры для точек гомографии.
        """
        # Преобразуем угол наклона в радианы
        angle_rad = np.radians(self.perspective_angle)

        # Базовые параметры для угла 30 градусов (исходные значения)
        base_angle = 30.0
        base_top_x = 72
        base_top_y = 4
        base_bottom_x = 115
        base_bottom_y = 120

        # Вычисляем коэффициент масштабирования на основе угла наклона
        # Чем больше угол, тем сильнее искажение, поэтому увеличиваем top_x и bottom_x
        # и уменьшаем top_y и bottom_y
        angle_ratio = self.perspective_angle / base_angle

        # Обновляем параметры гомографии
        self.top_x = base_top_x * angle_ratio
        self.top_y = base_top_y / angle_ratio
        self.bottom_x = base_bottom_x * angle_ratio
        self.bottom_y = base_bottom_y / angle_ratio

        rospy.loginfo(f"{self.camera_name} homography parameters updated: top_x={self.top_x}, top_y={self.top_y}, bottom_x={self.bottom_x}, bottom_y={self.bottom_y}")

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
