#!/usr/bin/env python3
import sys
import rospy
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QPushButton, QLabel, QGridLayout, 
                            QGroupBox, QSlider, QFormLayout)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QPointF
from PyQt5.QtGui import (QFont, QKeyEvent, QPainter, QColor, QPen, QBrush, 
                         QPolygonF, QTransform, QImage, QPixmap)
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState, LaserScan
from nav_msgs.msg import Odometry
from turtlebot3_msgs.msg import SensorState
import math

class LidarWidget(QWidget):
    def __init__(self, parent=None):
        super(LidarWidget, self).__init__(parent)
        self.laser_data = None
        self.setMinimumSize(300, 300)

    def update_laser_data(self, laser_data):
        self.laser_data = laser_data
        self.update()  # Перерисовать виджет

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Размеры виджета
        width = self.width()
        height = self.height()
        center_x = width / 2
        center_y = height / 2
        radius = min(width, height) / 2 - 10

        # Фон
        painter.fillRect(self.rect(), QColor(240, 240, 240))

        # Рисуем робота в центре
        painter.setPen(QPen(QColor(0, 0, 255), 2))
        painter.setBrush(QBrush(QColor(200, 200, 255)))
        painter.drawEllipse(QPointF(center_x, center_y), 10, 10)

        # Рисуем круг для обзора лидара
        painter.setPen(QPen(QColor(200, 200, 200), 1, Qt.DashLine))
        painter.drawEllipse(QPointF(center_x, center_y), radius, radius)

        # Если есть данные лидара, рисуем их
        if self.laser_data is not None:
            # Преобразуем данные лидара в точки на виджете
            points = []
            angle_min = self.laser_data.angle_min
            angle_increment = self.laser_data.angle_increment
            range_max = min(self.laser_data.range_max, radius * 0.9)

            for i, range_val in enumerate(self.laser_data.ranges):
                if math.isinf(range_val) or math.isnan(range_val):
                    continue

                # Ограничиваем максимальную дальность
                if range_val > range_max:
                    range_val = range_max

                # Вычисляем угол
                angle = angle_min + i * angle_increment

                # Преобразуем в декартовы координаты
                x = range_val * math.cos(angle)
                y = range_val * math.sin(angle)

                # Масштабируем и смещаем для отображения на виджете
                # Y инвертируется, так как в Qt ось Y направлена вниз
                scaled_x = center_x + x * radius / range_max
                scaled_y = center_y - y * radius / range_max

                points.append(QPointF(scaled_x, scaled_y))

            # Рисуем точки лидара
            if points:
                painter.setPen(QPen(QColor(255, 0, 0), 2))
                for point in points:
                    painter.drawPoint(point)

                # Соединяем точки линиями
                painter.setPen(QPen(QColor(255, 100, 100), 1))
                for i in range(len(points)):
                    if i < len(points) - 1:
                        painter.drawLine(points[i], points[i+1])

class CameraWidget(QLabel):
    def __init__(self, camera_name="Camera", parent=None):
        super(CameraWidget, self).__init__(parent)
        self.camera_name = camera_name
        self.setMinimumSize(320, 240)
        self.setAlignment(Qt.AlignCenter)
        self.setText(f"{camera_name}: No Image")
        self.setStyleSheet("background-color: #f0f0f0; border: 1px solid #cccccc;")

    def update_image(self, msg):
        try:
            # Преобразование ROS Image в QImage
            from cv_bridge import CvBridge
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

            # Преобразование OpenCV Image в QImage
            height, width, channel = cv_image.shape
            bytes_per_line = 3 * width
            q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()

            # Отображение изображения
            pixmap = QPixmap.fromImage(q_image)
            self.setPixmap(pixmap.scaled(self.width(), self.height(), Qt.KeepAspectRatio))

        except Exception as e:
            print(f"Error displaying {self.camera_name} image: {e}")

class TurtleBotGUI(QMainWindow):
    # Сигналы для обновления скорости
    update_speed = pyqtSignal(float, float)

    def __init__(self):
        super().__init__()
        rospy.init_node('turtlebot_gui', anonymous=True)

        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscribers
        rospy.Subscriber('/battery_state', BatteryState, self.battery_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/sensor_state', SensorState, self.sensor_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # Подписка на камеры
        from sensor_msgs.msg import Image
        rospy.Subscriber('/front_camera/image_raw', Image, self.front_camera_callback)
        rospy.Subscriber('/top_camera/image_raw', Image, self.top_camera_callback)

        # Текущие значения скорости
        self.linear_speed = 0.0
        self.angular_speed = 0.0

        # Параметры робота для расчета одометрии
        self.wheel_radius = 0.033  # Радиус колеса в метрах (для TurtleBot3 Burger)
        self.wheel_base = 0.16  # Расстояние между колесами в метрах
        self.ticks_per_rev = 4096  # Количество тиков энкодера на один оборот

        # Переменные для расчета положения на основе энкодеров
        self.prev_left_encoder = 0
        self.prev_right_encoder = 0
        self.left_encoder = 0  # Текущее значение левого энкодера
        self.right_encoder = 0  # Текущее значение правого энкодера
        self.encoder_x = 0.0  # Расчетная позиция X
        self.encoder_y = 0.0  # Расчетная позиция Y
        self.encoder_yaw = 0.0  # Расчетный угол ориентации

        # Флаг для инициализации начальных значений энкодеров
        self.encoders_initialized = False

        # Подключаем сигнал обновления скорости
        self.update_speed.connect(self.on_speed_update)

        self.init_ui()

        # Timer для обновления GUI
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_display)
        self.timer.start(100)  # Обновление каждые 100 мс

    def init_ui(self):
        self.setWindowTitle('TurtleBot3 Control Panel')
        self.setGeometry(100, 100, 800, 600)

        # Основной виджет и layout
        central_widget = QWidget()
        main_layout = QVBoxLayout()

        # Группа для отображения данных
        data_group = QGroupBox("Sensor Data")
        data_layout = QVBoxLayout()

        # Создание виджетов для отображения данных
        self.battery_label = QLabel('Battery: --')
        self.battery_label.setFont(QFont("Arial", 12))

        self.odom_label = QLabel('Position: --')
        self.odom_label.setFont(QFont("Arial", 12))

        self.scan_label = QLabel('LDS: --')
        self.scan_label.setFont(QFont("Arial", 12))

        self.encoder_label = QLabel('Encoders: --')
        self.encoder_label.setFont(QFont("Arial", 12))

        self.speed_label = QLabel('Speed: Linear: 0.0 m/s, Angular: 0.0 rad/s')
        self.speed_label.setFont(QFont("Arial", 12))

        self.encoder_odom_label = QLabel('Encoder Odometry: X=0.0, Y=0.0, Yaw=0.0°')
        self.encoder_odom_label.setFont(QFont("Arial", 12))

        data_layout.addWidget(self.battery_label)
        data_layout.addWidget(self.odom_label)
        data_layout.addWidget(self.encoder_odom_label)
        data_layout.addWidget(self.scan_label)
        data_layout.addWidget(self.encoder_label)
        data_layout.addWidget(self.speed_label)
        data_group.setLayout(data_layout)

        # Группа для управления кнопками
        control_group = QGroupBox("Robot Control")
        control_layout = QGridLayout()

        # Кнопки управления
        forward_btn = QPushButton('Forward (W)')
        forward_btn.clicked.connect(lambda: self.move_robot(0.2, 0.0))

        backward_btn = QPushButton('Backward (S)')
        backward_btn.clicked.connect(lambda: self.move_robot(-0.2, 0.0))

        left_btn = QPushButton('Left (A)')
        left_btn.clicked.connect(lambda: self.move_robot(0.0, 0.5))

        right_btn = QPushButton('Right (D)')
        right_btn.clicked.connect(lambda: self.move_robot(0.0, -0.5))

        stop_btn = QPushButton('STOP (Space)')
        stop_btn.setStyleSheet("background-color: red; color: white;")
        stop_btn.clicked.connect(lambda: self.move_robot(0.0, 0.0))

        # Размещение кнопок в сетке
        control_layout.addWidget(forward_btn, 0, 1)
        control_layout.addWidget(left_btn, 1, 0)
        control_layout.addWidget(stop_btn, 1, 1)
        control_layout.addWidget(right_btn, 1, 2)
        control_layout.addWidget(backward_btn, 2, 1)

        control_group.setLayout(control_layout)

        # Группа для управления скоростью
        speed_group = QGroupBox("Speed Control")
        speed_layout = QFormLayout()

        # Создаем слайдеры для управления скоростью
        self.linear_slider = QSlider(Qt.Horizontal)
        self.linear_slider.setMinimum(-100)
        self.linear_slider.setMaximum(100)
        self.linear_slider.setValue(0)
        self.linear_slider.setTickPosition(QSlider.TicksBelow)
        self.linear_slider.setTickInterval(20)
        self.linear_slider.valueChanged.connect(self.on_linear_slider_change)

        self.angular_slider = QSlider(Qt.Horizontal)
        self.angular_slider.setMinimum(-100)
        self.angular_slider.setMaximum(100)
        self.angular_slider.setValue(0)
        self.angular_slider.setTickPosition(QSlider.TicksBelow)
        self.angular_slider.setTickInterval(20)
        self.angular_slider.valueChanged.connect(self.on_angular_slider_change)

        self.linear_value_label = QLabel("0.0 m/s")
        self.angular_value_label = QLabel("0.0 rad/s")

        speed_layout.addRow("Linear Speed:", self.linear_slider)
        speed_layout.addRow("", self.linear_value_label)
        speed_layout.addRow("Angular Speed:", self.angular_slider)
        speed_layout.addRow("", self.angular_value_label)

        speed_group.setLayout(speed_layout)

        # Группа для визуализации лидара
        lidar_group = QGroupBox("LIDAR Visualization")
        lidar_layout = QVBoxLayout()

        self.lidar_widget = LidarWidget()
        lidar_layout.addWidget(self.lidar_widget)
        lidar_group.setLayout(lidar_layout)

        # Группа для камер
        cameras_group = QGroupBox("Cameras")
        cameras_layout = QHBoxLayout()

        # Создаем виджеты для камер
        self.front_camera_widget = CameraWidget("Front Camera")
        self.top_camera_widget = CameraWidget("Top Camera")

        cameras_layout.addWidget(self.front_camera_widget)
        cameras_layout.addWidget(self.top_camera_widget)
        cameras_group.setLayout(cameras_layout)

        # Информационная группа
        info_group = QGroupBox("Instructions")
        info_layout = QVBoxLayout()

        info_text = QLabel(
            "Keyboard Controls:\n"
            "W/↑ - Move Forward\n"
            "S/↓ - Move Backward\n"
            "A/← - Turn Left\n"
            "D/→ - Turn Right\n"
            "Space - Stop\n"
            "\n"
            "Use sliders for precise speed control"
        )
        info_text.setWordWrap(True)
        info_layout.addWidget(info_text)
        info_group.setLayout(info_layout)

        # Добавляем группы в основной layout
        main_layout.addWidget(data_group)
        main_layout.addWidget(control_group)
        main_layout.addWidget(speed_group)
        main_layout.addWidget(lidar_group)
        main_layout.addWidget(cameras_group)
        main_layout.addWidget(info_group)

        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

        # Включаем прием событий клавиатуры
        self.setFocusPolicy(Qt.StrongFocus)

    def keyPressEvent(self, event: QKeyEvent):
        """Обработка нажатий клавиатуры"""
        key = event.key()

        if key == Qt.Key_W or key == Qt.Key_Up:
            self.update_speed.emit(0.2, self.angular_speed)
        elif key == Qt.Key_S or key == Qt.Key_Down:
            self.update_speed.emit(-0.2, self.angular_speed)
        elif key == Qt.Key_A or key == Qt.Key_Left:
            self.update_speed.emit(self.linear_speed, 0.5)
        elif key == Qt.Key_D or key == Qt.Key_Right:
            self.update_speed.emit(self.linear_speed, -0.5)
        elif key == Qt.Key_Space:
            self.update_speed.emit(0.0, 0.0)

    def on_linear_slider_change(self, value):
        """Обработка изменения слайдера линейной скорости"""
        linear_speed = value / 100.0  # Преобразуем в диапазон -1.0 до 1.0
        self.update_speed.emit(linear_speed, self.angular_speed)
        self.linear_value_label.setText(f"{linear_speed:.2f} m/s")

    def on_angular_slider_change(self, value):
        """Обработка изменения слайдера угловой скорости"""
        angular_speed = value / 100.0  # Преобразуем в диапазон -1.0 до 1.0
        self.update_speed.emit(self.linear_speed, angular_speed)
        self.angular_value_label.setText(f"{angular_speed:.2f} rad/s")

    def on_speed_update(self, linear, angular):
        """Обновление значений скорости и отправка команды роботу"""
        self.linear_speed = linear
        self.angular_speed = angular
        self.move_robot(linear, angular)

        # Обновляем слайдеры, если изменение было не через них
        if not self.linear_slider.isSliderDown():
            self.linear_slider.setValue(int(linear * 100))
        if not self.angular_slider.isSliderDown():
            self.angular_slider.setValue(int(angular * 100))

    def move_robot(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)
        self.speed_label.setText(f'Speed: Linear: {linear:.2f} m/s, Angular: {angular:.2f} rad/s')

    def battery_callback(self, msg):
        self.battery_voltage = msg.voltage
        self.battery_percentage = msg.percentage

    def odom_callback(self, msg):
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y
        # Получаем ориентацию (кватернион)
        orientation_q = msg.pose.pose.orientation
        # Преобразуем кватернион в углы Эйлера (рыскание, тангаж, крен)
        _, _, yaw = self.euler_from_quaternion(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        self.orientation_yaw = yaw

    def sensor_callback(self, msg):
        self.left_encoder = msg.left_encoder
        self.right_encoder = msg.right_encoder

        # Расчет положения на основе данных энкодеров
        self.calculate_odometry_from_encoders()

    def scan_callback(self, msg):
        # Находим минимальное расстояние
        self.min_range = min(msg.ranges)
        # Находим максимальное расстояние (игнорируя бесконечность)
        ranges = [r for r in msg.ranges if not math.isinf(r)]
        self.max_range = max(ranges) if ranges else float('inf')

        # Обновляем виджет визуализации лидара
        self.lidar_widget.update_laser_data(msg)

    def front_camera_callback(self, msg):
        """Обработка изображения с передней камеры"""
        self.front_camera_widget.update_image(msg)

    def top_camera_callback(self, msg):
        """Обработка изображения с верхней камеры"""
        self.top_camera_widget.update_image(msg)

    def euler_from_quaternion(self, x, y, z, w):
        """
        Преобразование кватерниона в углы Эйлера
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z  # в радианах

    def update_display(self):
        if hasattr(self, 'battery_voltage'):
            self.battery_label.setText(f'Battery: {self.battery_voltage:.2f}V ({self.battery_percentage:.1f}%)')
        if hasattr(self, 'position_x'):
            self.odom_label.setText(f'Position: X={self.position_x:.2f}, Y={self.position_y:.2f}, Yaw={math.degrees(self.orientation_yaw):.1f}°')
        if hasattr(self, 'encoder_x'):
            self.encoder_odom_label.setText(f'Encoder Odometry: X={self.encoder_x:.2f}, Y={self.encoder_y:.2f}, Yaw={math.degrees(self.encoder_yaw):.1f}°')
        if hasattr(self, 'min_range'):
            self.scan_label.setText(f'Laser: Min={self.min_range:.2f}m, Max={self.max_range:.2f}m')
        if hasattr(self, 'left_encoder') and hasattr(self, 'right_encoder'):
            self.encoder_label.setText(f'Encoders: Left={self.left_encoder}, Right={self.right_encoder}')

    def calculate_odometry_from_encoders(self):
        """
        Расчет положения робота на основе данных с энкодеров
        """
        # Инициализация при первом вызове
        if not self.encoders_initialized:
            self.prev_left_encoder = self.left_encoder
            self.prev_right_encoder = self.right_encoder
            self.encoders_initialized = True
            return

        # Вычисляем изменение тиков энкодеров
        delta_left = self.left_encoder - self.prev_left_encoder
        delta_right = self.right_encoder - self.prev_right_encoder

        # Сохраняем текущие значения энкодеров для следующего вызова
        self.prev_left_encoder = self.left_encoder
        self.prev_right_encoder = self.right_encoder

        # Вычисляем расстояние, пройденное каждым колесом
        # (количество тиков / тиков на оборот) * (2 * PI * радиус колеса)
        left_distance = (delta_left / self.ticks_per_rev) * (2 * math.pi * self.wheel_radius)
        right_distance = (delta_right / self.ticks_per_rev) * (2 * math.pi * self.wheel_radius)

        # Вычисляем среднее расстояние и изменение угла
        avg_distance = (left_distance + right_distance) / 2.0
        delta_yaw = (right_distance - left_distance) / self.wheel_base

        # Обновляем позицию с использованием дифференциального привода
        self.encoder_x += avg_distance * math.cos(self.encoder_yaw + delta_yaw/2)
        self.encoder_y += avg_distance * math.sin(self.encoder_yaw + delta_yaw/2)
        self.encoder_yaw += delta_yaw

        # Нормализуем угол в диапазон [-π, π]
        self.encoder_yaw = math.atan2(math.sin(self.encoder_yaw), math.cos(self.encoder_yaw))

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = TurtleBotGUI()
    gui.show()
    sys.exit(app.exec_())
