#!/usr/bin/env python3
import sys
import rospy
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                            QHBoxLayout, QPushButton, QLabel, QGridLayout,
                            QGroupBox, QSlider, QFormLayout, QLineEdit, QTabWidget)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QPointF
from PyQt5.QtGui import (QFont, QKeyEvent, QPainter, QColor, QPen, QBrush, 
                         QPolygonF, QTransform, QImage, QPixmap)
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState, LaserScan
from nav_msgs.msg import Odometry
from turtlebot3_msgs.msg import SensorState
from std_msgs.msg import Int32, String
import math

class LidarWidget(QWidget):
    def __init__(self, parent=None):
        super(LidarWidget, self).__init__(parent)
        self.laser_data = None
        self.setFixedSize(400, 400)  # Фиксированный размер 400x400 px
        self.scale = 50  # 50 px/м
        self.group_size = 5  # Количество соседних точек для объединения

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
            # Преобразуем данные лидара в точки на виджете с объединением соседних точек
            angle_min = self.laser_data.angle_min
            angle_increment = self.laser_data.angle_increment
            max_display_range = min(width, height) / 2 / self.scale  # Максимальная дальность в метрах
            range_max = min(self.laser_data.range_max, max_display_range)

            # Группируем соседние точки для повышения надежности
            grouped_points = []
            for i in range(0, len(self.laser_data.ranges), self.group_size):
                group = self.laser_data.ranges[i:i+self.group_size]
                valid_ranges = [r for r in group if not (math.isinf(r) or math.isnan(r))]

                if valid_ranges:
                    # Находим минимальное значение в группе (ближайшую точку)
                    min_range = min(valid_ranges)
                    if min_range > range_max:
                        min_range = range_max

                    # Вычисляем средний угол для группы
                    group_index = i + len(valid_ranges) // 2
                    angle = angle_min + group_index * angle_increment

                    # Добавляем 90 градусов (π/2 радиан) для поворота отображения
                    # Теперь 0 радиан будет указывать вверх, а не вправо
                    rotated_angle = angle + math.pi / 2

                    # Преобразуем в декартовы координаты
                    x = min_range * math.cos(rotated_angle)
                    y = min_range * math.sin(rotated_angle)

                    # Масштабируем и смещаем для отображения на виджете
                    # Y инвертируется, так как в Qt ось Y направлена вниз
                    scaled_x = center_x + x * self.scale
                    scaled_y = center_y - y * self.scale

                    grouped_points.append(QPointF(scaled_x, scaled_y))

            # Рисуем концы лучей как небольшие прямоугольники
            if grouped_points:
                painter.setPen(QPen(QColor(255, 0, 0), 2))
                painter.setBrush(QBrush(QColor(255, 0, 0)))
                rect_size = 3  # Размер прямоугольника

                for point in grouped_points:
                    # Рисуем небольшой прямоугольник вместо точки
                    painter.drawRect(int(point.x() - rect_size/2),
                                    int(point.y() - rect_size/2),
                                    rect_size, rect_size)

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

        # Publishers для управления визуализацией камер
        self.lane_mode_pub = rospy.Publisher('/lane_camera/visualization_mode', Int32, queue_size=10)
        self.top_mode_pub = rospy.Publisher('/top_camera/visualization_mode', Int32, queue_size=10)
        self.lane_text_pub = rospy.Publisher('/lane_camera/text_overlay', String, queue_size=10)
        self.top_text_pub = rospy.Publisher('/top_camera/text_overlay', String, queue_size=10)

        # Subscribers
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/sensor_state', SensorState, self.sensor_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # Подписка на обработанные камеры
        from sensor_msgs.msg import Image
        rospy.Subscriber('/lane_camera/processed', Image, self.lane_camera_callback)
        rospy.Subscriber('/top_camera/processed', Image, self.top_camera_callback)

        # Текущие значения скорости
        self.linear_speed = 0.0
        self.angular_speed = 0.0

        # Состояния управления движением
        self.movement_state = 'none'  # 'forward', 'backward', 'none'
        self.rotation_state = 'none'  # 'left', 'right', 'none'
        self.auto_mode = False  # Флаг автономного режима

        # Состояния управления визуализацией
        self.text_overlay_enabled = False  # Флаг включения текстового оверлея
        self.visualization_text_buffer = ""  # Буфер для текстовых сообщений
        self.lane_mode = 0  # Режим визуализации для lane камеры
        self.top_mode = 0  # Режим визуализации для top камеры

        # Параметры скоростей
        self.base_linear_speed = 0.15  # Основная линейная скорость (м/с)
        self.rotation_increment = 0.4  # Дифференциальная разница для поворотов (rad/s)

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

        # Стартовые координаты
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0  # в радианах

        # Флаг для инициализации начальных значений энкодеров
        self.encoders_initialized = False

        # Подключаем сигнал обновления скорости
        self.update_speed.connect(self.on_speed_update)

        self.init_ui()

        # Timer для обновления GUI
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_display)
        self.timer.start(100)  # Обновление каждые 100 мс

        # Таймер для постоянного управления движением
        self.control_timer = QTimer()
        self.control_timer.timeout.connect(self.update_robot_control)
        self.control_timer.start(100)

    def init_ui(self):
        self.setWindowTitle('TurtleBot3 Control Panel')
        self.setGeometry(100, 100, 800, 600)

        # Создаем основной виджет с вкладками
        central_widget = QWidget()
        main_layout = QVBoxLayout()

        # Создаем виджет с вкладками
        self.tab_widget = QTabWidget()
        self.tab_widget.setTabPosition(QTabWidget.North)  # Вкладки сверху

        # ВКЛАДКА 1: Управление и данные
        control_tab = QWidget()
        control_tab_layout = QVBoxLayout()

        # Группа для отображения данных
        data_group = QGroupBox("Sensor Data")
        data_layout = QVBoxLayout()

        # Start Position Inputs
        start_pos_layout = QHBoxLayout()
        start_pos_layout.addWidget(QLabel('Start X:'))
        self.start_x_edit = QLineEdit('0.0')
        start_pos_layout.addWidget(self.start_x_edit)
        start_pos_layout.addWidget(QLabel('Y:'))
        self.start_y_edit = QLineEdit('0.0')
        start_pos_layout.addWidget(self.start_y_edit)
        start_pos_layout.addWidget(QLabel('Yaw (°):'))
        self.start_yaw_edit = QLineEdit('0.0')
        start_pos_layout.addWidget(self.start_yaw_edit)

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

        self.mode_label = QLabel('Mode: Manual, Movement: none, Rotation: none')
        self.mode_label.setFont(QFont("Arial", 12))

        start_pos_widget = QWidget()
        start_pos_widget.setLayout(start_pos_layout)
        data_layout.addWidget(start_pos_widget)
        data_layout.addWidget(self.odom_label)
        data_layout.addWidget(self.encoder_odom_label)
        data_layout.addWidget(self.scan_label)
        data_layout.addWidget(self.encoder_label)
        data_layout.addWidget(self.speed_label)
        data_layout.addWidget(self.mode_label)
        data_group.setLayout(data_layout)

        # Группа для управления кнопками
        control_group = QGroupBox("Robot Control")
        control_layout = QGridLayout()

        # Кнопка авторежима
        self.auto_btn = QPushButton('AUTO MODE')
        self.auto_btn.setCheckable(True)
        self.auto_btn.setStyleSheet("background-color: orange;")
        self.auto_btn.clicked.connect(self.on_auto_toggle)
        control_layout.addWidget(self.auto_btn, 0, 0, 1, 3)  # spanning across top

        # Кнопки управления движением
        self.forward_btn = QPushButton('Forward (W)')
        self.forward_btn.setCheckable(True)
        self.forward_btn.clicked.connect(self.on_forward_toggle)

        self.backward_btn = QPushButton('Backward (S)')
        self.backward_btn.setCheckable(True)
        self.backward_btn.clicked.connect(self.on_backward_toggle)

        self.left_btn = QPushButton('Start Left Turn (A)')
        self.left_btn.setCheckable(True)
        self.left_btn.clicked.connect(self.on_left_rotation_toggle)

        self.right_btn = QPushButton('Start Right Turn (D)')
        self.right_btn.setCheckable(True)
        self.right_btn.clicked.connect(self.on_right_rotation_toggle)

        self.stop_turn_btn = QPushButton('Stop Turn')
        self.stop_turn_btn.clicked.connect(self.on_stop_turn_toggle)

        self.reset_odom_btn = QPushButton('Reset Odometry')
        self.reset_odom_btn.clicked.connect(self.on_reset_odom)

        self.stop_btn = QPushButton('STOP (Space)')
        self.stop_btn.setStyleSheet("background-color: red; color: white;")
        self.stop_btn.clicked.connect(self.on_stop_toggle)

        # Размещение кнопок
        control_layout.addWidget(self.left_btn, 1, 0)
        control_layout.addWidget(self.forward_btn, 1, 1)
        control_layout.addWidget(self.right_btn, 1, 2)
        control_layout.addWidget(self.stop_turn_btn, 2, 0)
        control_layout.addWidget(self.stop_btn, 2, 1)
        control_layout.addWidget(self.backward_btn, 3, 1)
        control_layout.addWidget(self.reset_odom_btn, 4, 1)

        control_group.setLayout(control_layout)

        # Группа для управления скоростью
        speed_group = QGroupBox("Speed Control")
        speed_layout = QFormLayout()

        # Группа для управления визуализацией
        visualization_group = QGroupBox("Camera Visualization Control")
        visualization_layout = QFormLayout()

        # Элементы управления для режимов визуализации
        self.lane_mode_edit = QLineEdit('0')
        self.top_mode_edit = QLineEdit('0')
        self.apply_modes_btn = QPushButton('Apply Visualization Modes')
        self.apply_modes_btn.clicked.connect(self.on_apply_modes)

        # Элементы управления для текстового оверлея
        self.text_overlay_edit = QLineEdit()
        self.text_overlay_edit.setPlaceholderText("Enter text to display on images")
        self.text_overlay_toggle = QPushButton('Toggle Text Overlay')
        self.text_overlay_toggle.setCheckable(True)
        self.text_overlay_toggle.setChecked(False)
        self.text_overlay_toggle.clicked.connect(self.on_text_overlay_toggle)

        # Добавляем элементы в layout визуализации
        visualization_layout.addRow("Lane Camera Mode:", self.lane_mode_edit)
        visualization_layout.addRow("Top Camera Mode:", self.top_mode_edit)
        visualization_layout.addRow(self.apply_modes_btn)
        visualization_layout.addRow("Text Overlay:", self.text_overlay_edit)
        visualization_layout.addRow(self.text_overlay_toggle)

        visualization_group.setLayout(visualization_layout)

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

        # Добавляем группы на вкладку управления
        control_tab_layout.addWidget(data_group)
        control_tab_layout.addWidget(control_group)
        control_tab_layout.addWidget(speed_group)
        control_tab_layout.addWidget(visualization_group)
        control_tab_layout.addWidget(info_group)
        control_tab.setLayout(control_tab_layout)

        # ВКЛАДКА 2: Сенсоры и визуализация
        sensors_tab = QWidget()
        sensors_tab_layout = QVBoxLayout()

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
        self.lane_camera_widget = CameraWidget("Lane Camera")
        self.top_camera_widget = CameraWidget("Top Camera")

        cameras_layout.addWidget(self.lane_camera_widget)
        cameras_layout.addWidget(self.top_camera_widget)
        cameras_group.setLayout(cameras_layout)

        # Добавляем группы на вкладку сенсоров
        sensors_tab_layout.addWidget(lidar_group)
        sensors_tab_layout.addWidget(cameras_group)
        sensors_tab.setLayout(sensors_tab_layout)

        # Добавляем вкладки в основной виджет вкладок
        self.tab_widget.addTab(control_tab, "Control Panel")
        self.tab_widget.addTab(sensors_tab, "Sensors & Visualization")

        # Добавляем виджет вкладок в основной layout
        main_layout.addWidget(self.tab_widget)
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

        # Включаем прием событий клавиатуры
        self.setFocusPolicy(Qt.StrongFocus)

    def keyPressEvent(self, event: QKeyEvent):
        """Обработка нажатий клавиатуры"""
        if self.auto_mode: return
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

    def keyReleaseEvent(self, event: QKeyEvent):
        """Обработка отпускания клавиш"""
        if self.auto_mode: return
        key = event.key()

        if key in (Qt.Key_A, Qt.Key_Left, Qt.Key_D, Qt.Key_Right):
            # Сброс угловой скорости при отпускании клавиш поворота
            self.update_speed.emit(self.linear_speed, 0.0)
        elif key in (Qt.Key_W, Qt.Key_Up, Qt.Key_S, Qt.Key_Down):
            # Сброс линейной скорости при отпускании клавиш движения вперед/назад
            self.update_speed.emit(0.0, self.angular_speed)

    def on_linear_slider_change(self, value):
        """Обработка изменения слайдера линейной скорости"""
        if self.auto_mode: return
        linear_speed = value / 100.0  # Преобразуем в диапазон -1.0 до 1.0
        self.update_speed.emit(linear_speed, self.angular_speed)
        self.linear_value_label.setText(f"{linear_speed:.2f} m/s")

    def on_apply_modes(self):
        """Применение режимов визуализации"""
        try:
            lane_mode = int(self.lane_mode_edit.text())
            top_mode = int(self.top_mode_edit.text())

            # Ограничиваем режимы допустимыми значениями
            lane_mode = max(0, min(2, lane_mode))
            top_mode = max(0, min(2, top_mode))

            self.lane_mode = lane_mode
            self.top_mode = top_mode

            # Публикуем режимы
            self.lane_mode_pub.publish(lane_mode)
            self.top_mode_pub.publish(top_mode)

            # Обновляем текстовые поля с корректными значениями
            self.lane_mode_edit.setText(str(lane_mode))
            self.top_mode_edit.setText(str(top_mode))

            rospy.loginfo(f"Visualization modes set: Lane={lane_mode}, Top={top_mode}")

        except ValueError:
            rospy.logerr("Invalid mode values. Please enter integers 0-2.")

    def on_text_overlay_toggle(self):
        """Переключение текстового оверлея"""
        self.text_overlay_enabled = self.text_overlay_toggle.isChecked()

        if self.text_overlay_enabled:
            # Применяем текущий текст из буфера
            self.apply_text_overlay()
        else:
            # Отключаем текстовый оверлей
            self.lane_text_pub.publish(String(data=""))
            self.top_text_pub.publish(String(data=""))
            rospy.loginfo("Text overlay disabled")

    def apply_text_overlay(self):
        """Применение текстового оверлея"""
        text = self.text_overlay_edit.text()
        self.visualization_text_buffer = text

        if self.text_overlay_enabled and text:
            # Публикуем текст для обеих камер
            self.lane_text_pub.publish(String(data=text))
            self.top_text_pub.publish(String(data=text))
            rospy.loginfo(f"Text overlay applied: {text}")

    def on_angular_slider_change(self, value):
        """Обработка изменения слайдера угловой скорости"""
        if self.auto_mode: return
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

    def lane_camera_callback(self, msg):
        """Обработка изображения с lane камеры"""
        self.lane_camera_widget.update_image(msg)

    def top_camera_callback(self, msg):
        """Обработка изображения с top камеры"""
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
        if hasattr(self, 'position_x'):
            self.odom_label.setText(f'Position: X={self.position_x:.2f}, Y={self.position_y:.2f}, Yaw={math.degrees(self.orientation_yaw):.1f}°')
        if hasattr(self, 'encoder_x'):
            self.encoder_odom_label.setText(f'Encoder Odometry: X={self.encoder_x:.2f}, Y={self.encoder_y:.2f}, Yaw={math.degrees(self.encoder_yaw):.1f}°')
        if hasattr(self, 'min_range'):
            self.scan_label.setText(f'Laser: Min={self.min_range:.2f}m, Max={self.max_range:.2f}m')
        if hasattr(self, 'left_encoder') and hasattr(self, 'right_encoder'):
            self.encoder_label.setText(f'Encoders: Left={self.left_encoder}, Right={self.right_encoder}')
        self.mode_label.setText(f'Mode: {"Auto" if self.auto_mode else "Manual"}, Movement: {self.movement_state}, Rotation: {self.rotation_state}')

        # Обновляем текстовый оверлей, если он включен
        if self.text_overlay_enabled and hasattr(self, 'text_overlay_edit'):
            current_text = self.text_overlay_edit.text()
            if current_text != self.visualization_text_buffer:
                self.apply_text_overlay()

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

    # Методы управления движением

    def on_forward_toggle(self):
        if self.auto_mode: return
        if self.forward_btn.isChecked():
            self.movement_state = 'forward'
            self.backward_btn.setChecked(False)
        else:
            self.movement_state = 'none'

    def on_backward_toggle(self):
        if self.auto_mode: return
        if self.backward_btn.isChecked():
            self.movement_state = 'backward'
            self.forward_btn.setChecked(False)
        else:
            self.movement_state = 'none'

    def on_stop_turn_toggle(self):
        if self.auto_mode: return
        self.rotation_state = 'none'
        self.left_btn.setChecked(False)
        self.right_btn.setChecked(False)

    def on_stop_toggle(self):
        if self.auto_mode: return
        self.movement_state = 'none'
        self.rotation_state = 'none'
        self.forward_btn.setChecked(False)
        self.backward_btn.setChecked(False)
        self.left_btn.setChecked(False)
        self.right_btn.setChecked(False)

    def on_left_rotation_toggle(self):
        if self.auto_mode: return
        if self.left_btn.isChecked():
            self.rotation_state = 'left'
            self.right_btn.setChecked(False)
        else:
            self.rotation_state = 'none'

    def on_right_rotation_toggle(self):
        if self.auto_mode: return
        if self.right_btn.isChecked():
            self.rotation_state = 'right'
            self.left_btn.setChecked(False)
        else:
            self.rotation_state = 'none'

    def on_auto_toggle(self):
        self.auto_mode = self.auto_btn.isChecked()
        manual_enabled = not self.auto_mode
        self.forward_btn.setEnabled(manual_enabled)
        self.backward_btn.setEnabled(manual_enabled)
        self.left_btn.setEnabled(manual_enabled)
        self.right_btn.setEnabled(manual_enabled)
        self.stop_turn_btn.setEnabled(manual_enabled)
        self.stop_btn.setEnabled(manual_enabled)
        self.linear_slider.setEnabled(manual_enabled)
        self.angular_slider.setEnabled(manual_enabled)
        if not manual_enabled:
            # Reset states
            self.movement_state = 'none'
            self.rotation_state = 'none'
            self.forward_btn.setChecked(False)
            self.backward_btn.setChecked(False)
            self.left_btn.setChecked(False)
            self.right_btn.setChecked(False)
            self.move_robot(0, 0)

    def on_reset_odom(self):
        try:
            self.start_x = float(self.start_x_edit.text())
            self.start_y = float(self.start_y_edit.text())
            self.start_yaw = math.radians(float(self.start_yaw_edit.text()))
            self.encoder_x = self.start_x
            self.encoder_y = self.start_y
            self.encoder_yaw = self.start_yaw
            self.encoders_initialized = False  # Reset encoders for next calculation
        except ValueError:
            print("Invalid start coordinates")

    def update_robot_control(self):
        if self.auto_mode: return
        linear = 0.0
        angular = 0.0
        if self.movement_state == 'forward':
            linear = self.base_linear_speed
        elif self.movement_state == 'backward':
            linear = -self.base_linear_speed
        
        if self.rotation_state == 'left':
            angular = self.rotation_increment
        elif self.rotation_state == 'right':
            angular = -self.rotation_increment
        
        self.move_robot(linear, angular)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = TurtleBotGUI()
    gui.show()
    sys.exit(app.exec_())
