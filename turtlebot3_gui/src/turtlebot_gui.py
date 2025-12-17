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
from sensor_msgs.msg import LaserScan, Image, CompressedImage
from nav_msgs.msg import Odometry
from turtlebot3_msgs.msg import SensorState
from std_msgs.msg import Int32, String, UInt8
import math
from cv_bridge import CvBridge
import os # Добавлен импорт os

# Динамический импорт ROS-сообщений, чтобы избежать ModuleNotFoundError,
# если окружение не полностью настроено (например, не выполнен source devel/setup.bash)
try:
    from turtlebot3_gui.msg import DetectedObjectArray, DetectedObject
except ImportError:
    # Используем относительный путь для поиска сгенерированных сообщений
    current_dir = os.path.dirname(os.path.abspath(__file__))
    catkin_ws_src = os.path.abspath(os.path.join(current_dir, '..', '..'))
    devel_path = os.path.join(catkin_ws_src, 'devel', 'lib', 'python3', 'dist-packages')
    
    if os.path.exists(devel_path):
        sys.path.append(devel_path)
    
    # Повторная попытка импорта
    try:
        from turtlebot3_gui.msg import DetectedObjectArray, DetectedObject
    except ImportError as e:
        # Если импорт все равно не удался, мы не можем продолжить
        print(f"FATAL ERROR: Failed to import turtlebot3_gui.msg: {e}")
        raise

class LidarWidget(QWidget):
    def __init__(self, parent=None):
        super(LidarWidget, self).__init__(parent)
        self.laser_data = None
        self.detected_sign = "None"
        self.detected_light = "None"
        self.detected_objects = [] # Добавлен список обнаруженных объектов
        self.setFixedSize(400, 400)
        self.scale = 50
        self.group_size = 5

    def update_laser_data(self, laser_data):
        self.laser_data = laser_data
        self.update()

    def update_detected_sign(self, sign_name):
        self.detected_sign = sign_name
        self.update()

    def update_detected_light(self, light_name):
        self.detected_light = light_name
        self.update()

    def update_detected_objects(self, objects):
        """Обновляет список обнаруженных объектов для локальной карты."""
        self.detected_objects = objects
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        width, height = self.width(), self.height()
        center_x, center_y = width / 2, height / 2
        radius = min(width, height) / 2 - 10
        painter.fillRect(self.rect(), QColor(240, 240, 240))
        painter.setPen(QPen(QColor(0, 0, 255), 2))
        painter.setBrush(QBrush(QColor(200, 200, 255)))
        painter.drawEllipse(QPointF(center_x, center_y), 10, 10)
        painter.setPen(QPen(QColor(200, 200, 200), 1, Qt.DashLine))
        painter.drawEllipse(QPointF(center_x, center_y), radius, radius)

        if self.laser_data is not None:
            angle_min = self.laser_data.angle_min
            angle_increment = self.laser_data.angle_increment
            max_display_range = min(width, height) / 2 / self.scale
            range_max = min(self.laser_data.range_max, max_display_range)
            grouped_points = []
            for i in range(0, len(self.laser_data.ranges), self.group_size):
                group = self.laser_data.ranges[i:i+self.group_size]
                valid_ranges = [r for r in group if not (math.isinf(r) or math.isnan(r))]
                if valid_ranges:
                    min_range = min(valid_ranges)
                    if min_range > range_max: min_range = range_max
                    group_index = i + len(valid_ranges) // 2
                    angle = angle_min + group_index * angle_increment
                    rotated_angle = angle + math.pi / 2
                    x = min_range * math.cos(rotated_angle)
                    y = min_range * math.sin(rotated_angle)
                    scaled_x = center_x + x * self.scale
                    scaled_y = center_y - y * self.scale
                    grouped_points.append(QPointF(scaled_x, scaled_y))
            if grouped_points:
                painter.setPen(QPen(QColor(255, 0, 0), 2))
                painter.setBrush(QBrush(QColor(255, 0, 0)))
                rect_size = 3
                for point in grouped_points:
                    painter.drawRect(int(point.x() - rect_size/2), int(point.y() - rect_size/2), rect_size, rect_size)

        if self.detected_sign != "None":
            painter.setPen(QPen(QColor(0, 0, 255), 2))
            painter.setFont(QFont("Arial", 12))
            painter.drawLine(int(center_x), int(center_y), int(center_x), int(center_y - 100))
            painter.drawText(int(center_x + 5), int(center_y - 105), self.detected_sign)

        if self.detected_light != "None":
            painter.setPen(QPen(QColor(255, 165, 0), 2))
            painter.setFont(QFont("Arial", 12))
            painter.drawLine(int(center_x), int(center_y), int(center_x + 70), int(center_y - 70))
            painter.drawText(int(center_x + 75), int(center_y - 75), self.detected_light)

class CameraWidget(QLabel):
    def __init__(self, camera_name="Camera", parent=None):
        super(CameraWidget, self).__init__(parent)
        self.camera_name = camera_name
        self.setMinimumSize(320, 240)
        self.setAlignment(Qt.AlignCenter)
        self.setText(f"{camera_name}: No Image")
        self.setStyleSheet("background-color: #f0f0f0; border: 1px solid #cccccc;")
        self.bridge = CvBridge()
        self.current_pixmap = None
        self.detected_objects = []
        self.debug_mode_enabled = False

    def set_debug_mode(self, enabled):
        """Включает/выключает отладочный режим для отображения информации о положении."""
        self.debug_mode_enabled = enabled
        self.update()

    def update_detected_objects(self, objects):
        """Обновляет список обнаруженных объектов."""
        self.detected_objects = objects
        self.update()

    def update_image_slot(self, msg):
        """Обновляет виджет, используя несжатое ROS Image сообщение."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self._display_cv_image(cv_image)
        except Exception as e:
            print(f"Error displaying {self.camera_name} raw image: {e}")

    def update_compressed_image_slot(self, msg):
        """Обновляет виджет, используя сжатое ROS CompressedImage сообщение."""
        # rospy.loginfo(f"GUI received compressed image for {self.camera_name}.") # Убрал, чтобы не спамить лог
        try:
            # Используем cv2.imdecode для обработки сжатого изображения
            import numpy as np
            import cv2
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is not None:
                self._display_cv_image(cv_image)
            else:
                print(f"Error decoding {self.camera_name} compressed image.")
        except Exception as e:
            print(f"Error displaying {self.camera_name} compressed image: {e}")

    def _display_cv_image(self, cv_image):
        """Вспомогательный метод для отображения изображения OpenCV в PyQt."""
        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
        self.current_pixmap = QPixmap.fromImage(q_image)
        self.setPixmap(self.current_pixmap.scaled(self.width(), self.height(), Qt.KeepAspectRatio))

    def paintEvent(self, event):
        """Отображает изображение и накладывает отладочную информацию."""
        super().paintEvent(event)
        
        if self.current_pixmap is None or not self.debug_mode_enabled or not self.detected_objects:
            return

        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Вывод информации о расчете положения (Задача 9)
        painter.setPen(QPen(QColor(255, 255, 0), 2)) # Желтый цвет
        painter.setFont(QFont("Arial", 10))
        
        y_offset = 20
        for obj in self.detected_objects:
            # Выводим информацию о положении рядом с объектом
            text = f"{obj.name}: D={obj.distance:.2f}m, A={obj.angle:.1f}°"
            painter.drawText(10, y_offset, text)
            y_offset += 15

class TurtleBotGUI(QMainWindow):
    # Сигналы для потокобезопасного обновления GUI
    update_speed_signal = pyqtSignal(float, float)
    update_scan_signal = pyqtSignal(LaserScan)
    update_lane_image_signal = pyqtSignal(Image)
    update_top_image_signal = pyqtSignal(Image)
    update_detection_image_signal = pyqtSignal(CompressedImage)
    update_sign_signal = pyqtSignal(str)
    update_light_signal = pyqtSignal(str)
    update_objects_signal = pyqtSignal(list) # Сигнал для передачи списка DetectedObject

    def __init__(self):
        super().__init__()
        rospy.init_node('turtlebot_gui', anonymous=True)

        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.lane_mode_pub = rospy.Publisher('/lane_camera/visualization_mode', Int32, queue_size=10)
        self.top_mode_pub = rospy.Publisher('/top_camera/visualization_mode', Int32, queue_size=10)
        self.lane_text_pub = rospy.Publisher('/lane_camera/text_overlay', String, queue_size=10)
        self.top_text_pub = rospy.Publisher('/top_camera/text_overlay', String, queue_size=10)
        self.lane_angle_pub = rospy.Publisher('/lane_camera/perspective_angle', Int32, queue_size=10)
        self.top_angle_pub = rospy.Publisher('/top_camera/perspective_angle', Int32, queue_size=10)
        self.lane_perspective_pub = rospy.Publisher('/lane_camera/perspective_correction_enabled', Int32, queue_size=10)
        self.top_perspective_pub = rospy.Publisher('/top_camera/perspective_correction_enabled', Int32, queue_size=10)
        self.lane_detection_pub = rospy.Publisher('/lane_camera/lane_detection_enabled', Int32, queue_size=10)
        self.top_detection_pub = rospy.Publisher('/top_camera/lane_detection_enabled', Int32, queue_size=10)
        self.object_detection_pub = rospy.Publisher('/object_detection/enable', Int32, queue_size=10)

        # Subscribers
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.sub_sensor = rospy.Subscriber('/sensor_state', SensorState, self.sensor_callback)
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.sub_lane_cam = rospy.Subscriber('/lane_camera/processed', Image, self.lane_camera_callback)
        self.sub_top_cam = rospy.Subscriber('/top_camera/processed', Image, self.top_camera_callback)
        self.sub_sign = rospy.Subscriber('/detect/traffic_sign', UInt8, self.traffic_sign_callback)
        self.sub_light = rospy.Subscriber('/detect/traffic_light', UInt8, self.traffic_light_callback)
        self.sub_objects = rospy.Subscriber('/gui/detected_objects', DetectedObjectArray, self.detected_objects_callback) # Подписка на объекты

        # State variables
        self.linear_speed, self.angular_speed = 0.0, 0.0
        self.movement_state, self.rotation_state = 'none', 'none'
        self.auto_mode = False
        self.text_overlay_enabled = False
        self.visualization_text_buffer = ""
        self.lane_mode, self.top_mode = 0, 0
        self.perspective_correction_enabled = False
        self.perspective_angle = 180.0
        self.lane_detection_enabled = False
        self.object_detection_enabled = False
        self.prev_lane_mode, self.prev_top_mode = 0, 0
        self.base_linear_speed, self.rotation_increment = 0.15, 0.4
        self.wheel_radius, self.wheel_base, self.ticks_per_rev = 0.033, 0.16, 4096
        self.prev_left_encoder, self.prev_right_encoder = 0, 0
        self.left_encoder, self.right_encoder = 0, 0
        self.encoder_x, self.encoder_y, self.encoder_yaw = 0.0, 0.0, 0.0
        self.start_x, self.start_y, self.start_yaw = 0.0, 0.0, 0.0
        self.encoders_initialized = False

        self.init_ui()

        # Connect signals to slots
        self.update_speed_signal.connect(self.on_speed_update)
        self.update_scan_signal.connect(self.update_scan_slot)
        self.update_lane_image_signal.connect(self.lane_camera_widget.update_image_slot)
        self.update_top_image_signal.connect(self.top_camera_widget.update_image_slot)
        self.update_detection_image_signal.connect(self.top_camera_widget.update_compressed_image_slot)
        self.update_sign_signal.connect(self.update_sign_slot)
        self.update_light_signal.connect(self.update_light_slot)
        self.update_objects_signal.connect(self.update_objects_slot) # Соединение для объектов

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_display)
        self.timer.start(100)
        self.control_timer = QTimer(self)
        self.control_timer.timeout.connect(self.update_robot_control)
        self.control_timer.start(100)

    def init_ui(self):
        self.setWindowTitle('TurtleBot3 Control Panel')
        self.setGeometry(100, 100, 800, 600)
        central_widget = QWidget()
        main_layout = QVBoxLayout()
        self.tab_widget = QTabWidget()
        control_tab, sensors_tab = QWidget(), QWidget()
        control_tab_layout, sensors_tab_layout = QVBoxLayout(), QVBoxLayout()

        data_group = QGroupBox("Sensor Data")
        data_layout = QVBoxLayout()
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
        self.scan_label = QLabel('LDS: --')
        self.encoder_label = QLabel('Encoders: --')
        self.speed_label = QLabel('Speed: Linear: 0.0 m/s, Angular: 0.0 rad/s')
        self.encoder_odom_label = QLabel('Encoder Odometry: X=0.0, Y=0.0, Yaw=0.0°')
        self.mode_label = QLabel('Mode: Manual, Movement: none, Rotation: none')
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

        control_group = self.create_control_group()
        speed_group = self.create_speed_group()
        visualization_group = self.create_visualization_group()

        control_tab_layout.addWidget(data_group)
        control_tab_layout.addWidget(control_group)
        control_tab_layout.addWidget(speed_group)
        control_tab_layout.addWidget(visualization_group)
        control_tab.setLayout(control_tab_layout)

        lidar_group = QGroupBox("LIDAR Visualization")
        lidar_layout = QVBoxLayout()
        self.lidar_widget = LidarWidget()
        lidar_layout.addWidget(self.lidar_widget)
        lidar_group.setLayout(lidar_layout)
        cameras_group = QGroupBox("Cameras")
        cameras_layout = QHBoxLayout()
        self.lane_camera_widget = CameraWidget("Lane Camera")
        self.top_camera_widget = CameraWidget("Top Camera")
        cameras_layout.addWidget(self.lane_camera_widget)
        cameras_layout.addWidget(self.top_camera_widget)
        cameras_group.setLayout(cameras_layout)
        sensors_tab_layout.addWidget(lidar_group)
        sensors_tab_layout.addWidget(cameras_group)
        sensors_tab.setLayout(sensors_tab_layout)

        self.tab_widget.addTab(control_tab, "Control Panel")
        self.tab_widget.addTab(sensors_tab, "Sensors & Visualization")
        main_layout.addWidget(self.tab_widget)
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)
        self.setFocusPolicy(Qt.StrongFocus)

    def create_control_group(self):
        control_group = QGroupBox("Robot Control")
        control_layout = QGridLayout()
        self.auto_btn = QPushButton('AUTO MODE'); self.auto_btn.setCheckable(True); self.auto_btn.setStyleSheet("background-color: orange;"); self.auto_btn.clicked.connect(self.on_auto_toggle)
        self.forward_btn = QPushButton('Forward (W)'); self.forward_btn.setCheckable(True); self.forward_btn.clicked.connect(self.on_forward_toggle)
        self.backward_btn = QPushButton('Backward (S)'); self.backward_btn.setCheckable(True); self.backward_btn.clicked.connect(self.on_backward_toggle)
        self.left_btn = QPushButton('Start Left Turn (A)'); self.left_btn.setCheckable(True); self.left_btn.clicked.connect(self.on_left_rotation_toggle)
        self.right_btn = QPushButton('Start Right Turn (D)'); self.right_btn.setCheckable(True); self.right_btn.clicked.connect(self.on_right_rotation_toggle)
        self.stop_turn_btn = QPushButton('Stop Turn'); self.stop_turn_btn.clicked.connect(self.on_stop_turn_toggle)
        self.reset_odom_btn = QPushButton('Reset Odometry'); self.reset_odom_btn.clicked.connect(self.on_reset_odom)
        self.stop_btn = QPushButton('STOP (Space)'); self.stop_btn.setStyleSheet("background-color: red; color: white;"); self.stop_btn.clicked.connect(self.on_stop_toggle)
        control_layout.addWidget(self.auto_btn, 0, 0, 1, 3)
        control_layout.addWidget(self.left_btn, 1, 0); control_layout.addWidget(self.forward_btn, 1, 1); control_layout.addWidget(self.right_btn, 1, 2)
        control_layout.addWidget(self.stop_turn_btn, 2, 0); control_layout.addWidget(self.stop_btn, 2, 1); control_layout.addWidget(self.backward_btn, 3, 1)
        control_layout.addWidget(self.reset_odom_btn, 4, 1)
        control_group.setLayout(control_layout)
        return control_group

    def create_speed_group(self):
        speed_group = QGroupBox("Speed Control")
        speed_layout = QFormLayout()
        self.linear_slider = QSlider(Qt.Horizontal); self.linear_slider.setRange(-100, 100); self.linear_slider.setValue(0); self.linear_slider.valueChanged.connect(self.on_linear_slider_change)
        self.angular_slider = QSlider(Qt.Horizontal); self.angular_slider.setRange(-100, 100); self.angular_slider.setValue(0); self.angular_slider.valueChanged.connect(self.on_angular_slider_change)
        self.linear_value_label = QLabel("0.0 m/s")
        self.angular_value_label = QLabel("0.0 rad/s")
        speed_layout.addRow("Linear Speed:", self.linear_slider); speed_layout.addRow("", self.linear_value_label)
        speed_layout.addRow("Angular Speed:", self.angular_slider); speed_layout.addRow("", self.angular_value_label)
        speed_group.setLayout(speed_layout)
        return speed_group

    def create_visualization_group(self):
        vis_group = QGroupBox("Camera Visualization Control")
        vis_layout = QFormLayout()
        self.binary_mode_toggle = QPushButton('Toggle Binary Mode (B/W)'); self.binary_mode_toggle.setCheckable(True); self.binary_mode_toggle.clicked.connect(self.on_binary_mode_toggle)
        self.text_overlay_edit = QLineEdit(); self.text_overlay_edit.setPlaceholderText("Enter text for overlay")
        self.text_overlay_toggle = QPushButton('Toggle Text Overlay'); self.text_overlay_toggle.setCheckable(True); self.text_overlay_toggle.clicked.connect(self.on_text_overlay_toggle)
        self.lane_detection_toggle = QPushButton('Toggle Lane Detection'); self.lane_detection_toggle.setCheckable(True); self.lane_detection_toggle.clicked.connect(self.on_lane_detection_toggle)
        self.perspective_toggle = QPushButton('Toggle Perspective Correction'); self.perspective_toggle.setCheckable(True); self.perspective_toggle.clicked.connect(self.on_perspective_toggle)
        self.perspective_angle_slider = QSlider(Qt.Horizontal); self.perspective_angle_slider.setRange(10, 360); self.perspective_angle_slider.setValue(180); self.perspective_angle_slider.valueChanged.connect(self.on_perspective_angle_change)
        self.perspective_angle_label = QLabel("Perspective Angle: 180°")
        self.object_detection_toggle = QPushButton('Toggle Object Detection (Debug)'); self.object_detection_toggle.setCheckable(True); self.object_detection_toggle.clicked.connect(self.on_object_detection_toggle)
        
        vis_layout.addRow(self.binary_mode_toggle)
        vis_layout.addRow("Text Overlay:", self.text_overlay_edit)
        vis_layout.addRow(self.text_overlay_toggle)
        vis_layout.addRow(self.lane_detection_toggle)
        vis_layout.addRow(self.perspective_toggle)
        vis_layout.addRow("Angle:", self.perspective_angle_slider)
        vis_layout.addRow("", self.perspective_angle_label)
        vis_layout.addRow(self.object_detection_toggle)
        vis_group.setLayout(vis_layout)
        return vis_group

    # --- ROS Callbacks (emit signals) ---
    def odom_callback(self, msg):
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = self.euler_from_quaternion(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        self.orientation_yaw = yaw

    def sensor_callback(self, msg):
        self.left_encoder = msg.left_encoder
        self.right_encoder = msg.right_encoder
        self.calculate_odometry_from_encoders()

    def scan_callback(self, msg): self.update_scan_signal.emit(msg)
    def lane_camera_callback(self, msg): self.update_lane_image_signal.emit(msg)
    def top_camera_callback(self, msg):
        if not self.object_detection_enabled:
            self.update_top_image_signal.emit(msg)

    def detection_image_callback(self, msg):
        self.update_detection_image_signal.emit(msg)

    def traffic_sign_callback(self, msg):
        sign_map = {0: "None", 1: "Intersection", 2: "Left", 3: "Right", 4: "Stop", 5: "Parking", 6: "Tunnel"}
        sign_name = sign_map.get(msg.data, "Unknown")
        rospy.loginfo(f"GUI received traffic sign data: {sign_name} ({msg.data})")
        self.update_sign_signal.emit(sign_name)

    def traffic_light_callback(self, msg):
        light_map = {0: "Red Light", 1: "Yellow Light", 2: "Green Light", 3: "None"}
        light_name = light_map.get(msg.data, "Unknown")
        rospy.loginfo(f"GUI received traffic light data: {light_name} ({msg.data})")
        self.update_light_signal.emit(light_name)

    def detected_objects_callback(self, msg):
        """Колбэк для получения массива обнаруженных объектов."""
        self.update_objects_signal.emit(msg.objects)

    # --- PyQt Slots (update GUI) ---
    def update_objects_slot(self, objects):
        """Обновляет виджеты данными об обнаруженных объектах."""
        self.lidar_widget.update_detected_objects(objects)
        # Обновляем только виджет верхней камеры, так как детекция объектов
        # обычно происходит с фронтальной/верхней камеры, которая отображается в top_camera_widget
        self.top_camera_widget.update_detected_objects(objects)
        self.lane_camera_widget.update_detected_objects(objects) # Обновляем и камеру полосы

    def update_scan_slot(self, msg):
        self.min_range = min(msg.ranges)
        ranges = [r for r in msg.ranges if not math.isinf(r)]
        self.max_range = max(ranges) if ranges else float('inf')
        self.lidar_widget.update_laser_data(msg)

    def update_sign_slot(self, sign_name):
        self.lidar_widget.update_detected_sign(sign_name)

    def update_light_slot(self, light_name):
        self.lidar_widget.update_detected_light(light_name)

    def on_speed_update(self, linear, angular):
        self.linear_speed = linear
        self.angular_speed = angular
        self.move_robot(linear, angular)
        if not self.linear_slider.isSliderDown(): self.linear_slider.setValue(int(linear * 100))
        if not self.angular_slider.isSliderDown(): self.angular_slider.setValue(int(angular * 100))

    # --- UI Event Handlers ---
    def on_binary_mode_toggle(self):
        mode = 1 if self.binary_mode_toggle.isChecked() else 0
        self.lane_mode, self.top_mode = mode, mode
        self.lane_mode_pub.publish(mode); self.top_mode_pub.publish(mode)

    def on_text_overlay_toggle(self):
        self.text_overlay_enabled = self.text_overlay_toggle.isChecked()
        text = self.text_overlay_edit.text() if self.text_overlay_enabled else ""
        self.lane_text_pub.publish(String(data=text)); self.top_text_pub.publish(String(data=text))

    def on_lane_detection_toggle(self):
        self.lane_detection_enabled = self.lane_detection_toggle.isChecked()
        self.lane_detection_pub.publish(Int32(data=int(self.lane_detection_enabled)))
        self.top_detection_pub.publish(Int32(data=int(self.lane_detection_enabled)))

    def on_perspective_toggle(self):
        self.perspective_correction_enabled = self.perspective_toggle.isChecked()
        self.lane_perspective_pub.publish(Int32(data=int(self.perspective_correction_enabled)))
        self.top_perspective_pub.publish(Int32(data=int(self.perspective_correction_enabled)))
        if self.perspective_correction_enabled:
            self.lane_angle_pub.publish(Int32(data=int(self.perspective_angle)))
            self.top_angle_pub.publish(Int32(data=int(self.perspective_angle)))

    def on_perspective_angle_change(self, value):
        self.perspective_angle = value
        self.perspective_angle_label.setText(f"Perspective Angle: {value}°")
        self.lane_angle_pub.publish(Int32(data=value)); self.top_angle_pub.publish(Int32(data=value))

    def on_object_detection_toggle(self):
        self.object_detection_enabled = self.object_detection_toggle.isChecked()
        
        # Устанавливаем отладочный режим для CameraWidget
        self.top_camera_widget.set_debug_mode(self.object_detection_enabled)
        self.lane_camera_widget.set_debug_mode(self.object_detection_enabled)
        
        # Отписываемся от текущего топика верхней камеры
        if hasattr(self, 'sub_top_cam') and self.sub_top_cam:
            self.sub_top_cam.unregister()
        if hasattr(self, 'sub_detection_cam') and self.sub_detection_cam:
            self.sub_detection_cam.unregister()

        if self.object_detection_enabled:
            # Если детекция включена, подписываемся на сжатый поток от узлов детекции
            rospy.loginfo("Object detection enabled. Subscribing top camera to /detect/image_processed/compressed")
            # Подписываемся на сжатый топик, который публикуют детекторы
            self.sub_detection_cam = rospy.Subscriber('/detect/image_processed/compressed', CompressedImage, self.detection_image_callback)
            # Переподписываем стандартный топик верхней камеры, чтобы он не конфликтовал
            self.sub_top_cam = rospy.Subscriber('/top_camera/processed', Image, self.top_camera_callback)
            
            # Публикуем команду на включение детекции в узлах autorace (если они на это подписаны)
            self.object_detection_pub.publish(Int32(data=1))
        else:
            # Если выключена, возвращаемся к стандартному потоку от нашего обработчика
            rospy.loginfo("Object detection disabled. Subscribing top camera to /top_camera/processed")
            self.sub_top_cam = rospy.Subscriber('/top_camera/processed', Image, self.top_camera_callback)
            # Убеждаемся, что подписка на детекцию удалена
            self.sub_detection_cam = None
            
            # Публикуем команду на выключение детекции
            self.object_detection_pub.publish(Int32(data=0))

    def keyPressEvent(self, event: QKeyEvent):
        if self.auto_mode: return
        key = event.key()
        if key == Qt.Key_W or key == Qt.Key_Up: self.update_speed_signal.emit(0.2, self.angular_speed)
        elif key == Qt.Key_S or key == Qt.Key_Down: self.update_speed_signal.emit(-0.2, self.angular_speed)
        elif key == Qt.Key_A or key == Qt.Key_Left: self.update_speed_signal.emit(self.linear_speed, 0.5)
        elif key == Qt.Key_D or key == Qt.Key_Right: self.update_speed_signal.emit(self.linear_speed, -0.5)
        elif key == Qt.Key_Space: self.update_speed_signal.emit(0.0, 0.0)

    def keyReleaseEvent(self, event: QKeyEvent):
        if self.auto_mode: return
        key = event.key()
        if key in (Qt.Key_A, Qt.Key_Left, Qt.Key_D, Qt.Key_Right): self.update_speed_signal.emit(self.linear_speed, 0.0)
        elif key in (Qt.Key_W, Qt.Key_Up, Qt.Key_S, Qt.Key_Down): self.update_speed_signal.emit(0.0, self.angular_speed)

    def on_linear_slider_change(self, value):
        if self.auto_mode: return
        self.update_speed_signal.emit(value / 100.0, self.angular_speed)
        self.linear_value_label.setText(f"{value / 100.0:.2f} m/s")

    def on_angular_slider_change(self, value):
        if self.auto_mode: return
        self.update_speed_signal.emit(self.linear_speed, value / 100.0)
        self.angular_value_label.setText(f"{value / 100.0:.2f} rad/s")

    def move_robot(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)
        self.speed_label.setText(f'Speed: Linear: {linear:.2f} m/s, Angular: {angular:.2f} rad/s')

    def euler_from_quaternion(self, x, y, z, w):
        t0, t1 = +2.0 * (w * x + y * z), +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else (-1.0 if t2 < -1.0 else t2)
        pitch_y = math.asin(t2)
        t3, t4 = +2.0 * (w * z + x * y), +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        return roll_x, pitch_y, yaw_z

    def update_display(self):
        if hasattr(self, 'position_x'): self.odom_label.setText(f'Position: X={self.position_x:.2f}, Y={self.position_y:.2f}, Yaw={math.degrees(self.orientation_yaw):.1f}°')
        if hasattr(self, 'encoder_x'): self.encoder_odom_label.setText(f'Encoder Odometry: X={self.encoder_x:.2f}, Y={self.encoder_y:.2f}, Yaw={math.degrees(self.encoder_yaw):.1f}°')
        if hasattr(self, 'min_range'): self.scan_label.setText(f'Laser: Min={self.min_range:.2f}m, Max={self.max_range:.2f}m')
        if hasattr(self, 'left_encoder'): self.encoder_label.setText(f'Encoders: Left={self.left_encoder}, Right={self.right_encoder}')
        self.mode_label.setText(f'Mode: {"Auto" if self.auto_mode else "Manual"}, Movement: {self.movement_state}, Rotation: {self.rotation_state}')

    def calculate_odometry_from_encoders(self):
        if not self.encoders_initialized:
            self.prev_left_encoder, self.prev_right_encoder = self.left_encoder, self.right_encoder
            self.encoders_initialized = True
            return
        delta_left = self.left_encoder - self.prev_left_encoder
        delta_right = self.right_encoder - self.prev_right_encoder
        self.prev_left_encoder, self.prev_right_encoder = self.left_encoder, self.right_encoder
        left_distance = (delta_left / self.ticks_per_rev) * (2 * math.pi * self.wheel_radius)
        right_distance = (delta_right / self.ticks_per_rev) * (2 * math.pi * self.wheel_radius)
        avg_distance = (left_distance + right_distance) / 2.0
        delta_yaw = (right_distance - left_distance) / self.wheel_base
        self.encoder_x += avg_distance * math.cos(self.encoder_yaw + delta_yaw/2)
        self.encoder_y += avg_distance * math.sin(self.encoder_yaw + delta_yaw/2)
        self.encoder_yaw += delta_yaw
        self.encoder_yaw = math.atan2(math.sin(self.encoder_yaw), math.cos(self.encoder_yaw))

    def on_forward_toggle(self):
        if self.auto_mode: return
        if self.forward_btn.isChecked(): self.movement_state = 'forward'; self.backward_btn.setChecked(False)
        else: self.movement_state = 'none'

    def on_backward_toggle(self):
        if self.auto_mode: return
        if self.backward_btn.isChecked(): self.movement_state = 'backward'; self.forward_btn.setChecked(False)
        else: self.movement_state = 'none'

    def on_stop_turn_toggle(self):
        if self.auto_mode: return
        self.rotation_state = 'none'; self.left_btn.setChecked(False); self.right_btn.setChecked(False)

    def on_stop_toggle(self):
        if self.auto_mode: return
        self.movement_state = 'none'; self.rotation_state = 'none'
        self.forward_btn.setChecked(False); self.backward_btn.setChecked(False)
        self.left_btn.setChecked(False); self.right_btn.setChecked(False)

    def on_left_rotation_toggle(self):
        if self.auto_mode: return
        if self.left_btn.isChecked(): self.rotation_state = 'left'; self.right_btn.setChecked(False)
        else: self.rotation_state = 'none'

    def on_right_rotation_toggle(self):
        if self.auto_mode: return
        if self.right_btn.isChecked(): self.rotation_state = 'right'; self.left_btn.setChecked(False)
        else: self.rotation_state = 'none'

    def on_auto_toggle(self):
        self.auto_mode = self.auto_btn.isChecked()
        manual_enabled = not self.auto_mode
        for btn in [self.forward_btn, self.backward_btn, self.left_btn, self.right_btn, self.stop_turn_btn, self.stop_btn]:
            btn.setEnabled(manual_enabled)
        self.linear_slider.setEnabled(manual_enabled); self.angular_slider.setEnabled(manual_enabled)
        if not manual_enabled:
            self.movement_state, self.rotation_state = 'none', 'none'
            for btn in [self.forward_btn, self.backward_btn, self.left_btn, self.right_btn]:
                btn.setChecked(False)
            self.move_robot(0, 0)

    def on_reset_odom(self):
        try:
            self.start_x = float(self.start_x_edit.text())
            self.start_y = float(self.start_y_edit.text())
            self.start_yaw = math.radians(float(self.start_yaw_edit.text()))
            self.encoder_x, self.encoder_y, self.encoder_yaw = self.start_x, self.start_y, self.start_yaw
            self.encoders_initialized = False
        except ValueError:
            print("Invalid start coordinates")

    def update_robot_control(self):
        if self.auto_mode: return
        linear, angular = 0.0, 0.0
        if self.movement_state == 'forward': linear = self.base_linear_speed
        elif self.movement_state == 'backward': linear = -self.base_linear_speed
        if self.rotation_state == 'left': angular = self.rotation_increment
        elif self.rotation_state == 'right': angular = -self.rotation_increment
        self.move_robot(linear, angular)

    def closeEvent(self, event):
        rospy.loginfo("Shutting down GUI and ROS subscribers.")
        self.timer.stop()
        self.control_timer.stop()
        self.sub_odom.unregister()
        self.sub_sensor.unregister()
        self.sub_scan.unregister()
        self.sub_lane_cam.unregister()
        self.sub_top_cam.unregister()
        if hasattr(self, 'sub_detection_cam') and self.sub_detection_cam:
            self.sub_detection_cam.unregister()
        self.sub_sign.unregister()
        self.sub_light.unregister()
        self.sub_objects.unregister() # Отписка от топика объектов
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = TurtleBotGUI()
    gui.show()
    sys.exit(app.exec_())