#!/usr/bin/env python3
import sys
import rospy
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QPushButton, QLabel, QGridLayout, 
                            QGroupBox, QSlider, QFormLayout)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QFont, QKeyEvent
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState, LaserScan
from nav_msgs.msg import Odometry
from turtlebot3_msgs.msg import SensorState
import math

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

        # Текущие значения скорости
        self.linear_speed = 0.0
        self.angular_speed = 0.0

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

        data_layout.addWidget(self.battery_label)
        data_layout.addWidget(self.odom_label)
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

    def scan_callback(self, msg):
        # Находим минимальное расстояние
        self.min_range = min(msg.ranges)
        # Находим максимальное расстояние (игнорируя бесконечность)
        ranges = [r for r in msg.ranges if not math.isinf(r)]
        self.max_range = max(ranges) if ranges else float('inf')

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
        if hasattr(self, 'min_range'):
            self.scan_label.setText(f'Laser: Min={self.min_range:.2f}m, Max={self.max_range:.2f}m')
        if hasattr(self, 'left_encoder'):
            self.encoder_label.setText(f'Encoders: Left={self.left_encoder}, Right={self.right_encoder}')

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = TurtleBotGUI()
    gui.show()
    sys.exit(app.exec_())
