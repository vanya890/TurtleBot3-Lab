#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from turtlebot3_msgs.msg import SensorState
import math

class SensorStateSimulator:
    def __init__(self):
        rospy.init_node('sensor_state_simulator', anonymous=True)

        # Параметры робота
        self.wheel_radius = 0.033  # Радиус колеса в метрах
        self.wheel_base = 0.16  # Расстояние между колесами в метрах
        self.ticks_per_rev = 4096  # Количество тиков энкодера на один оборот

        # Текущие значения энкодеров
        self.left_encoder = 0
        self.right_encoder = 0

        # Инициализация
        self.prev_time = rospy.Time.now()
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_yaw = 0.0
        self.initialized = False

        # Издатель для топика sensor_state
        self.sensor_state_pub = rospy.Publisher('/sensor_state', SensorState, queue_size=10)

        # Подписчик на одометрию
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        rospy.loginfo("Sensor State Simulator started")

    def odom_callback(self, msg):
        current_time = rospy.Time.now()

        # Получаем текущую позицию
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        # Получаем ориентацию (кватернион)
        orientation_q = msg.pose.pose.orientation
        # Преобразуем кватернион в углы Эйлера
        _, _, current_yaw = self.euler_from_quaternion(
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)

        if not self.initialized:
            # Первоначальная инициализация
            self.prev_time = current_time
            self.prev_x = current_x
            self.prev_y = current_y
            self.prev_yaw = current_yaw
            self.initialized = True
            return

        # Вычисляем пройденное расстояние и изменение угла
        delta_time = (current_time - self.prev_time).to_sec()

        # Расстояние, пройденное роботом
        distance = math.sqrt((current_x - self.prev_x)**2 + (current_y - self.prev_y)**2)

        # Изменение угла ориентации
        delta_yaw = self.normalize_angle(current_yaw - self.prev_yaw)

        # Расчет расстояний, пройденных каждым колесом
        # Для дифференциального привода:
        # left_distance = total_distance - (wheel_base * delta_yaw) / 2
        # right_distance = total_distance + (wheel_base * delta_yaw) / 2

        # Если робот двигался
        if delta_time > 0:
            linear_velocity = distance / delta_time
            angular_velocity = delta_yaw / delta_time

            # Расстояния, пройденные каждым колесом
            left_distance = (linear_velocity - angular_velocity * self.wheel_base / 2) * delta_time
            right_distance = (linear_velocity + angular_velocity * self.wheel_base / 2) * delta_time

            # Преобразуем расстояние в тики энкодера
            left_ticks = int((left_distance / (2 * math.pi * self.wheel_radius)) * self.ticks_per_rev)
            right_ticks = int((right_distance / (2 * math.pi * self.wheel_radius)) * self.ticks_per_rev)

            # Обновляем значения энкодеров
            self.left_encoder += left_ticks
            self.right_encoder += right_ticks

        # Создаем и публикуем сообщение SensorState
        sensor_msg = SensorState()
        sensor_msg.header.stamp = current_time
        sensor_msg.header.frame_id = "base_link"
        sensor_msg.left_encoder = self.left_encoder
        sensor_msg.right_encoder = self.right_encoder
        sensor_msg.battery = 12.0  # Произвольное значение для симуляции

        self.sensor_state_pub.publish(sensor_msg)

        # Сохраняем текущие значения для следующего вызова
        self.prev_time = current_time
        self.prev_x = current_x
        self.prev_y = current_y
        self.prev_yaw = current_yaw

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

    def normalize_angle(self, angle):
        """
        Нормализация угла в диапазон [-π, π]
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

if __name__ == '__main__':
    try:
        simulator = SensorStateSimulator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
