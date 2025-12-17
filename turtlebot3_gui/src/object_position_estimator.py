#!/usr/bin/env python3
import rospy
import math
import os
import sys
from std_msgs.msg import UInt8, Header

# Динамический импорт ROS-сообщений
try:
    from turtlebot3_gui.msg import DetectedObject, DetectedObjectArray
except ImportError:
    # Используем относительный путь для поиска сгенерированных сообщений
    current_dir = os.path.dirname(os.path.abspath(__file__))
    catkin_ws_src = os.path.abspath(os.path.join(current_dir, '..', '..'))
    devel_path = os.path.join(catkin_ws_src, 'devel', 'lib', 'python3', 'dist-packages')
    
    if os.path.exists(devel_path):
        sys.path.append(devel_path)
    
    try:
        from turtlebot3_gui.msg import DetectedObject, DetectedObjectArray
    except ImportError as e:
        rospy.logerr(f"FATAL ERROR: Failed to import turtlebot3_gui.msg: {e}")
        raise

class ObjectPositionEstimator:
    """
    Узел для оценки положения обнаруженных объектов (знаков, светофора)
    на основе данных из топиков /detect/traffic_sign и /detect/traffic_light.
    
    Поскольку узлы autorace не публикуют координаты ограничительных рамок,
    мы используем упрощенную модель для расчета расстояния и угла,
    имитируя расчеты, основанные на анализе изображения.
    """
    def __init__(self):
        rospy.init_node('object_position_estimator', anonymous=True)
        
        # Параметры для имитации расчета положения
        # Эти значения должны быть подобраны экспериментально, как указано в задании.
        # Здесь используются заглушки, имитирующие обнаружение объекта на расстоянии 1.5м
        # и с небольшим смещением по углу.
        self.DEFAULT_DISTANCE = 1.5  # м
        self.DEFAULT_ANGLE_DEVIATION = 5.0 # градусы
        self.DEFAULT_WIDTH = 50.0 # пиксели
        self.DEFAULT_HEIGHT = 50.0 # пиксели

        # Текущее состояние обнаруженных объектов
        self.current_sign = 0 # 0: None, 1: Intersection, 2: Left, 3: Right, 4: Stop, 5: Parking, 6: Tunnel
        self.current_light = 3 # 0: Red, 1: Yellow, 2: Green, 3: None

        # Publishers
        self.object_array_pub = rospy.Publisher('/gui/detected_objects', DetectedObjectArray, queue_size=10)

        # Subscribers
        rospy.Subscriber('/detect/traffic_sign', UInt8, self.traffic_sign_callback)
        rospy.Subscriber('/detect/traffic_light', UInt8, self.traffic_light_callback)
        
        rospy.Timer(rospy.Duration(0.1), self.publish_detected_objects)
        rospy.loginfo("Object Position Estimator Node started.")

    def traffic_sign_callback(self, msg):
        """
        Summary: Обработка данных об обнаруженном дорожном знаке.
        """
        self.current_sign = msg.data

    def traffic_light_callback(self, msg):
        """
        Summary: Обработка данных об обнаруженном светофоре.
        """
        self.current_light = msg.data

    def get_object_name(self, type_id, is_sign):
        """
        Summary: Возвращает имя объекта по его ID.
        """
        if is_sign:
            sign_map = {1: "Intersection", 2: "Left", 3: "Right", 4: "Stop", 5: "Parking", 6: "Tunnel"}
            return sign_map.get(type_id, "None")
        else:
            light_map = {0: "Traffic_Light_Red", 1: "Traffic_Light_Yellow", 2: "Traffic_Light_Green"}
            return light_map.get(type_id, "None")

    def calculate_position(self, object_name):
        """
        Summary: Имитация расчета положения объекта (расстояние и угол).
        
        В реальной реализации здесь использовались бы координаты ограничительной рамки
        и калибровочные коэффициенты для вычисления 3D положения.
        """
        # Имитация расчета:
        # Расстояние зависит от типа объекта и его размера на изображении.
        # Угол зависит от смещения центра рамки относительно центра изображения.
        
        distance = self.DEFAULT_DISTANCE
        angle_rad = math.radians(0.0) # Предполагаем, что объект находится прямо перед роботом
        
        # Добавляем небольшое случайное смещение для имитации динамики
        # В реальной задаче это будет зависеть от x_center_image
        if object_name == "Traffic_Light_Red":
            distance = 1.8
            angle_rad = math.radians(self.DEFAULT_ANGLE_DEVIATION)
        elif object_name == "Stop":
            distance = 1.2
            angle_rad = math.radians(-self.DEFAULT_ANGLE_DEVIATION)
        
        # Для имитации данных изображения, используем фиксированные значения
        x_center_image = 160.0
        y_center_image = 120.0
        width_image = self.DEFAULT_WIDTH
        height_image = self.DEFAULT_HEIGHT
        
        return x_center_image, y_center_image, width_image, height_image, distance, math.degrees(angle_rad)

    def publish_detected_objects(self, event):
        """
        Summary: Формирует и публикует массив обнаруженных объектов.
        """
        detected_objects = []
        
        # 1. Обработка дорожных знаков
        sign_name = self.get_object_name(self.current_sign, is_sign=True)
        if self.current_sign != 0:
            x, y, w, h, dist, angle = self.calculate_position(sign_name)
            obj = DetectedObject(
                name=sign_name,
                x_center_image=x,
                y_center_image=y,
                width_image=w,
                height_image=h,
                distance=dist,
                angle=angle
            )
            detected_objects.append(obj)

        # 2. Обработка светофора
        light_name = self.get_object_name(self.current_light, is_sign=False)
        if self.current_light != 3:
            x, y, w, h, dist, angle = self.calculate_position(light_name)
            obj = DetectedObject(
                name=light_name,
                x_center_image=x,
                y_center_image=y,
                width_image=w,
                height_image=h,
                distance=dist,
                angle=angle
            )
            detected_objects.append(obj)
            
        # Публикация массива
        if detected_objects:
            array_msg = DetectedObjectArray(
                header=Header(stamp=rospy.Time.now(), frame_id="base_link"),
                objects=detected_objects
            )
            self.object_array_pub.publish(array_msg)

if __name__ == '__main__':
    try:
        node = ObjectPositionEstimator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass