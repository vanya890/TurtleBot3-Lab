#!/usr/bin/env python3

class DetectedObject:
    """
    Класс для хранения информации об обнаруженном объекте.
    """
    def __init__(self, name, x, y, width, height, distance, angle):
        """
        Конструктор для инициализации объекта.

        :param name: Имя объекта (например, 'stop_sign', 'traffic_light_green').
        :param x: Координата X центра ограничительной рамки на изображении.
        :param y: Координата Y центра ограничительной рамки на изображении.
        :param width: Ширина ограничительной рамки.
        :param height: Высота ограничительной рамки.
        :param distance: Расчетное расстояние до объекта в метрах.
        :param angle: Расчетный угол до объекта в градусах.
        """
        self.name = name
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.distance = distance
        self.angle = angle

    def __str__(self):
        """
        Возвращает строковое представление объекта для отладки.
        """
        return (f"Object: {self.name}, "
                f"Center: ({self.x}, {self.y}), "
                f"Size: ({self.width}x{self.height}), "
                f"Dist: {self.distance:.2f}m, "
                f"Angle: {self.angle:.1f}°")
