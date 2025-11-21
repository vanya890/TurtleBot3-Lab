После успешной сборки вы сможете запустить GUI для управления TurtleBot3 одним из двух способов:

С помощью launch файла:

roslaunch turtlebot3_gui turtlebot_gui.launch
Или напрямую запустить скрипт:

rosrun turtlebot3_gui turtlebot_gui.py
Не забудьте предварительно запустить симуляцию TurtleBot3:

export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
После этого вы увидите окно приложения, где сможете управлять роботом и наблюдать за данными с датчиков.

Пакет полностью готов к использованию и интегрирован в экосистему ROS!