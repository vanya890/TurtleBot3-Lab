## Правильные launch-файлы для Noetic

Отлично! Вы нашли правильные launch-файлы.<cite /> В пакете `turtlebot3_autorace_2020` для Noetic используются упрощенные названия launch-файлов:<cite />

- `intrinsic_camera_calibration.launch` (вместо `turtlebot3_autorace_intrinsic_camera_calibration.launch`)
- `extrinsic_camera_calibration.launch` (вместо `turtlebot3_autorace_extrinsic_camera_calibration.launch`) [1](#5-0) 

## Правильная последовательность запуска

Используйте следующие команды для запуска AutoRace в симуляции:<cite />

**Терминал 1: Запустите Gazebo**
```bash
roslaunch turtlebot3_gazebo turtlebot3_autorace.launch
```

**Терминал 2: Запустите intrinsic калибровку**
```bash
export GAZEBO_MODE=true
export AUTO_IN_CALIB=action
roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch
```

**Терминал 3: Запустите extrinsic калибровку**
```bash
export AUTO_EX_CALIB=action
roslaunch turtlebot3_autorace_camera extrinsic_camera_calibration.launch
```

**Терминал 4: Запустите основной узел AutoRace**
```bash
export AUTO_DT_CALIB=action
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_autorace_core turtlebot3_autorace_core.launch
```

**Терминал 5: Активируйте автоматический режим**
```bash
rostopic pub -1 /core/decided_mode std_msgs/UInt8 "data: 2"
``` [2](#5-1) 

## Notes

Структура launch-файлов в `turtlebot3_autorace_2020` для Noetic упрощена по сравнению со старой версией для Kinetic.<cite /> Файлы теперь имеют более короткие названия без префикса `turtlebot3_autorace_`.<cite /> Для симуляции в Gazebo обязательно установите переменную окружения `GAZEBO_MODE=true` перед запуском калибровки камеры. [3](#5-2)

### Citations

**File:** _includes/en/platform/turtlebot3/autonomous_driving/humble/autonomous_driving_lane_detection_humble.md (L17-22)
```markdown
``` bash
$ ros2 launch turtlebot3_autorace_camera intrinsic_camera_calibration.launch.py
```  
``` bash
$ ros2 launch turtlebot3_autorace_camera extrinsic_camera_calibration.launch.py
```  
```

**File:** _includes/en/platform/turtlebot3/autonomous_driving/kinetic/autonomous_driving_autorace_kinetic.md (L477-480)
```markdown
   ```bash
   $ export GAZEBO_MODE=true
   $ export AUTO_IN_CALIB=action
   $ roslaunch turtlebot3_autorace_camera turtlebot3_autorace_intrinsic_camera_calibration.launch
```

**File:** _includes/en/platform/turtlebot3/autonomous_driving/kinetic/autonomous_driving_autorace_kinetic.md (L484-495)
```markdown

   ```bash
   $ export AUTO_EX_CALIB=action
   $ export AUTO_DT_CALIB=action
   $ export TURTLEBOT3_MODEL=burger
   $ roslaunch turtlebot3_autorace_core turtlebot3_autorace_core.launch
   ```

5. `Remote PC` Open new terminal, then enter

   ```bash
   $ rostopic pub -1 /core/decided_mode std_msgs/UInt8 "data: 2"
```
