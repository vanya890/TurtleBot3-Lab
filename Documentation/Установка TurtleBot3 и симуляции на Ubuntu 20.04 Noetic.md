## Установка TurtleBot3 и симуляции на Ubuntu 20.04 Noetic

Вот пошаговые команды для установки всего необходимого на чистую Ubuntu 20.04:<cite />

### 1. Установка ROS Noetic

```bash
sudo apt update
sudo apt upgrade
wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_noetic.sh
chmod 755 ./install_ros_noetic.sh
bash ./install_ros_noetic.sh
``` [1](#0-0) 

### 2. Установка зависимых ROS пакетов

```bash
sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers
``` [2](#0-1) 

### 3. Установка пакетов TurtleBot3

```bash
sudo apt install ros-noetic-dynamixel-sdk
sudo apt install ros-noetic-turtlebot3-msgs
sudo apt install ros-noetic-turtlebot3
``` [3](#0-2) 

### 4. Установка пакетов симуляции

```bash
cd ~/catkin_ws/src/
git clone -b noetic https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/catkin_ws && catkin_make
``` [4](#0-3) 

### 5. Настройка модели TurtleBot3

Добавьте в `.bashrc` переменную окружения для модели (выберите burger, waffle или waffle_pi):

```bash
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
source ~/.bashrc
``` [5](#0-4) 

### 6. Запуск симуляции

Теперь можно запустить симуляцию Gazebo:

```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
``` [6](#0-5) 

## Notes

Эти команды устанавливают полную среду для работы с TurtleBot3 в симуляции на Remote PC [7](#0-6) . Если вам нужна работа с реальным роботом, потребуется дополнительная настройка SBC (Raspberry Pi) [8](#0-7) . Для SLAM и навигации доступны дополнительные пакеты, которые можно установить по необходимости [9](#0-8) .

### Citations

**File:** _includes/en/platform/turtlebot3/quick_start/quickstart_noetic.md (L37-42)
```markdown
$ sudo apt update
$ sudo apt upgrade
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_noetic.sh
$ chmod 755 ./install_ros_noetic.sh 
$ bash ./install_ros_noetic.sh
```
```

**File:** _includes/en/platform/turtlebot3/quick_start/quickstart_noetic.md (L49-57)
```markdown
$ sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers
```
```

**File:** _includes/en/platform/turtlebot3/quick_start/quickstart_noetic.md (L64-67)
```markdown
$ sudo apt install ros-noetic-dynamixel-sdk
$ sudo apt install ros-noetic-turtlebot3-msgs
$ sudo apt install ros-noetic-turtlebot3
```
```

**File:** _includes/en/platform/turtlebot3/simulation/simulation_noetic.md (L4-4)
```markdown
- Simulation should be run on the **Remote PC**.
```

**File:** _includes/en/platform/turtlebot3/simulation/simulation_noetic.md (L34-37)
```markdown
$ cd ~/catkin_ws/src/
$ git clone -b noetic https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ cd ~/catkin_ws && catkin_make
```
```

**File:** _includes/en/platform/turtlebot3/simulation/simulation_noetic.md (L50-52)
```markdown
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```
```

**File:** _includes/en/platform/turtlebot3/slam/slam_run_slam_node_noetic.md (L37-39)
```markdown
$ echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
$ source ~/.bashrc
```
```

**File:** _includes/en/platform/turtlebot3/quick_start/sbc_setup_noetic.md (L121-121)
```markdown
Please follow the instructions below on the **SBC (Raspberry Pi)**.
```

**File:** _includes/en/platform/turtlebot3/deprecated/slam_noetic_deprecated.md (L53-90)
```markdown
<details>
<summary>
![](/assets/images/icon_unfold.png) Read more about **other SLAM methods**
</summary>
- **Gmapping** ([ROS WIKI](http://wiki.ros.org/gmapping), [Github](https://github.com/ros-perception/slam_gmapping))
  1. Install dependent packages on PC.  
    Packages related to Gmapping have already been installed on [PC Setup](/docs/en/platform/turtlebot3/quick-start) section.
  2. Launch the Gmapping SLAM node.
  ```bash
  $ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
  ```
- **Cartographer** ([ROS WIKI](http://wiki.ros.org/cartographer), [Github](https://github.com/googlecartographer/cartographer))
  1. Download and build packages on PC.  
  The Cartographer currently does not provide the binary installation method for ROS1 Noetic. Please download and build the source code as follows. Please refer to [official wiki page](https://google-cartographer-ros.readthedocs.io/en/latest/#building-installation) for more details.
  ```bash
  $ sudo apt update
  $ sudo apt install -y python3-wstool python3-rosdep ninja-build stow
  $ cd ~/catkin_ws/src
  $ wstool init src
  $ wstool merge -t src https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall
  $ wstool update -t src
  $ sudo rosdep init
  $ rosdep update
  $ rosdep install --from-paths src --ignore-src --rosdistro=noetic -y
  $ src/cartographer/scripts/install_abseil.sh
  $ sudo apt remove ros-noetic-abseil-cpp
  $ catkin_make_isolated --install --use-ninja
  ```
  2. Launch the Cartographer SLAM node.
  ```bash
  $ source ~/catkin_ws/install_isolated/setup.bash
  $ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=cartographer
  ```
- **Karto** ([ROS WIKI](http://wiki.ros.org/slam_karto), [Github](https://github.com/ros-perception/slam_karto))
  1. Install dependent packages on PC.
  ```bash
  $ sudo apt install ros-noetic-slam-karto
  ```
```
