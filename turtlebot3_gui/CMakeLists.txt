cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_gui)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  sensor_msgs
  nav_msgs
  turtlebot3_msgs
  std_msgs
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  DetectedObject.msg
  DetectedObjectArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
