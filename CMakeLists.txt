cmake_minimum_required(VERSION 2.8.3)
project(roscopter)

find_package(catkin REQUIRED COMPONENTS
    rospy
    message_generation
    std_msgs
    sensor_msgs
    geometry_msgs
    message_generation
)

add_message_files(
  FILES
  Attitude.msg
  Control.msg
  Mavlink_RAW_IMU.msg
  RC.msg
  State.msg
  VFR_HUD.msg)

generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs)

catkin_package()

install(PROGRAMS nodes/roscopter_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

