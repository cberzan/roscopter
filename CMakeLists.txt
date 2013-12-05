cmake_minimum_required(VERSION 2.8.3)
project(roscopter)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    message_generation
)

## Declare ROS messages, services and actions
add_message_files(
  DIRECTORY msg
  FILES
  Attitude.msg
  Control.msg
  Mavlink_RAW_IMU.msg
  RC.msg
  State.msg
  VFR_HUD.msg)

generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs)

## catkin specific configuration
#catkin_package()


catkin_package(CATKIN_DEPENDS message_runtime std_msgs sensor_msgs geometry_msgs)

## Build

include_directories(include ${catkin_INCLUDE_DIRS})

#catkin_python_setup()

#install(PROGRAMS nodes/
#         DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#add_executable(Roscopter
#    src/roscopter.py
#)

#target_link_libraries(Lander ${catkin_LIBRARIES})

#set( CMAKE_BUILD_TYPE Debug )
