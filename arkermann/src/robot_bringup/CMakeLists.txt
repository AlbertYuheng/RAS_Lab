cmake_minimum_required(VERSION 2.8.3)
project(robot_bringup)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  tf
  serial
  std_msgs
  message_generation
  nav_msgs
)

catkin_package(
   INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}

)

add_executable(robot_bringup 
               src/robot_bringup.cpp
               src/robot.cpp
               src/mbot_linux_serial.cpp
	       )
               
target_link_libraries(robot_bringup ${catkin_LIBRARIES})




