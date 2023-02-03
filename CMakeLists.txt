cmake_minimum_required(VERSION 3.0.2)
project(qsbot_odometry)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  rviz
  sensor_msgs
  serial
  std_msgs
  tf
  tf2
  urdf
  xacro
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES QS_Bot_Odometry
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy rviz sensor_msgs serial std_msgs tf tf2 urdf xacro
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(odometry src/odometry.cpp)

target_link_libraries(odometry ${catkin_LIBRARIES})

#add_dependencies(odomdtry)
