cmake_minimum_required(VERSION 3.5.0)
project(tigra_description)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  gazebo_ros
)

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  ${catkin_INCLUDE_DIRS} 
  ${GAZEBO_INCLUDE_DIRS}
)

catkin_package(
  DEPENDS
  roscpp
  gazebo_ros
  LIBRARIES
  CATKIN_DEPENDS message_runtime
)
