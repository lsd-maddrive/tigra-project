cmake_minimum_required(VERSION 3.5.0)
project(tigra_description)

find_package(catkin REQUIRED)

find_package(gazebo REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
