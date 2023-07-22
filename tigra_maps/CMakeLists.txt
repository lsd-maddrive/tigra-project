cmake_minimum_required(VERSION 3.0.2)
project(tigra_maps)

find_package(catkin REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
