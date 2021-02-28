cmake_minimum_required(VERSION 2.8.3)
project(tigra_description)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
)

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})

set(CAR_INTERFACE_PLUGIN tigra_interface_plugin)

include_directories(
${Boost_INCLUDE_DIR} 
${catkin_INCLUDE_DIRS} 
${GAZEBO_INCLUDE_DIRS}
plugins
)

catkin_package(
    DEPENDS
    roscpp
    gazebo_ros
    LIBRARIES
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  plugins
)

add_library(${CAR_INTERFACE_PLUGIN}
  plugins/plugin.cpp
)

target_link_libraries(${CAR_INTERFACE_PLUGIN}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}

)
