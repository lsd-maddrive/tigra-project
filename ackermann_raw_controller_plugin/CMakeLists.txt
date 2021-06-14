cmake_minimum_required(VERSION 3.0.2)
project(ackermann_raw_controller_plugin)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    tigra_msgs
    message_generation
    gazebo_ros
)

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})

set(CAR_INTERFACE_PLUGIN ackermann_raw_controller_plugin)

include_directories(
    ${catkin_INCLUDE_DIRS} 
    ${GAZEBO_INCLUDE_DIRS}
    src
)

catkin_package(
    DEPENDS
    roscpp
    gazebo_ros
    LIBRARIES
    CATKIN_DEPENDS message_runtime
)

add_library(${CAR_INTERFACE_PLUGIN}
    src/plugin.cpp
)

target_link_libraries(${CAR_INTERFACE_PLUGIN}
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
)

