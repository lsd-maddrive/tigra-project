cmake_minimum_required(VERSION 3.0.2)
project(ackermann_controller_plugin)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    message_generation
    tf
    pcl_conversions
    pcl_ros
    tf
    tf2
    tf2_eigen
    tf2_ros
    pcl_ros
    gazebo_ros
)

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})

set(CAR_INTERFACE_PLUGIN ackermann_controller_plugin)

include_directories(
    ${Boost_INCLUDE_DIR} 
    ${catkin_INCLUDE_DIRS} 
    ${GAZEBO_INCLUDE_DIRS}
    src
)

generate_messages(
    DEPENDENCIES
    sensor_msgs
    std_msgs
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

