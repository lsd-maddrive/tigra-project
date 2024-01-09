cmake_minimum_required(VERSION 3.0.2)
project(ackermann_raw_controller_plugin)

find_package(catkin REQUIRED
    COMPONENTS
        roscpp
        realtime_tools
        tigra_msgs
        gazebo_ros
        gazebo_plugins
)

find_package(gazebo REQUIRED)

find_package(Threads REQUIRED)
if(Threads_FOUND)
    message("<<< Threads found >>>")
else()
    message(FATAL_ERROR "<<< Cannot find Thread >>>s")
endif()

find_package(Boost 1.7 REQUIRED
    COMPONENTS
        system
)
if(Boost_FOUND)
    message("<<< Boost found >>>")
else()
    message(FATAL_ERROR "<<< Cannot find Boost >>>")
endif()

set(CAR_INTERFACE_PLUGIN ackermann_raw_controller_plugin)

catkin_package(
    INCLUDE_DIRS
        include
    CATKIN_DEPENDS
        roscpp
        realtime_tools
        tigra_msgs
        gazebo_ros
        gazebo_plugins
    DEPENDS
        GAZEBO
)

include_directories(
    include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

add_library(
    ${CAR_INTERFACE_PLUGIN} src/plugin.cpp
)

target_link_libraries(
    ${CAR_INTERFACE_PLUGIN}
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    ${Boost_LIBRARIES}
    Threads::Threads
)
