cmake_minimum_required(VERSION 3.0.2)
project(tigra_software)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    tf
    nav_msgs
    geometry_msgs
    tigra_msgs
    realtime_tools
)

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

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    rospy
    tf
    nav_msgs
    geometry_msgs
    tigra_msgs
    realtime_tools
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

set(ROBOT_LAYER_NAME robot_layer)
set(ROBOT_LAYER_SRC
  src/robot_layer/main.cpp
  src/robot_layer/odom_model.cpp
)

add_executable(
  ${ROBOT_LAYER_NAME} ${ROBOT_LAYER_SRC}
)

target_link_libraries(
  ${ROBOT_LAYER_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  Threads::Threads
)
