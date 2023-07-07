cmake_minimum_required(VERSION 3.0.2)
project(tigra_software)

find_package(Threads REQUIRED)
find_package( Boost 1.7 COMPONENTS system REQUIRED )
if(Boost_FOUND)
    # message ("Boost found")
else()
    message (FATAL_ERROR "Cannot find Boost")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tigra_msgs
  realtime_tools
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  include
)

set(ROBOT_LAYER_NAME robot_layer)
set(ROBOT_LAYER_SRC 
  src/robot_layer/main.cpp
  src/robot_layer/odom_model.cpp
)

add_executable(${ROBOT_LAYER_NAME}
  ${ROBOT_LAYER_SRC}
)

target_link_libraries(${ROBOT_LAYER_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES} 
  Threads::Threads
)