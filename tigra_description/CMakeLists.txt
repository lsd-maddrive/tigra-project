cmake_minimum_required(VERSION 2.8.3)
project(tigra_description)

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


find_package(PCL REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR NAMES yaml_cpp.h PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY NAMES YAML_CPP PATHS ${YAML_CPP_LIBRARY_DIRS})
link_directories(${YAML_CPP_LIBRARY_DIRS})

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})

set(CAR_INTERFACE_PLUGIN tigra_interface_plugin)

include_directories(
${Boost_INCLUDE_DIR} 
${catkin_INCLUDE_DIRS} 
${GAZEBO_INCLUDE_DIRS}
plugins
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

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  plugins
)

add_library(${CAR_INTERFACE_PLUGIN}
  plugins/plugin.cpp
)

set(CONCATENATE_POINT_CLOUD_TIGRA_NODE_NAME concatenate_point_cloud_tigra)
set(CONCATENATE_POINT_CLOUD_TIGRA_NODE_SRC  src/concatenate_point_cloud_tigra.cpp)

add_executable(${CONCATENATE_POINT_CLOUD_TIGRA_NODE_NAME} ${CONCATENATE_POINT_CLOUD_TIGRA_NODE_SRC})

add_dependencies(${CONCATENATE_POINT_CLOUD_TIGRA_NODE_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${CONCATENATE_POINT_CLOUD_TIGRA_NODE_NAME} 
${catkin_LIBRARIES} 
${PCL_ROS_LIBRARIES}
${YAML_CPP_LIBRARIES})


target_link_libraries(${CAR_INTERFACE_PLUGIN}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}

)
