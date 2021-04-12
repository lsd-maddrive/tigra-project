cmake_minimum_required(VERSION 3.0.2)
project(tigra_software)


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

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs  
)


catkin_package(
  CATKIN_DEPENDS message_runtime

)

set(CONCATENATE_POINT_CLOUD_TIGRA_NODE_NAME concatenate_point_cloud_tigra)
set(CONCATENATE_POINT_CLOUD_TIGRA_NODE_SRC  src/concatenate_point_cloud_tigra.cpp)

add_executable(${CONCATENATE_POINT_CLOUD_TIGRA_NODE_NAME} ${CONCATENATE_POINT_CLOUD_TIGRA_NODE_SRC})

add_dependencies(${CONCATENATE_POINT_CLOUD_TIGRA_NODE_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${CONCATENATE_POINT_CLOUD_TIGRA_NODE_NAME} 
${catkin_LIBRARIES} 

${PCL_ROS_LIBRARIES}
${YAML_CPP_LIBRARIES})


include_directories(

  ${catkin_INCLUDE_DIRS}

)
