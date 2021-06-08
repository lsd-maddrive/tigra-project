cmake_minimum_required(VERSION 3.0.2)
project(tigra_vision)

find_package(catkin REQUIRED)

catkin_package(
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

# set(CONCATENATE_POINT_CLOUD_TIGRA_NODE_NAME concatenate_point_cloud_tigra)
# set(CONCATENATE_POINT_CLOUD_TIGRA_NODE_SRC  src/concatenate_point_cloud_tigra.cpp)

# add_executable(${CONCATENATE_POINT_CLOUD_TIGRA_NODE_NAME} ${CONCATENATE_POINT_CLOUD_TIGRA_NODE_SRC})
# add_dependencies(${CONCATENATE_POINT_CLOUD_TIGRA_NODE_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${CONCATENATE_POINT_CLOUD_TIGRA_NODE_NAME} 
#   ${catkin_LIBRARIES} 
#   ${PCL_ROS_LIBRARIES}
#   ${YAML_CPP_LIBRARIES}
# )
