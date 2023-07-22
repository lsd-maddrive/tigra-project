cmake_minimum_required(VERSION 3.5)
project(tigra_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

# Generate messages in the 'msg' folder
add_message_files(FILES
  TigraState.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
