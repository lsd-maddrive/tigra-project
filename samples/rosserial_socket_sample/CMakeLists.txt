cmake_minimum_required(VERSION 3.5)

project(rosserial_socker_sample)

find_package(Threads REQUIRED)
find_package( Boost 1.7 COMPONENTS system REQUIRED )
if(Boost_FOUND)
    message ("Boost found")
else()
    message (FATAL_ERROR "Cannot find Boost")
endif()

include_directories(
  include
  ros_lib
  ${Boost_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}
  src/main.cpp
  ros_lib/duration.cpp
  ros_lib/time.cpp
)

target_link_libraries(${PROJECT_NAME}
  PRIVATE 
    ${Boost_LIBRARIES} 
    Threads::Threads
)

