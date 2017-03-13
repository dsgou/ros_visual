cmake_minimum_required(VERSION 2.8.3)
project(depth)

set(CMAKE_CXX_FLAGS "-std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

find_package(vision REQUIRED)

catkin_package()

aux_source_directory(src/ SRC_LIST)

include_directories(
  include
  /usr/include/
  ${catkin_INCLUDE_DIRS}
  ${vision_INCLUDE_DIRS}
)

link_directories(
  /usr/lib/
  /usr/local/lib/ 
  /usr/lib/x86_64-linux-gnu/ 
)

add_executable(depth ${SRC_LIST})

target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES}
  ${vision_LIBRARIES}  
)

