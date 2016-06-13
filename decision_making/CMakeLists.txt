cmake_minimum_required(VERSION 2.8.3)
project(decision_making)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  message_generation
  fusion
)

add_message_files(
  FILES
  Event.msg
)

find_package(Boost  COMPONENTS filesystem)

generate_messages(
	DEPENDENCIES
 	 sensor_msgs
	std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

aux_source_directory(src/ SRC_LIST)

include_directories(
  include
  /usr/include/
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(
  /usr/lib/
  /usr/local/lib/ 
  /usr/lib/x86_64-linux-gnu/ 
)

add_executable(decision_making ${SRC_LIST})

add_dependencies(decision_making ${fusion_EXPORTED_TARGETS})
add_dependencies(decision_making ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
