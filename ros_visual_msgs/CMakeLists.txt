cmake_minimum_required(VERSION 2.8.3)
project(ros_visual_msgs)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  std_msgs
  message_generation
)

add_message_files(
  FILES
  Position.msg
  Rectangle.msg
  Box.msg
  FusionMsg.msg
)

generate_messages(
	DEPENDENCIES
 	 sensor_msgs
	std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime message_generation std_msgs)


