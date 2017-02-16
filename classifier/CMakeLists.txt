cmake_minimum_required(VERSION 2.8.3)
project(classifier)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  message_generation
  fusion
)

add_message_files(
  FILES
  Event.msg
)

include_directories(${catkin_INCLUDE_DIRS})
generate_messages(
	DEPENDENCIES
 	 sensor_msgs
	std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

