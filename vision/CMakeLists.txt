cmake_minimum_required(VERSION 2.8.3)
project(vision)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE Release)

find_package(OpenCV 3.2 REQUIRED)

catkin_package(INCLUDE_DIRS include
               LIBRARIES vision)

include_directories(
  include
  /usr/include/
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


file(GLOB SOURCES "src/*.cpp")
file(GLOB HEADERS "include/*.hpp")

link_directories(
  /usr/lib/
  /usr/local/lib/ 
  /usr/lib/x86_64-linux-gnu/
)

add_library(vision SHARED ${SOURCES})

target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

