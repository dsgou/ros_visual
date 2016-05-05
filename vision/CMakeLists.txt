cmake_minimum_required(VERSION 2.8.3)
project(vision)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE Release)

find_package(OpenCV REQUIRED)

include_directories(
  include
  /usr/include/
  ${OpenCV_INCLUDE_DIRS}
)

 
file(GLOB SOURCES "src/*.cpp")
file(GLOB HEADERS "include/*.hpp")

link_directories(
  /usr/lib/
  /usr/lib/x86_64-linux-gnu/
)

add_library(vision SHARED ${SOURCES})

target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  alglib
)

install(TARGETS vision DESTINATION /usr/local/lib)
install(FILES ${HEADERS} DESTINATION include)







