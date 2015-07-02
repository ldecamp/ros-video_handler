cmake_minimum_required(VERSION 2.8.3)
project(video_handler)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
add_executable(vhandler src/video_handler_node.cpp)
target_link_libraries(vhandler ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

