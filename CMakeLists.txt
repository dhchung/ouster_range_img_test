cmake_minimum_required(VERSION 3.0.2)
project(ouster_range_img_test)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ouster_range_img_test
 CATKIN_DEPENDS geometry_msgs pcl_conversions pcl_ros roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)
add_executable(${PROJECT_NAME}_node src/main.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node_32 src/ouster32.cpp)

target_link_libraries(${PROJECT_NAME}_node_32
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)