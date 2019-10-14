cmake_minimum_required(VERSION 2.8.3)
project(rss_map_image_server)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  image_geometry
  nav_msgs
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rss_map_image_server
#  CATKIN_DEPENDS geometry_msgs image_geometry nav_msgs rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)