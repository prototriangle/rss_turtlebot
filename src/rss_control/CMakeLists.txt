cmake_minimum_required(VERSION 2.8.3)
project(rss_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        nav_msgs
        geometry_msgs
        tf
        )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rss_description
  CATKIN_DEPENDS roscpp nav_msgs geometry_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_library(PID src/PID.cpp)

add_executable(a_star_node src/a_star.cpp)
add_executable(controller_node src/controller.cpp)

target_link_libraries(a_star_node
        ${catkin_LIBRARIES}
        )

target_link_libraries(controller_node
        PID
        ${catkin_LIBRARIES}
        )

add_dependencies(controller_node PID)

#############
## Install ##
#############

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rss_description.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
