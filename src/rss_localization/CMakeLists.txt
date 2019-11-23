cmake_minimum_required(VERSION 2.8.3)
project(rss_localization)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        nav_msgs
        geometry_msgs
        sensor_msgs
        visualization_msgs
        tf
        )

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
        INCLUDE_DIRS include
        LIBRARIES rss_localization
        CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs geometry_msgs sensor_msgs visualization_msgs tf
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(LidarMeasurementModel src/LidarMeasurementModel.cpp)
target_link_libraries(LidarMeasurementModel ${catkin_LIBRARIES})

add_library(MapHandler src/MapHandler.cpp)
target_link_libraries(MapHandler ${catkin_LIBRARIES})

add_library(OdometryMotionModel src/OdometryMotionModel.cpp)
target_link_libraries(OdometryMotionModel ${catkin_LIBRARIES})

add_library(ParticleFilterStateEstimator src/ParticleFilterStateEstimator.cpp)
target_link_libraries(ParticleFilterStateEstimator ${catkin_LIBRARIES})

add_library(ScanProcessor src/ScanProcessor.cpp)
target_link_libraries(ScanProcessor ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME} src/localization.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME} LidarMeasurementModel MapHandler OdometryMotionModel ParticleFilterStateEstimator ScanProcessor)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES} LidarMeasurementModel MapHandler OdometryMotionModel ParticleFilterStateEstimator ScanProcessor)
