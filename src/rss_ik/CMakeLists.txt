cmake_minimum_required(VERSION 2.8.3)
project(rss_ik)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        geometry_msgs
        nav_msgs
        sensor_msgs
        std_msgs
        tf
        visualization_msgs
        message_generation
        )

add_service_files(
        FILES
        GetTargetJointAngles.srv
)

add_message_files(
        DIRECTORY msg
        FILES
        SendIKCommand.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package()

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)
add_executable(${PROJECT_NAME}_server src/ik_server.cpp)
target_link_libraries(${PROJECT_NAME}_server
        ${catkin_LIBRARIES}
        )
add_dependencies(${PROJECT_NAME}_server
        ${PROJECT_NAME}_gencpp
        )

add_executable(${PROJECT_NAME}_command_server src/command_server.cpp)
target_link_libraries(${PROJECT_NAME}_command_server
        ${catkin_LIBRARIES}
        )
add_dependencies(${PROJECT_NAME}_command_server
        ${PROJECT_NAME}_gencpp
        )