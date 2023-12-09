cmake_minimum_required(VERSION 3.0.2)
project(second_coursework)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    geometry_msgs
    actionlib_msgs
    message_generation
)

catkin_python_setup()

add_service_files(
    FILES
    GetRoomCoord.srv
)

add_action_files(
    FILES
    Search.action
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    actionlib_msgs
    second_coursework
)

catkin_package(
    CATKIN_DEPENDS 
        rospy 
        actionlib_msgs 
        message_runtime 
        std_msgs 
        geometry_msgs
    # INCLUDE_DIRS include
    # LIBRARIES second_coursework
    # DEPENDS system_lib
)

include_directories(
    # include
    ${catkin_INCLUDE_DIRS}
)