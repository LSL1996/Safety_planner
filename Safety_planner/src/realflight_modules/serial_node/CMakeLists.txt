cmake_minimum_required(VERSION 3.0.2)
project(serial_node)
set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        serial
        quadrotor_msgs
        geometry_msgs
        uav_utils
        std_msgs
        sensor_msgs
        nav_msgs
        message_generation
)
add_message_files(FILES
        my_msg.msg
        rawpose.msg
        rc.msg
        extforce.msg
)
generate_messages(DEPENDENCIES
        std_msgs
        sensor_msgs
        quadrotor_msgs
        geometry_msgs
        nav_msgs
        )
catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS
        message_runtime
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        "mavlink_headers"
        "serial_node/cmake-build-debug/devel/include"
        "usr/local/include/eigen3"
)

add_executable(6inc_ctrl_node src/6inc_ctrl_node.cpp
)

add_library(6inc_ctrl_node_lib src/6inc_ctrl_node.cpp)

add_dependencies(6inc_ctrl_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})  #该项让编译器知道要注意编译顺序
add_dependencies(6inc_ctrl_node ${PROJECT_NAME}_generate_messages_cpp  ${PROJECT_NAME}_gencpp)
target_link_libraries(6inc_ctrl_node ${catkin_LIBRARIES} )
