cmake_minimum_required(VERSION 3.5)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(SimulatorDepthNode SimulatorDepthNode.cpp main.cpp)
target_link_libraries(SimulatorDepthNode
        opencv_core
        LoggingLib
        SharedMemLib
        rosUtil)
ament_target_dependencies(SimulatorDepthNode rclcpp)
set_property(TARGET SimulatorDepthNode PROPERTY CXX_STANDARD ${NODE_CXX_STANDARD})

default_install_binary(SimulatorDepthNode)
