cmake_minimum_required(VERSION 3.5)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

project(SimulatorDepthNode)

add_executable(${PROJECT_NAME} SimulatorDepthNode.cpp main.cpp)
target_link_libraries(${PROJECT_NAME}
        opencv_core
        LoggingLib
        SharedMemLib
        rosUtil
        SimCameraCalib)
ament_target_dependencies(${PROJECT_NAME} rclcpp)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD ${NODE_CXX_STANDARD})

default_install_binary(${PROJECT_NAME})
