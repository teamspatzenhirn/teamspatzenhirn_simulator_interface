cmake_minimum_required(VERSION 3.5)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

project(SimulatorDepthNode)

add_executable(${PROJECT_NAME} SimulatorDepthNode.cpp main.cpp)
target_link_libraries(${PROJECT_NAME}
        opencv_core
        SharedMemLib
        SimCameraCalib)
ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs)
