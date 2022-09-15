cmake_minimum_required(VERSION 3.5)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

project(SimulatorDepthNode)

add_carolo_rosnode(${PROJECT_NAME} SimulatorDepthNode.cpp main.cpp)
target_link_libraries(${PROJECT_NAME}
        opencv_core
        LoggingLib
        SharedMemLib
        rosUtil
        SimCameraCalib)
ament_target_dependencies(${PROJECT_NAME} rclcpp)
