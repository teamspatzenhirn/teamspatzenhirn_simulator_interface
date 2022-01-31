cmake_minimum_required(VERSION 3.5)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(spatz_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)

project(SimulatorImageNode)

add_executable(${PROJECT_NAME} SimulatorImageNode.cpp main.cpp)
target_link_libraries(${PROJECT_NAME}
        opencv_core
        opencv_imgproc
        opencv_calib3d
        LoggingLib
        SharedMemLib
        rosUtil
        opencvUtil
        SimCameraCalib)
ament_target_dependencies(${PROJECT_NAME} rclcpp spatz_interfaces tf2_ros)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD ${NODE_CXX_STANDARD})

default_install_binary(${PROJECT_NAME})
