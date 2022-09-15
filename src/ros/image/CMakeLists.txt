cmake_minimum_required(VERSION 3.5)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(spatz_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

project(SimulatorImageNodeLib)
add_library(${PROJECT_NAME} SimulatorImageNode.cpp)
target_link_libraries(${PROJECT_NAME}
        opencv_core
        opencv_imgproc
        opencv_calib3d
        SharedMemLib
        SimCameraCalib)
ament_target_dependencies(${PROJECT_NAME} rclcpp spatz_interfaces tf2_ros sensor_msgs tf2_geometry_msgs)

project(SimulatorImageNode)
add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} SimulatorImageNodeLib)
ament_target_dependencies(${PROJECT_NAME} rclcpp)
