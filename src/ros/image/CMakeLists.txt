cmake_minimum_required(VERSION 3.5)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(spatz_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)

project(SimulatorImageNodeLib)
add_carolo_library(${PROJECT_NAME} SimulatorImageNode.cpp)
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

project(SimulatorImageNode)
add_carolo_rosnode(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} SimulatorImageNodeLib)
ament_target_dependencies(${PROJECT_NAME} rclcpp)
