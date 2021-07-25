cmake_minimum_required(VERSION 3.5)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(spatz_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(SimulatorImageNode SimulatorImageNode.cpp main.cpp)
target_link_libraries(SimulatorImageNode
        opencv_core
        opencv_imgproc
        opencv_calib3d
        LoggingLib
        SharedMemLib
        rosUtil
        opencvUtil)
ament_target_dependencies(SimulatorImageNode rclcpp spatz_interfaces tf2_ros)

install(TARGETS SimulatorImageNode
        DESTINATION lib/${PROJECT_NAME})
