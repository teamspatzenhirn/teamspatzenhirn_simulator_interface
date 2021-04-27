cmake_minimum_required(VERSION 3.5)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(spatz_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)


add_executable(SimulatorSpatzNode SimulatorSpatzNode.cpp main.cpp)
target_link_libraries(SimulatorSpatzNode SharedMemLib rosUtil)
ament_target_dependencies(SimulatorSpatzNode
        rclcpp
        spatz_interfaces
        tf2
        tf2_ros
        tf2_geometry_msgs
        visualization_msgs
        sensor_msgs)

default_install_binary(SimulatorSpatzNode)
