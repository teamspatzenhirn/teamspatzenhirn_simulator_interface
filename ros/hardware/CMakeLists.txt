cmake_minimum_required(VERSION 3.5)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(spatz_interfaces REQUIRED)

set(NODE_NAME SimulatorHardwareNode)

add_executable(${NODE_NAME} SimulatorHardwareNode.cpp main.cpp)
target_link_libraries(${NODE_NAME} SharedMemLib rosUtil)
ament_target_dependencies(${NODE_NAME}
        rclcpp
        spatz_interfaces)

default_install_binary(${NODE_NAME})
