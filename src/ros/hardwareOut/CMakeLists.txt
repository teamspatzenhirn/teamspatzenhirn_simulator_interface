cmake_minimum_required(VERSION 3.5)

project(SimulatorHardwareOutNode)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(spatz_interfaces REQUIRED)

add_executable(${PROJECT_NAME} SimulatorHardwareOutNode.cpp main.cpp)
default_install_binary(${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} SharedMemLib)
ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        spatz_interfaces)
