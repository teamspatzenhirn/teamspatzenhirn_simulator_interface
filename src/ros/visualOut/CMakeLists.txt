cmake_minimum_required(VERSION 3.5)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(spatz_interfaces REQUIRED)

project(SimulatorVisualOutNode)

add_executable(${PROJECT_NAME} SimulatorVisualOutNode.cpp main.cpp)
default_install_binary(${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} SharedMemLib #[[InterpolatedTrajectory]])
ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        spatz_interfaces
        )
