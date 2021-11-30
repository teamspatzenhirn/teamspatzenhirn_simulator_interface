cmake_minimum_required(VERSION 3.5)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(spatz_interfaces REQUIRED)

add_executable(SimulatorVisualOutNode SimulatorVisualOutNode.cpp main.cpp)
target_link_libraries(SimulatorVisualOutNode SharedMemLib InterpolatedTrajectory)
ament_target_dependencies(SimulatorVisualOutNode
        rclcpp
        spatz_interfaces
        )
set_property(TARGET SimulatorVisualOutNode PROPERTY CXX_STANDARD ${NODE_CXX_STANDARD})

default_install_binary(SimulatorVisualOutNode)
