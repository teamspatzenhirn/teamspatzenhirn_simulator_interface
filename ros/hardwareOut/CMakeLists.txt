cmake_minimum_required(VERSION 3.5)

project(SimulatorHardwareOutNode)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(spatz_interfaces REQUIRED)

add_carolo_rosnode(${PROJECT_NAME} SimulatorHardwareOutNode.cpp main.cpp)
target_link_libraries(${PROJECT_NAME} SharedMemLib rosUtil)
ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        spatz_interfaces)
