cmake_minimum_required(VERSION 3.5)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(spatz_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED) # Time data type for /clock

project(SimulatorSpatzNode)

add_carolo_rosnode(${PROJECT_NAME} SimulatorSpatzNode.cpp main.cpp)
target_link_libraries(${PROJECT_NAME} SharedMemLib rosUtil SpatzXLib)
ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        spatz_interfaces
        tf2
        tf2_ros
        tf2_geometry_msgs
        visualization_msgs
        sensor_msgs)
