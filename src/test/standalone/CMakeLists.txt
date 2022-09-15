find_package(tf2 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(spatz_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(simulatorExtrinsic simulatorExtrinsic.main.cpp)
target_link_libraries(simulatorExtrinsic ${OpenCV_LIBS} SimCameraCalib)
ament_target_dependencies(simulatorExtrinsic tf2)
