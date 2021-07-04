#include <rclcpp/rclcpp.hpp>

#include "SimulatorHardwareNode.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimulatorHardwareNode>());
    rclcpp::shutdown();
    return 0;
}
