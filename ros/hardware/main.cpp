#include <rclcpp/rclcpp.hpp>

#include "SimulatorHardwareOutNode.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimulatorHardwareOutNode>());
    rclcpp::shutdown();
    return 0;
}
