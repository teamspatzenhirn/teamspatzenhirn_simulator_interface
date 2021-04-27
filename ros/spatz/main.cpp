#include <rclcpp/rclcpp.hpp>

#include "SimulatorSpatzNode.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimulatorSpatzNode>());
    rclcpp::shutdown();
    return 0;
}
