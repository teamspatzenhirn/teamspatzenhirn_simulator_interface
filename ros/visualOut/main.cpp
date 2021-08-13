#include <rclcpp/rclcpp.hpp>

#include "SimulatorVisualOutNode.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimulatorVisualOutNode>("SimulatorVisualOutNode"));
    rclcpp::shutdown();
    return 0;
}
