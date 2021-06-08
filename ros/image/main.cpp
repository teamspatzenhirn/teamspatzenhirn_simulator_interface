/**
 * @file main.cpp
 * @author jonasotto
 * @date 5/06/21
 */

#include "SimulatorImageNode.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimulatorImageNode>());
    rclcpp::shutdown();
    return 0;
}
