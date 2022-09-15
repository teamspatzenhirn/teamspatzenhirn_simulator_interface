/**
 * @file    main.cpp
 * @author  Dominik Authaler
 * @date    31.12.2021
 */

#include "SimulatorDepthNode.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimulatorDepthNode>());
    rclcpp::shutdown();
    return 0;
}
