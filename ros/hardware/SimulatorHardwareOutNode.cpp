/**
 * @file   SimulatorHardwareNode.cpp
 * @author Dominik Authaler
 * @date   03.07.2021
 */

#include "SimulatorHardwareOutNode.hpp"

SimulatorHardwareOutNode::SimulatorHardwareOutNode(const std::string &name) :
    rclcpp::Node(name),
    tx(SimulatorSHM::SERVER, SHM_HWOUT_ID),
    controlSub(create_subscription<spatz_interfaces::msg::ControlSetpoint>(
            "/vesc", rclcpp::QoS(1).reliable(),
            [this](spatz_interfaces::msg::ControlSetpoint::ConstSharedPtr m) { onControlIn(m); })) {
    if (!tx.attach()) {
        RCLCPP_ERROR(get_logger(), "SimulatorHardwareNode cannot open shared memory interface!");
        throw std::runtime_error("SimulatorHardwareNode cannot open shared memory interface!");
    } else {
        RCLCPP_INFO(get_logger(), "SimulatorHardwareNode node starting");
    }
}


SimulatorHardwareOutNode::~SimulatorHardwareOutNode() {
    tx.detach();
}

void SimulatorHardwareOutNode::onControlIn(
        const spatz_interfaces::msg::ControlSetpoint::ConstSharedPtr &controlMessage) {
    auto hwOut = tx.lock(SimulatorSHM::WRITE_OVERWRITE_OLDEST);

    if (hwOut == nullptr) {
        RCLCPP_ERROR(get_logger(), "SimulatorHardwareNode cannot send hardware output: FIFO full!");
        return;
    }

    hwOut->vel = controlMessage->vel;
    hwOut->deltaFront = controlMessage->delta_front;
    hwOut->deltaRear = controlMessage->delta_rear;

    tx.unlock(hwOut);
}
