/**
 * @file   SimulatorHardwareOutNode.hpp
 * @author Dominik Authaler
 * @date   03.07.2021
 */

#ifndef SIMULATORFILTERS_SIMULATOR_HARDWARE_OUT_NODE_HPP
#define SIMULATORFILTERS_SIMULATOR_HARDWARE_OUT_NODE_HPP

#include <SimulatorFilters/lib/shm_ids.h>
#include <SimulatorFilters/lib/shmcomm.h>
#include <rclcpp/rclcpp.hpp>
#include <spatz_interfaces/msg/control_setpoint.hpp>

/**
 * @brief Node communicating the controller commands to the simulator
 * @ingroup ROSNodes
 */
class SimulatorHardwareOutNode : public rclcpp::Node {
  public:
    explicit SimulatorHardwareOutNode(const std::string &name = "SimulatorHardwareNode");
    ~SimulatorHardwareOutNode() override;

    // Copy: delete to prevent bricking shared memory transport
    SimulatorHardwareOutNode(const SimulatorHardwareOutNode &) = delete;
    SimulatorHardwareOutNode &operator=(const SimulatorHardwareOutNode &) = delete;

    SimulatorHardwareOutNode(SimulatorHardwareOutNode &&) = delete;
    SimulatorHardwareOutNode &operator=(SimulatorHardwareOutNode &&) = delete;

  private:
    SimulatorSHM::SHMComm<HardwareOut> tx;

    rclcpp::Subscription<spatz_interfaces::msg::ControlSetpoint>::SharedPtr controlSub;

    void onControlIn(const spatz_interfaces::msg::ControlSetpoint::ConstSharedPtr &controlMessage);
};

#endif // SIMULATORFILTERS_SIMULATOR_HARDWARE_OUT_NODE_HPP
