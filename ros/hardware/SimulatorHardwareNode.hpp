/**
 * @file   SimulatorHardwareNode.hpp
 * @author Dominik Authaler
 * @date   03.07.2021
 */

#ifndef SIMULATORFILTERS_SIMULATOR_HARDWARE_NODE_HPP
#define SIMULATORFILTERS_SIMULATOR_HARDWARE_NODE_HPP

#include <SimulatorFilters/lib/shm_ids.h>
#include <SimulatorFilters/lib/shmcomm.h>
#include <rclcpp/rclcpp.hpp>
#include <spatz_interfaces/msg/control_vars_vesc.hpp>

/**
 * @brief Node communicating the controller commands to the simulator
 * @ingroup ROSNodes
 */
class SimulatorHardwareNode : public rclcpp::Node {
  public:
    explicit SimulatorHardwareNode(const std::string &name = "SimulatorHardwareNode");
    ~SimulatorHardwareNode() override;

    // Copy: delete to prevent bricking shared memory transport
    SimulatorHardwareNode(const SimulatorHardwareNode &) = delete;
    SimulatorHardwareNode &operator=(const SimulatorHardwareNode &) = delete;

    SimulatorHardwareNode(SimulatorHardwareNode &&) = delete;
    SimulatorHardwareNode &operator=(SimulatorHardwareNode &&) = delete;

  private:
    SimulatorSHM::SHMComm<HardwareOut> tx;

    rclcpp::Subscription<spatz_interfaces::msg::ControlVarsVESC>::SharedPtr controlSub;

    void onControlIn(const spatz_interfaces::msg::ControlVarsVESC::ConstSharedPtr &controlMessage);
};

#endif // SIMULATORFILTERS_SIMULATOR_HARDWARE_NODE_HPP
