/**
 * @file SimulatorSpatzNode.hpp
 * @author jonasotto
 * @date 4/1/21
 */

#ifndef SIMULATORFILTERS_SIMULATORSPATZNODE_HPP
#define SIMULATORFILTERS_SIMULATORSPATZNODE_HPP

#include <SimulatorFilters/lib/shm_ids.h>
#include <SimulatorFilters/lib/shmcomm.h>
#include <Spatz/lib/Spatz.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <spatz_interfaces/msg/rc_mode.hpp>
#include <spatz_interfaces/msg/spatz.hpp>

/**
 * @brief Node receiving Spatz from simulator. Publishes spatz and rc-mode.
 * @ingroup ROSNodes
 */
class SimulatorSpatzNode : public rclcpp::Node {
  public:
    SimulatorSpatzNode();
    ~SimulatorSpatzNode() override;

    // Copy: delete to prevent bricking shared memory transport
    SimulatorSpatzNode(const SimulatorSpatzNode &) = delete;
    SimulatorSpatzNode &operator=(const SimulatorSpatzNode &) = delete;

    SimulatorSpatzNode(SimulatorSpatzNode &&) = delete;
    SimulatorSpatzNode &operator=(SimulatorSpatzNode &&) = delete;

  private:
    SimulatorSHM::SHMComm<HardwareIn> rx;

    rclcpp::Time lastSpatzTime = rclcpp::Time(static_cast<int64_t>(0), rcl_clock_type_t::RCL_ROS_TIME);

    bool prevPaused = false;

    rclcpp::Publisher<spatz_interfaces::msg::Spatz>::SharedPtr spatzPublisher;
    rclcpp::Publisher<spatz_interfaces::msg::RCMode>::SharedPtr rcModePublisher;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clockPublisher;

    rclcpp::TimerBase::SharedPtr timer;
    void timerCallback();

    void onSpatzLivelinessLost(const rclcpp::QOSLivelinessLostInfo &info) const;

    static env::Spatz spatzFromHWIn(const HardwareIn &hwIn);
};

#endif // SIMULATORFILTERS_SIMULATORSPATZNODE_HPP
