/**
 * @file SimulatorVisualOutNode.hpp
 * @author jonasotto
 * @date 8.8.2021
 */

#ifndef SIMULATORFILTERS_SIMULATORVISUALOUTNODE_HPP
#define SIMULATORFILTERS_SIMULATORVISUALOUTNODE_HPP

#include <lib/shm_ids.h>
#include <lib/shmcomm.h>
#include <rclcpp/rclcpp.hpp>
#include <spatz_interfaces/msg/trajectory.hpp>

/**
 * @brief Node receiving trajectory for visualization in simulator
 * @ingroup ROSNodes
 */
class SimulatorVisualOutNode : public rclcpp::Node {
  public:
    explicit SimulatorVisualOutNode(const std::string &name);

  private:
    SimulatorSHM::SHMComm<VisualizationOut> tx;

    rclcpp::Subscription<spatz_interfaces::msg::Trajectory>::SharedPtr trajectorySub;
    void onTrajectoryIn(spatz_interfaces::msg::Trajectory::ConstSharedPtr trajectory);
};


#endif // SIMULATORFILTERS_SIMULATORVISUALOUTNODE_HPP
