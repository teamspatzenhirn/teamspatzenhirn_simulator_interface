/**
 * @file SimulatorVisualOutNode.cpp
 * @author jonasotto
 * @date 8.8.2021
 */

#include "SimulatorVisualOutNode.hpp"

//#include <Trajectory/ros/TrajectoryNode/InterpolatedTrajectory.hpp>

SimulatorVisualOutNode::SimulatorVisualOutNode(const std::string &name) :
    rclcpp::Node(name),
    tx(SimulatorSHM::SERVER, SHM_VISOUT_ID),
    trajectorySub(create_subscription<spatz_interfaces::msg::Trajectory>(
            "trajectory", rclcpp::QoS(1).best_effort(),
            [this](spatz_interfaces::msg::Trajectory::ConstSharedPtr m) { onTrajectoryIn(std::move(m)); })) {
    if (!tx.attach()) {
        RCLCPP_ERROR(get_logger(), "Cannot open shared memory interface!");
        throw std::runtime_error("Cannot open shared memory interface!");
    }
    RCLCPP_INFO(get_logger(), "Node starting");
}

void SimulatorVisualOutNode::onTrajectoryIn(spatz_interfaces::msg::Trajectory::ConstSharedPtr trajectoryMessage) {

    VisualizationOut *visOut = tx.lock(SimulatorSHM::WRITE_OVERWRITE_OLDEST);

    if (nullptr == visOut) {
        RCLCPP_ERROR(get_logger(), "Cannot send visualization output: Fifo full!");
    } else {
        assert(trajectoryMessage->states.size() > 1);
        // TODO: Implement this. This hasn't been done so far because it uses the InterpolatedTrajectory class which
        //  i don't want to move to this package from the (closed-source) Team-Spatzenhirn monorepo.
        //  It doesn't do anything more fancy than linearly interpolating all trajectory values.
        /*
        InterpolatedTrajectory trajectory(*trajectoryMessage);
        int N = sizeof(visOut->trajectoryPoints) / sizeof(visOut->trajectoryPoints[0]);
        double stepSize = (trajectory.endTime() - trajectory.startTime()) / N;

        for (int i = 0; i < N; i++) {
            double t = trajectory.startTime() + stepSize * i;
            visOut->trajectoryPoints[i] = trajectory.pos(t);
        }

        RCLCPP_INFO(get_logger(), "Sending trajectory visualization to simulator");
         */
        RCLCPP_WARN(get_logger(),
                    "The conversion from ROS message for trajectory visualization is not yet implemented");
        tx.unlock(visOut);
    }
}
