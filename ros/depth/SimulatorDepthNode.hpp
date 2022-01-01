/**
 * @file   SimulatorDepthNode.hpp
 * @author Dominik Authaler
 * @date   31.12.2021
 */

#ifndef SIMULATORFILTERS_SIMULATORDEPTHNODE_HPP
#define SIMULATORFILTERS_SIMULATORDEPTHNODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <spatz_interfaces/msg/extrinsic_calib.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include "ImageProcessing/lib/cameraParams.h"
#include "SimulatorFilters/lib/SimCameraCalib.hpp"
#include "SimulatorFilters/lib/shm_ids.h"
#include "SimulatorFilters/lib/shmcomm.h"

/**
 * @brief Node receiving point clouds from simulator.
 *
 * Publishes the point cloud generated from the simulator. This corresponds to the point cloud obtained
 * after fusing the two individual ones on the real hardware. Additionally, a filtered point cloud is published. It
 * consists of every fourth point of the original one, leading to a point cloud with half the width and height of
 * the original one.
 *
 * @ingroup ROSNodes
 */
class SimulatorDepthNode : public rclcpp::Node {
  public:
    SimulatorDepthNode();
    ~SimulatorDepthNode() override;

    SimulatorDepthNode(const SimulatorDepthNode &) = delete;
    SimulatorDepthNode &operator=(const SimulatorDepthNode &) = delete;

    SimulatorDepthNode(SimulatorDepthNode &&) = delete;
    SimulatorDepthNode &operator=(SimulatorDepthNode &&) = delete;

  private:
    rclcpp::TimerBase::SharedPtr timer;
    void timer_callback();

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloudPublisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filteredPointCloudPublisher;

    SimulatorSHM::SHMComm<DepthObject> rx;
};

#endif // SIMULATORFILTERS_SIMULATORDEPTHNODE_HPP