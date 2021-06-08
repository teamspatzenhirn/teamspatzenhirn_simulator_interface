/**
 * @file SimulatorImageNode.hpp
 * @author jonasotto
 * @date 4/28/21
 */

#ifndef SIMULATORFILTERS_SIMULATORIMAGENODE_HPP
#define SIMULATORFILTERS_SIMULATORIMAGENODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <spatz_interfaces/msg/extrinsic_calib.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include "ImageProcessing/lib/cameraParams.h"
#include "SimulatorFilters/lib/shm_ids.h"
#include "SimulatorFilters/lib/shmcomm.h"

/**
 * @brief Node receiving camera image from simulator
 *
 * Publishes raw camera image with CameraInfo on main_camera_image topic (ImageWithParams)
 *
 * Publishes static transform spatz->camera
 *
 * @ingroup ROSNodes
 */
class SimulatorImageNode : public rclcpp::Node {
  public:
    SimulatorImageNode();
    ~SimulatorImageNode() override;

    SimulatorImageNode(const SimulatorImageNode &) = delete;
    SimulatorImageNode &operator=(const SimulatorImageNode &) = delete;

    SimulatorImageNode(SimulatorImageNode &&) = delete;
    SimulatorImageNode &operator=(SimulatorImageNode &&) = delete;

  private:
    rclcpp::TimerBase::SharedPtr timer;
    void timer_callback();

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imagePublisher;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cameraInfoPublisher;
    rclcpp::Publisher<spatz_interfaces::msg::ExtrinsicCalib>::SharedPtr extrinsicPublisher;

    SimulatorSHM::SHMComm<ImageObject> rx;

    // Simulator camera properties
    sensor_msgs::msg::CameraInfo camera_info;
    std::array<float, 3 * 3> to_img;

    tf2_ros::StaticTransformBroadcaster cameraFrameBroadcaster;

    static camera::cameraParam getSimParams();

    /**
     * Calculates transformation from 3D point too camera coordinate frame using to_img perspective transform in
     * camParam. See OpenCV solvePnP documentation for details.
     * @return Rotation matrix, Translation vector
     */
    static std::pair<cv::Matx33d, cv::Vec3d> transformFromCamParams(const camera::cameraParam &camParam);
};

#endif // SIMULATORFILTERS_SIMULATORIMAGENODE_HPP