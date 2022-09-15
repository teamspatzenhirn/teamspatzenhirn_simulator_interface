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

#include "lib/shm_ids.h"
#include "lib/shmcomm.h"

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
};

struct cameraParam {
    cv::Size size = {0, 0}; ///< Size of the camera image.
    cv::Matx33f camMat;
    cv::Matx<float, 1, 2> distTanMat; ///< Tangential distortion matrix.
    cv::Matx<float, 1, 3> distRadMat; ///< Radial distortion matrix.

    cv::Matx33f roiMat; ///< Virtual camera matrix after undistortion.

    cv::Matx33f toImg; ///< Matrix for conversion from car to img.
    cv::Matx33f toCar; ///< Matrix for conversion from img to car.

    /**
     * Point conversion from car to image coordinate system.
     * @param carCord Point in the car coordinate system.
     * @return Corresponding point in the image coordinate system.
     */
    [[nodiscard]] cv::Point2f transToImg(const cv::Point2f &carCord) const;
};

#endif // SIMULATORFILTERS_SIMULATORIMAGENODE_HPP