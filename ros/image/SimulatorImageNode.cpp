/**
 * @file SimulatorImageNode.cpp
 * @author jonasotto
 * @date 4/28/21
 */

#include "SimulatorImageNode.hpp"

#include <SimulatorFilters/lib/SimCameraCalib.hpp>
#include <chrono>
#include <opencv2/calib3d.hpp>
#include <sensor_msgs/fill_image.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "Util/ros/camParam_conversion.hpp"
#include "Util/ros/opencv_typeconversion.hpp"
#include "Util/ros/tf2_opencv_conversion.hpp"

using namespace std::chrono_literals;

SimulatorImageNode::SimulatorImageNode() :
    Node("simulator_input"),
    rx(SimulatorSHM::CLIENT, SHM_IMAGE_ID),
    cameraFrameBroadcaster(this) {

    std::tie(camera_info, to_img) = messageFromCamParams(getSimParams());

    rx.attach();

    imagePublisher = create_publisher<sensor_msgs::msg::Image>("camera/image_raw",
                                                               rclcpp::QoS(0).best_effort().durability_volatile());

    cameraInfoPublisher = create_publisher<sensor_msgs::msg::CameraInfo>(
            "camera/camera_info", rclcpp::QoS(0).best_effort().durability_volatile());

    extrinsicPublisher = create_publisher<spatz_interfaces::msg::ExtrinsicCalib>(
            "camera/extrinsic_calib", rclcpp::QoS(1).reliable().transient_local());

    timer = create_wall_timer(0s, [this] { timer_callback(); });

    // Publish camera frame transformation
    auto [rotMat, tvec] = transformFromCamParams(getSimParams());
    geometry_msgs::msg::TransformStamped tf_msg{};
    tf_msg.header.frame_id = "spatz";
    tf_msg.header.stamp = now();
    tf_msg.child_frame_id = "camera";
    tf2::convert(tf2::Transform(tf2MatFromCV(rotMat), tf2VecFromCV(tvec)).inverse(), tf_msg.transform);
    cameraFrameBroadcaster.sendTransform(tf_msg);
}

void SimulatorImageNode::timer_callback() {
    auto imageMessage = std::make_unique<sensor_msgs::msg::Image>();

    ImageObject *image = rx.lock(SimulatorSHM::READ_OLDEST);

    static int failCounter = 0;
    if (image == nullptr) {
        failCounter++;
        if (failCounter > 1000) {
            auto clock = rclcpp::Clock();
            RCLCPP_WARN_THROTTLE(get_logger(), clock, 1000, "Cannot get image: Server not sending messages");
        }
        std::this_thread::sleep_for(1ms);
        return;
    }
    failCounter = 0;

    RCLCPP_DEBUG(get_logger(), "Simulator: Image from Server");

    sensor_msgs::fillImage(*imageMessage, sensor_msgs::image_encodings::BAYER_RGGB8, image->imageHeight,
                           image->imageWidth, image->imageWidth, image->rgbdata);

    rx.unlock(image);

    auto stamp = now();
    const auto *frame = "camera";

    imageMessage->header.frame_id = frame;
    imageMessage->header.stamp = stamp;

    RCLCPP_INFO(get_logger(), "Publishing image");
    imagePublisher->publish(std::move(imageMessage));

    auto cameraInfoMessage = std::make_unique<sensor_msgs::msg::CameraInfo>();
    *cameraInfoMessage = camera_info;
    cameraInfoMessage->header.stamp = stamp;
    cameraInfoMessage->header.frame_id = frame;
    cameraInfoPublisher->publish(std::move(cameraInfoMessage));

    auto extrMessage = std::make_unique<spatz_interfaces::msg::ExtrinsicCalib>();
    extrMessage->to_img = to_img;
    extrMessage->undist_needed = false;
    extrMessage->stamp = stamp;
    extrinsicPublisher->publish(std::move(extrMessage));
}

SimulatorImageNode::~SimulatorImageNode() {
    rx.detach();
}

camera::cameraParam SimulatorImageNode::getSimParams() {
    camera::cameraParam simCamParam;
    simCamParam.size = SIM_CAMERA_SIZE;
    simCamParam.camMat = SIM_CAMERA_MATRIX;
    simCamParam.distTanMat = SIM_TANGENTIAL_DISTORTION_COEFFICIENTS;
    simCamParam.distRadMat = SIM_RADIAL_DISTORTION_COEFFICIENTS;
    simCamParam.roiMat = SIM_NEW_CAMERA_MATRIX;
    simCamParam.toImg = SIM_FLOORPLANE_TO_IMAGE_MATRIX;
    simCamParam.toCar = simCamParam.toImg.inv();
    simCamParam.undistNeeded = false;
    simCamParam.isSet = true;
    return simCamParam;
}

std::pair<cv::Matx33d, cv::Vec3d> SimulatorImageNode::transformFromCamParams(const camera::cameraParam &camParam) {
    // Default calib points from CalibTool (need z=0 to apply perspective transform)
    std::vector<cv::Point3d> spatzCoordinatePoints = {{0.5, -0.5, 0}, {0.5, 0.5, 0}, {1.2, 0.5, 0}, {1.2, -0.5, 0}};

    // Transform points to image coordinates
    std::vector<cv::Point2d> imageCoordinates;
    for (const auto &sp : spatzCoordinatePoints) {
        auto imgCord = camParam.transToImg(cv::Point2f(sp.x * 1000, sp.y * 1000));
        imageCoordinates.emplace_back(imgCord);
    }

    auto roiMat = camParam.roiMat;
    cv::Vec<double, 3> rotationVector;
    cv::Vec<double, 3> translation;
    cv::Matx<float, 1, 5> coeffs(camParam.distRadMat(0, 0), camParam.distRadMat(0, 1), camParam.distTanMat(0, 0),
                                 camParam.distTanMat(0, 1), camParam.distRadMat(0, 2));

    // Solve for rotation rotationVector and translation translation
    cv::solvePnP(spatzCoordinatePoints, imageCoordinates, roiMat, coeffs, rotationVector, translation);

    // Rotation vector to matrix
    cv::Matx<double, 3, 3> rotationMatrix;
    cv::Rodrigues(rotationVector, rotationMatrix);

    return {rotationMatrix, translation};
}