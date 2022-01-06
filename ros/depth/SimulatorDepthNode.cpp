/**
 * @file   SimulatorDepthNode.cpp
 * @author Dominik Authaler
 * @date   31.12.2021
 */

#include "SimulatorDepthNode.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "Util/ros/publishUtil.hpp"


using namespace std::chrono_literals;

SimulatorDepthNode::SimulatorDepthNode() :
    Node("simulator_depth_input"),
    timer(create_wall_timer(0s, [this] { timer_callback(); })),
    pointCloudPublisher(create_publisher<sensor_msgs::msg::PointCloud2>(
            "depth/fused_pointcloud", rclcpp::QoS(0).best_effort().durability_volatile())),
    filteredPointCloudPublisher(create_publisher<sensor_msgs::msg::PointCloud2>(
            "depth/filtered_fused_pointcloud", rclcpp::QoS(0).best_effort().durability_volatile())),
    rx(SimulatorSHM::CLIENT, SHM_DEPTH_ID) {

    rx.attach();
}

void SimulatorDepthNode::timer_callback() {
    bool generateRaw = (pointCloudPublisher->get_subscription_count() > 0);
    bool generateFiltered = (filteredPointCloudPublisher->get_subscription_count() > 0);

    // performing the conversion is only worth if there are subscribers
    if (!generateRaw and !generateFiltered) {
        return;
    }

    DepthObject *depthImage = rx.lock(SimulatorSHM::READ_OLDEST);

    if (depthImage == nullptr) {
        rclcpp::Clock steady_clock(RCL_STEADY_TIME);

        // warn only once per second
        RCLCPP_DEBUG_THROTTLE(get_logger(), steady_clock, 1000, "Simulator: No depth image from Server");
        std::this_thread::sleep_for(1ms);
        return;
    }

    RCLCPP_DEBUG(get_logger(), "Simulator: Depth image from Server");

    auto pointCloudMessage = std::make_unique<sensor_msgs::msg::PointCloud2>();
    auto filteredPointCloudMessage = std::make_unique<sensor_msgs::msg::PointCloud2>();

    // the fused point cloud is always published in spatz / vehicle coordinates --> frame "spatz"
    const auto frame = "spatz";
    const auto stamp = now();

    std::optional<sensor_msgs::PointCloud2Iterator<float>> iterX;
    std::optional<sensor_msgs::PointCloud2Iterator<float>> iterY;
    std::optional<sensor_msgs::PointCloud2Iterator<float>> iterZ;
    std::optional<sensor_msgs::PointCloud2Iterator<float>> filteredIterX;
    std::optional<sensor_msgs::PointCloud2Iterator<float>> filteredIterY;
    std::optional<sensor_msgs::PointCloud2Iterator<float>> filteredIterZ;

    if (generateRaw) {
        // set message header fields
        pointCloudMessage->header.frame_id = frame;
        pointCloudMessage->header.stamp = stamp;

        // set basic fields
        pointCloudMessage->height = SIM_DEPTH_CAMERA_HEIGHT;
        pointCloudMessage->width = SIM_DEPTH_CAMERA_WIDTH;

        sensor_msgs::PointCloud2Modifier modifier(*pointCloudMessage);

        // implicitly resizes the data field of the message
        modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                                      sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                                      sensor_msgs::msg::PointField::FLOAT32);

        iterX = sensor_msgs::PointCloud2Iterator<float>(*pointCloudMessage, "x");
        iterY = sensor_msgs::PointCloud2Iterator<float>(*pointCloudMessage, "y");
        iterZ = sensor_msgs::PointCloud2Iterator<float>(*pointCloudMessage, "z");
    }

    if (generateFiltered) {
        // set message header fields
        filteredPointCloudMessage->header.frame_id = frame;
        filteredPointCloudMessage->header.stamp = stamp;

        // set basic fields
        filteredPointCloudMessage->height = SIM_DEPTH_CAMERA_HEIGHT / 2;
        filteredPointCloudMessage->width = SIM_DEPTH_CAMERA_WIDTH / 2;

        sensor_msgs::PointCloud2Modifier filteredModifier(*filteredPointCloudMessage);

        // implicitly resizes the data field of the message
        filteredModifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                                              sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                                              sensor_msgs::msg::PointField::FLOAT32);

        filteredIterX = sensor_msgs::PointCloud2Iterator<float>(*filteredPointCloudMessage, "x");
        filteredIterY = sensor_msgs::PointCloud2Iterator<float>(*filteredPointCloudMessage, "y");
        filteredIterZ = sensor_msgs::PointCloud2Iterator<float>(*filteredPointCloudMessage, "z");
    }

    // extract the point cloud
    const auto *const cvDepthPoints = reinterpret_cast<cv::Point3f *>(depthImage->depthPoints);

    for (std::size_t y = 0; y < SIM_DEPTH_CAMERA_HEIGHT; ++y) {
        for (std::size_t x = 0; x < SIM_DEPTH_CAMERA_WIDTH; ++x) {
            const auto &d435Pt = cvDepthPoints[y * SIM_DEPTH_CAMERA_WIDTH + x];
            /*
             * Transformation between the coordinate system of the simulated depth camera (image coordinates: x to
             * the right, y downwards and z for the depth) to the vehicle coordinate system
             */
            const cv::Point3f pt{d435Pt.z + SIM_DEPTH_CAMERA_TRANS_X, -d435Pt.x + SIM_DEPTH_CAMERA_TRANS_Y,
                                 -d435Pt.y + SIM_DEPTH_CAMERA_TRANS_Z};

            if (generateRaw) {
                *(iterX.value()) = pt.x;
                *(iterY.value()) = pt.y;
                *(iterZ.value()) = pt.z;

                ++(iterX.value());
                ++(iterY.value());
                ++(iterZ.value());
            }

            if (generateFiltered and x % 2 == 0 and y % 2 == 0) {
                *(filteredIterX.value()) = pt.x;
                *(filteredIterY.value()) = pt.y;
                *(filteredIterZ.value()) = pt.z;

                ++(filteredIterX.value());
                ++(filteredIterY.value());
                ++(filteredIterZ.value());
            }
        }
    }

    rx.unlock(depthImage);

    RCLCPP_DEBUG(get_logger(), "Publishing fused pointclouds (filtered and normal)");

    publishIfNeeded(pointCloudPublisher, std::move(pointCloudMessage));
    publishIfNeeded(filteredPointCloudPublisher, std::move(filteredPointCloudMessage));
}

SimulatorDepthNode::~SimulatorDepthNode() {
    rx.detach();
}
