/**
 * @file SimulatorSpatzNode.hpp
 * @author ottojo
 * @date 4/1/21
 */

#ifndef SRC_SIMULATORSPATZNODE_HPP
#define SRC_SIMULATORSPATZNODE_HPP

#include <SimulatorFilters/lib/shm_ids.h>
#include <SimulatorFilters/lib/shmcomm.h>
#include <Spatz/lib/Spatz.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <spatz_interfaces/msg/rc_mode.hpp>
#include <spatz_interfaces/msg/spatz.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

/**
 * @brief Node receiving Spatz from simulator
 *
 * Mainly publishes spatz and rc-mode.
 * Additionally publishes the laser measurement separately, map->spatz transformation, box-marker for rviz.
 * Optionally publishes simulation time from spatz input (use_sim_time parameter).
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

    bool prevBinaryLightSensorTriggered = false;
    bool prevPaused = false;

    rclcpp::Publisher<spatz_interfaces::msg::Spatz>::SharedPtr spatzPublisher;
    rclcpp::Publisher<spatz_interfaces::msg::RCMode>::SharedPtr rcModePublisher;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr laserPublisher;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr rearLaserPublisher;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markerPublisher;

    tf2_ros::TransformBroadcaster tfBroadcaster;
    tf2_ros::StaticTransformBroadcaster tfStaticBroadcaster;

    rclcpp::TimerBase::SharedPtr timer;
    void timerCallback();

    void onSpatzLivelinessLost(const rclcpp::QOSLivelinessLostInfo &info);

    env::Spatz spatzFromHWIn(const HardwareIn &hwIn);

    static auto buildSpatzTransform(const rclcpp::Time &stamp, const env::Spatz &spatz)
            -> geometry_msgs::msg::TransformStamped;

    static sensor_msgs::msg::Range buildLaserMessage(const rclcpp::Time &stamp, float measurement);

    static visualization_msgs::msg::Marker buildSpatzMarker(const rclcpp::Time &stamp);

    /**
     * Publish position of laser relative to spatz
     */
    void publishStaticLaserTF();
};

#endif // SRC_SIMULATORSPATZNODE_HPP
