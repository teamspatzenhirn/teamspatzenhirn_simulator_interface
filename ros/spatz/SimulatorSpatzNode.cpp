/**
 * @file SimulatorSpatzNode.cpp
 * @author ottojo
 * @date 4/1/21
 */

#include "SimulatorSpatzNode.hpp"

#include <Util/ros/spatz_conversion.hpp>
#include <gsl/gsl>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

SimulatorSpatzNode::SimulatorSpatzNode() :
    Node("simulator_input"),
    rx(SimulatorSHM::CLIENT, SHM_HWIN_ID),
    tfBroadcaster(this),
    tfStaticBroadcaster(this) {
    rx.attach();
    using namespace rclcpp;
    using namespace spatz_interfaces;
    using namespace std::placeholders;

    PublisherOptions spatzOptions;
    spatzOptions.event_callbacks.liveliness_callback = std::bind(&SimulatorSpatzNode::onSpatzLivelinessLost, this, _1);
    auto spatzQoS = QoS(1).reliable()
                            .liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC)
                            // Expect at least 10 FPS to be considered alive, but no guaranteed Deadline
                            .liveliness_lease_duration(Duration(0, 1e8));
    spatzPublisher = create_publisher<msg::Spatz>("spatz", spatzQoS, spatzOptions);

    // Use TRANSIENT_LOCAL Durability to provide late joining nodes with current state
    rcModePublisher = create_publisher<msg::RCMode>("rcmode", QoS(1).reliable().transient_local());

    const auto bestEffortVolatile = QoS(0).best_effort().durability_volatile();
    laserPublisher = create_publisher<sensor_msgs::msg::Range>("laser_front", bestEffortVolatile);
    publishStaticLaserTF();

    // Reliable QoS, since rviz defaults to "reliable" and silently fails if QoS profiles dont match
    // TODO (jonasotto, April 2021): Rethink QoS whenever introspection and error messages improve in ROS
    markerPublisher = create_publisher<visualization_msgs::msg::Marker>("spatzMarker", QoS(0).reliable());

    // Timer for polling shared memory transport to simulator
    using namespace std::chrono_literals;
    timer = this->create_wall_timer(1ms, std::bind(&SimulatorSpatzNode::timerCallback, this));

    RCLCPP_INFO(get_logger(), "Node Starting");
}

void SimulatorSpatzNode::timerCallback() {
    HardwareIn *inobj = rx.lock(SimulatorSHM::READ_NEWEST);
    if (inobj == nullptr) {
        if (prevPaused) {
            // If simulator is paused, we expect to not receive any data until unpaused
            bool success = spatzPublisher->assert_liveliness();
            Expects(success); // This should only fail if publisher was configured incorrectly...
        }
        return;
    }

    RCLCPP_INFO(get_logger(), "Received Spatz from simulator");

    builtin_interfaces::msg::Time stamp = now();

    // Build and publish spatz
    auto spatz = spatzFromHWIn(*inobj);
    RCLCPP_INFO(get_logger(), "publishing spatz");
    spatzPublisher->publish(messageFromSpatz(spatz));

    // Publish spatz marker for rviz
    // TODO (jonasotto, April 2021): Figure out how to send this only once, since it is frame-locked to spatz frame
    auto marker_msg = buildSpatzMarker(stamp);
    markerPublisher->publish(marker_msg);

    // Publish map->spatz transformation
    auto tf_msg = buildSpatzTransform(stamp, spatz);
    RCLCPP_INFO(get_logger(), "publishing map->spatz transform");
    tfBroadcaster.sendTransform(tf_msg);

    // Publish laser sensor measurement separately. This is technically contained in spatz, but it looks fancy in rviz.
    auto laser_msg = buildLaserMessage(stamp, static_cast<float>(spatz.getLaser()));
    laserPublisher->publish(laser_msg);

    // Publish rcmode if changed
    if (prevPaused != inobj->paused) {
        hw::RCMode mode = inobj->paused ? hw::RCMode::remote : hw::RCMode::adtf;
        auto rcModeMessage = spatz_interfaces::msg::RCMode();
        rcModeMessage.enabled = mode == hw::RCMode::remote;
        rcModePublisher->publish(rcModeMessage);
    }
    prevPaused = inobj->paused;

    rx.unlock(inobj);
}

SimulatorSpatzNode::~SimulatorSpatzNode() {
    rx.detach();
}

void SimulatorSpatzNode::onSpatzLivelinessLost(const rclcpp::QOSLivelinessLostInfo &) {
    // Spatz publisher liveliness lost. Nothing we can do really.
    RCLCPP_WARN(get_logger(), "Spatz publisher lost expected liveliness. Is simulator running?");
}

visualization_msgs::msg::Marker SimulatorSpatzNode::buildSpatzMarker(const rclcpp::Time &stamp) {
    visualization_msgs::msg::Marker marker_msg;
    marker_msg.header.frame_id = "spatz";
    marker_msg.header.stamp = stamp;
    marker_msg.ns = "spatz";
    marker_msg.id = 0;
    marker_msg.type = visualization_msgs::msg::Marker::CUBE;
    marker_msg.action = visualization_msgs::msg::Marker::ADD;
    marker_msg.scale.x = env::Spatz::LENGTH;
    marker_msg.scale.y = env::Spatz::WIDTH;
    marker_msg.scale.z = 0.13;
    marker_msg.pose.position.x = marker_msg.scale.x / 2 - env::Spatz::ORIGIN_X;
    marker_msg.pose.position.z = marker_msg.scale.z / 2;
    // Team-Spatzenhirn-Orange: #F96900 (RGB 249, 105, 0)
    marker_msg.color.r = 249.0 / 255;
    marker_msg.color.g = 105.0 / 255;
    marker_msg.color.b = 0;
    marker_msg.color.a = 0.9; // Slightly transparent so one can see the spatz coordinate frame
    marker_msg.frame_locked = true;
    return marker_msg;
}

void SimulatorSpatzNode::publishStaticLaserTF() {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.frame_id = "spatz";
    tf_msg.header.stamp = now();
    tf_msg.child_frame_id = "laser_front";
    // TODO (jonasotto, April 2021): Confirm this. Those values are used in parken filter
    tf_msg.transform.translation.x = 0.215;
    tf_msg.transform.translation.y = -0.07;
    tf_msg.transform.translation.z = 0.1;
    tf2::Quaternion laserRot;
    laserRot.setRPY(0, 0, -M_PI_2);
    tf2::convert(laserRot, tf_msg.transform.rotation);

    tfStaticBroadcaster.sendTransform(tf_msg);
}

sensor_msgs::msg::Range SimulatorSpatzNode::buildLaserMessage(const rclcpp::Time &stamp, float measurement) {
    sensor_msgs::msg::Range laser_msg;
    laser_msg.header.stamp = stamp;
    laser_msg.header.frame_id = "laser_front";
    laser_msg.field_of_view = 1 * (2 * M_PI) / 360;
    // Closer to infrared than ultrasound... Laser type does not exist in standard message
    laser_msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
    // Leuze electronic ODSL B/C66-500-S12 sensor
    laser_msg.min_range = 0.02;
    laser_msg.max_range = 5;
    laser_msg.range = measurement;
    return laser_msg;
}

env::Spatz SimulatorSpatzNode::spatzFromHWIn(const HardwareIn &inobj) {
    env::SensorSide sensorSide{false, inobj.binaryLightSensorTriggered};
    env::Spatz spatz{inobj.time,
                     math::v3d{inobj.x, inobj.y, inobj.psi},
                     math::v2d{inobj.velX, inobj.velY},
                     math::v3d{inobj.accX, inobj.accY, 0},
                     inobj.dPsi,
                     inobj.steeringAngle,
                     inobj.laserSensorValue,
                     sensorSide,
                     inobj.drivenDistance};

    return spatz;
}

auto SimulatorSpatzNode::buildSpatzTransform(const rclcpp::Time &stamp, const env::Spatz &spatz)
        -> geometry_msgs::msg::TransformStamped {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.frame_id = "map";
    tf_msg.header.stamp = stamp;
    tf_msg.child_frame_id = "spatz";
    tf2::Quaternion spatzRot;
    spatzRot.setRPY(0, 0, spatz.getPsi());
    tf2::convert(spatzRot, tf_msg.transform.rotation);
    tf_msg.transform.translation.x = spatz.getPos().x;
    tf_msg.transform.translation.y = spatz.getPos().y;
    tf_msg.transform.translation.z = 0;
    return tf_msg;
}
