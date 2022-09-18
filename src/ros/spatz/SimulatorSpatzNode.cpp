/**
 * @file SimulatorSpatzNode.cpp
 * @author ottojo
 * @date 4/1/21
 */

#include "SimulatorSpatzNode.hpp"
#include <std_msgs/msg/header.hpp>

SimulatorSpatzNode::SimulatorSpatzNode() : Node("simulator_input"), rx(SimulatorSHM::CLIENT, SHM_HWIN_ID) {
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

    clockPublisher = create_publisher<rosgraph_msgs::msg::Clock>("/clock", QoS(1).reliable().transient_local());

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
            assert(success); // This should only fail if publisher was configured incorrectly...
        }
        return;
    }

    RCLCPP_INFO(get_logger(), "Received Spatz from simulator");

    // Build and publish spatz
    auto spatz = spatzFromHWIn(*inobj);
    rclcpp::Time spatzTime = spatz.header.stamp;

    if (spatzTime == lastSpatzTime) {
        rx.unlock(inobj);
        return;
    }
    lastSpatzTime = spatzTime;

    RCLCPP_INFO(get_logger(), "publishing ros time %fs", spatzTime.seconds());
    clockPublisher->publish(rosgraph_msgs::build<rosgraph_msgs::msg::Clock>().clock(spatzTime));

    RCLCPP_INFO(get_logger(), "publishing spatz at x=%f, y=%f, psi=%f", spatz.pose.x, spatz.pose.y,
                spatz.pose.z);
    spatzPublisher->publish(spatz);

    // Publish rcmode if changed
    if (prevPaused != inobj->paused) {
        auto rcModeMessage = spatz_interfaces::msg::RCMode();
        rcModeMessage.enabled = inobj->paused;
        rcModePublisher->publish(rcModeMessage);
    }
    prevPaused = inobj->paused;

    rx.unlock(inobj);
}

SimulatorSpatzNode::~SimulatorSpatzNode() {
    rx.detach();
}

void SimulatorSpatzNode::onSpatzLivelinessLost(const rclcpp::QOSLivelinessLostInfo & /* info */) const {
    // Spatz publisher liveliness lost. Nothing we can do really.
    RCLCPP_WARN(get_logger(), "Spatz publisher lost expected liveliness. Is simulator running?");
}

spatz_interfaces::msg::Spatz SimulatorSpatzNode::spatzFromHWIn(const HardwareIn &inobj) {

    spatz_interfaces::msg::Spatz msg{};

    msg.header = std_msgs::build<std_msgs::msg::Header>().stamp(
            rclcpp::Time(inobj.time * std::nano::den, RCL_ROS_TIME)).frame_id("map");
    msg.pose.x = inobj.x;
    msg.pose.y = inobj.y;
    msg.pose.z = inobj.psi;
    msg.velocity.x = inobj.velX;
    msg.velocity.y = inobj.velY;
    msg.acceleration.x = inobj.accX;
    msg.acceleration.y = inobj.accY;
    msg.d_psi = inobj.dPsi;
    msg.steer_angle_front = inobj.steeringAngleFront;
    msg.steer_angle_rear = inobj.steeringAngleRear;
    msg.laser_front = inobj.laserSensorValue;
    msg.light_switch_rear = inobj.binaryLightSensorTriggered;
    msg.integrated_distance = inobj.drivenDistance;

    // Note: The system params are not set. For Team-Spatzenhirn internal code, these come from the SpatzLib,
    // where the system parameters of our vehicle are located.
    // It would probably be a good design to allow selection of the vehicle parameters/model in the simulator,
    // and for the simulator to forward this to the SimulatorSpatzNode.

    return msg;
}
