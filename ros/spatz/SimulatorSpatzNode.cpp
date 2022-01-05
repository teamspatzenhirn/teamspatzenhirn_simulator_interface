/**
 * @file SimulatorSpatzNode.cpp
 * @author ottojo
 * @date 4/1/21
 */

#include "SimulatorSpatzNode.hpp"

#include <SpatzX/lib/SystemParams.hpp>
#include <Util/ros/Conversions/spatz.hpp>
#include <gsl/gsl>

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
    RCLCPP_INFO(get_logger(), "publishing spatz at x=%f, y=%f, psi=%f", spatz.getPos().x, spatz.getPos().y,
                spatz.getPsi());
    spatzPublisher->publish(conversions::messageFromSpatz(spatz, stamp));

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

void SimulatorSpatzNode::onSpatzLivelinessLost(const rclcpp::QOSLivelinessLostInfo & /* info */) const {
    // Spatz publisher liveliness lost. Nothing we can do really.
    RCLCPP_WARN(get_logger(), "Spatz publisher lost expected liveliness. Is simulator running?");
}

env::Spatz SimulatorSpatzNode::spatzFromHWIn(const HardwareIn &inobj) {
    env::Spatz spatz{/*t = */ inobj.time,
                     /*pose = */ math::Pose2d{inobj.x, inobj.y, inobj.psi},
                     /*vel = */ math::v2d{inobj.velX, inobj.velY},
                     /*acc = */ math::v3d{inobj.accX, inobj.accY, 0},
                     /*dPsi = */ inobj.dPsi,
                     /*steerAngleFront = */ inobj.steeringAngleFront,
                     /*steerAngleRear = */ inobj.steeringAngleRear,
                     /*laserFront = */ inobj.laserSensorValue,
                     /*lightSwitchRear = */ inobj.binaryLightSensorTriggered,
                     /*integratedDistance = */ inobj.drivenDistance,
                     /*systemParams = */ spatzx::systemParams};

    return spatz;
}
