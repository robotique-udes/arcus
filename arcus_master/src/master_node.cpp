#ifndef MASTER_NODE_CPP
#define MASTER_NODE_CPP

#include "master_node.hpp"
#include <sstream>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MasterNode>());
    rclcpp::shutdown();
    return 0;
}

MasterNode::MasterNode():
    Node("master_node")
{
    _mainLoopTimer = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&MasterNode::mainLoop, this));

    error_listener_ = this->create_subscription<arcus_msgs::msg::ErrorCode>(
        "/node_error_code",
        10,
        std::bind(&MasterNode::errorCodeCallback, this, std::placeholders::_1));

    error_publisher_ = this->create_publisher<arcus_msgs::msg::ErrorCode>("/master_error_code", 10);

    _drivePublisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(DRIVE_TOPIC, 10);

    _disparityDriveSubscriber = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        DISPARITY_DRIVE_TOPIC,
        10,
        std::bind(&MasterNode::disparityDriveCallback, this, std::placeholders::_1));

    _safetyDriveSubscriber = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        SAFETY_DRIVE_TOPIC,
        10,
        std::bind(&MasterNode::safetyDriveCallback, this, std::placeholders::_1));

    _purePursuitDriveSubscriber = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        PURE_PURSUIT_DRIVE_TOPIC,
        10,
        std::bind(&MasterNode::purePursuitDriveCallback, this, std::placeholders::_1));

    _controllerDriveSubscriber = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        CONTROLLER_DRIVE_TOPIC,
        10,
        std::bind(&MasterNode::controllerDriveCallback, this, std::placeholders::_1));

    _deadmanSubscriber
        = this->create_subscription<std_msgs::msg::Bool>(DEADMAN_TOPIC,
                                                         10,
                                                         std::bind(&MasterNode::deadmanCallback, this, std::placeholders::_1));
}

// void MasterNode::watchdog()
// {
//     for (int i = 0; i < 10; i++)
//     {
//         if (rclcpp::Clock().now().nanoseconds() - this->_lastHeartbeatNs[i] > 100000000)
//         {  // 0.1 second timeout
//             RCLCPP_WARN(this->get_logger(), "No heartbeat from node %d", i);
//             arcus_msgs::msg::ErrorCode error_msg;
//             error_msg.source = i;
//             error_msg.header.stamp = rclcpp::Clock().now();
//             error_msg.error_code = arcus_msgs::msg::ErrorCode::OFFLINE;
//             this->error_publisher_->publish(error_msg);
//         }
//     }
// }

void MasterNode::errorCodeCallback(const arcus_msgs::msg::ErrorCode::SharedPtr msg)
{
    if (msg->source >= 10)
    {
        RCLCPP_ERROR(this->get_logger(), "Received error code from invalid source: %d", msg->source);
        return;
    }

    // Use local receive time so online tracking is immune to clock skew between nodes.
    this->_lastHeartbeatNs[msg->source] = static_cast<uint64_t>(this->now().nanoseconds());
    this->_nodeOnline[msg->source] = msg->error_code != arcus_msgs::msg::ErrorCode::OFFLINE;

    _errorCodeLatch[msg->source] = msg->error_code;

    if (msg->source == arcus_msgs::msg::ErrorCode::SAFETY && msg->error_code == arcus_msgs::msg::ErrorCode::EMERGENCY_BRAKE)
    {
        emergencyBrakeEngaged = true;
    }
    else if (msg->source == arcus_msgs::msg::ErrorCode::SAFETY && msg->error_code == arcus_msgs::msg::ErrorCode::OK)
    {
        emergencyBrakeEngaged = false;
    }

    if (msg->source == arcus_msgs::msg::ErrorCode::PURE_PURSUIT && msg->error_code == arcus_msgs::msg::ErrorCode::EMERGENCY_BRAKE)
    {
        ppRecoveryEngaged = true;
    }
    else if (msg->source == arcus_msgs::msg::ErrorCode::PURE_PURSUIT && msg->error_code == arcus_msgs::msg::ErrorCode::OK)
    {
        ppRecoveryEngaged = false;
    }

    this->tryPublishDriveCommand();
}

void MasterNode::disparityDriveCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
    driveCommands[arcus_msgs::msg::ErrorCode::DISPARITY] = *msg;
    this->tryPublishDriveCommand();
}

void MasterNode::safetyDriveCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
    driveCommands[arcus_msgs::msg::ErrorCode::SAFETY] = *msg;
    this->tryPublishDriveCommand();
}

void MasterNode::purePursuitDriveCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
    driveCommands[arcus_msgs::msg::ErrorCode::PURE_PURSUIT] = *msg;
    this->tryPublishDriveCommand();
}

void MasterNode::controllerDriveCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
    driveCommands[arcus_msgs::msg::ErrorCode::CONTROLLER] = *msg;
    this->tryPublishDriveCommand();
}

void MasterNode::deadmanCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    _deadmanActive = msg->data;
}

void MasterNode::refreshOnlineStatus()
{
    uint64_t now_ns = this->now().nanoseconds();
    constexpr uint64_t timeout_ns = 300000000;  // 0.3 second timeout for scheduler/network jitter

    for (std::size_t i = 0; i < _nodeOnline.size(); ++i)
    {
        if ((now_ns - _lastHeartbeatNs[i]) > timeout_ns)
        {
            _nodeOnline[i] = false;
        }
    }
}

bool MasterNode::hasCommand(const ackermann_msgs::msg::AckermannDriveStamped& cmd) const
{
    return cmd.drive.speed != 0.0 || cmd.drive.steering_angle != 0.0;
}

MasterNode::DriveState MasterNode::determineDriveState() const
{
    if (!_deadmanActive)
    {
        return DriveState::SAFETY_EMERGENCY;
    }

    if (_nodeOnline[arcus_msgs::msg::ErrorCode::CONTROLLER] && hasCommand(driveCommands[arcus_msgs::msg::ErrorCode::CONTROLLER]))
    {
        return DriveState::CONTROLLER;
    }

    if (ppRecoveryEngaged && _nodeOnline[arcus_msgs::msg::ErrorCode::PURE_PURSUIT]
        && hasCommand(driveCommands[arcus_msgs::msg::ErrorCode::PURE_PURSUIT]))
    {
        return DriveState::PURE_PURSUIT;
    }

    if (emergencyBrakeEngaged && _nodeOnline[arcus_msgs::msg::ErrorCode::SAFETY])
    {
        return DriveState::SAFETY_EMERGENCY;
    }

    if (_nodeOnline[arcus_msgs::msg::ErrorCode::SAFETY] && hasCommand(driveCommands[arcus_msgs::msg::ErrorCode::SAFETY]))
    {
        return DriveState::SAFETY_OVERRIDE;
    }

    if (_nodeOnline[arcus_msgs::msg::ErrorCode::PURE_PURSUIT]
        && hasCommand(driveCommands[arcus_msgs::msg::ErrorCode::PURE_PURSUIT]))
    {
        return DriveState::PURE_PURSUIT;
    }

    if (_nodeOnline[arcus_msgs::msg::ErrorCode::DISPARITY] && hasCommand(driveCommands[arcus_msgs::msg::ErrorCode::DISPARITY]))
    {
        return DriveState::DISPARITY;
    }

    return DriveState::NONE;
}

void MasterNode::tryPublishDriveCommand()
{
    DriveState state = this->determineDriveState();

    switch (state)
    {
        case DriveState::SAFETY_EMERGENCY:
        {
            ackermann_msgs::msg::AckermannDriveStamped emergency_cmd = this->driveCommands[arcus_msgs::msg::ErrorCode::SAFETY];
            if (_hasLastNonEmergencySteering)
            {
                emergency_cmd.drive.steering_angle = this->driveCommands[arcus_msgs::msg::ErrorCode::PURE_PURSUIT].drive.steering_angle;
            }
            _drivePublisher->publish(emergency_cmd);
            break;
        }
        case DriveState::SAFETY_OVERRIDE:
            _drivePublisher->publish(this->driveCommands[arcus_msgs::msg::ErrorCode::SAFETY]);
            _lastNonEmergencySteering = this->driveCommands[arcus_msgs::msg::ErrorCode::SAFETY].drive.steering_angle;
            _hasLastNonEmergencySteering = true;
            break;
        case DriveState::CONTROLLER:
            _drivePublisher->publish(this->driveCommands[arcus_msgs::msg::ErrorCode::CONTROLLER]);
            _lastNonEmergencySteering = this->driveCommands[arcus_msgs::msg::ErrorCode::CONTROLLER].drive.steering_angle;
            _hasLastNonEmergencySteering = true;
            break;
        case DriveState::PURE_PURSUIT:
            _drivePublisher->publish(this->driveCommands[arcus_msgs::msg::ErrorCode::PURE_PURSUIT]);
            _lastNonEmergencySteering = this->driveCommands[arcus_msgs::msg::ErrorCode::PURE_PURSUIT].drive.steering_angle;
            _hasLastNonEmergencySteering = true;
            break;
        case DriveState::DISPARITY:
            _drivePublisher->publish(this->driveCommands[arcus_msgs::msg::ErrorCode::DISPARITY]);
            _lastNonEmergencySteering = this->driveCommands[arcus_msgs::msg::ErrorCode::DISPARITY].drive.steering_angle;
            _hasLastNonEmergencySteering = true;
            break;
        case DriveState::NONE:
            _emptyMsg.drive.speed = 0.0;
            _emptyMsg.drive.steering_angle = 0.0;
            _drivePublisher->publish(_emptyMsg);
            break;
    }
}

void MasterNode::mainLoop()
{
    this->refreshOnlineStatus();

    // Heartbeat + error publishing handled here
    bool noErrors = true;

        for (size_t i = 0; i < _errorCodeLatch.size(); ++i)
        {
            if (_errorCodeLatch[i] != arcus_msgs::msg::ErrorCode::OK)
            {
                noErrors = false;

                arcus_msgs::msg::ErrorCode error_msg;
                error_msg.source = i;
                error_msg.header.stamp = this->now();
                error_msg.error_code = _errorCodeLatch[i];

                this->error_publisher_->publish(error_msg);
            }
        }

        if (noErrors)
        {
            arcus_msgs::msg::ErrorCode error_msg;
            error_msg.source = arcus_msgs::msg::ErrorCode::MASTER;
            error_msg.header.stamp = this->now();
            error_msg.error_code = arcus_msgs::msg::ErrorCode::OK;

            this->error_publisher_->publish(error_msg);
        }
}
#endif  // MASTER_NODE_CPP
