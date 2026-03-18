#ifndef MASTER_NODE_CPP
#define MASTER_NODE_CPP

#include "master_node.hpp"

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

    this->_lastHeartbeatNs[msg->source] = (uint64_t)msg->header.stamp.sec * 1000000000ULL + msg->header.stamp.nanosec;
    this->_nodeOnline[msg->source] = msg->error_code != arcus_msgs::msg::ErrorCode::OFFLINE;

    if (msg->error_code != arcus_msgs::msg::ErrorCode::OK)
    {
        arcus_msgs::msg::ErrorCode error_msg;
        error_msg.source = msg->source;
        error_msg.header = msg->header;
        error_msg.error_code = msg->error_code;
        this->error_publisher_->publish(error_msg);
        RCLCPP_ERROR(this->get_logger(), "Received error code %d from node %d", msg->error_code, msg->source);
    }

    if (msg->source == arcus_msgs::msg::ErrorCode::SAFETY && msg->error_code == arcus_msgs::msg::ErrorCode::EMERGENCY_BRAKE)
    {
        emergencyBrakeEngaged = true;
    }
    else if (msg->source == arcus_msgs::msg::ErrorCode::SAFETY && msg->error_code == arcus_msgs::msg::ErrorCode::OK)
    {
        emergencyBrakeEngaged = false;
    }

    this->refreshOnlineStatus();
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

void MasterNode::refreshOnlineStatus()
{
    uint64_t now_ns = this->now().nanoseconds();
    constexpr uint64_t timeout_ns = 100000000;  // 0.1 second timeout

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
    if (emergencyBrakeEngaged && _nodeOnline[arcus_msgs::msg::ErrorCode::SAFETY])
    {
        return DriveState::SAFETY_EMERGENCY;
    }

    if (_nodeOnline[arcus_msgs::msg::ErrorCode::SAFETY]
        && hasCommand(driveCommands[arcus_msgs::msg::ErrorCode::SAFETY]))
    {
        return DriveState::SAFETY_OVERRIDE;
    }

    if (_nodeOnline[arcus_msgs::msg::ErrorCode::CONTROLLER]
        && hasCommand(driveCommands[arcus_msgs::msg::ErrorCode::CONTROLLER]))
    {
        return DriveState::CONTROLLER;
    }

    if (_nodeOnline[arcus_msgs::msg::ErrorCode::PURE_PURSUIT]
        && hasCommand(driveCommands[arcus_msgs::msg::ErrorCode::PURE_PURSUIT]))
    {
        return DriveState::PURE_PURSUIT;
    }

    if (_nodeOnline[arcus_msgs::msg::ErrorCode::DISPARITY]
        && hasCommand(driveCommands[arcus_msgs::msg::ErrorCode::DISPARITY]))
    {
        return DriveState::DISPARITY;
    }

    return DriveState::NONE;
}

void MasterNode::tryPublishDriveCommand()
{
    this->refreshOnlineStatus();

    RCLCPP_INFO(this->get_logger(), "ONLINE NODES, safety:  %d, controller: %d,", _nodeOnline[arcus_msgs::msg::ErrorCode::SAFETY], _nodeOnline[arcus_msgs::msg::ErrorCode::CONTROLLER]);
    DriveState state = this->determineDriveState();
    RCLCPP_INFO(this->get_logger(), "STATE %d", state);

    switch (state)
    {
        case DriveState::SAFETY_EMERGENCY:
        {
            ackermann_msgs::msg::AckermannDriveStamped emergency_cmd =
                this->driveCommands[arcus_msgs::msg::ErrorCode::SAFETY];
            if (_hasLastNonEmergencySteering)
            {
                emergency_cmd.drive.steering_angle = _lastNonEmergencySteering;
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
            break;
    }
}

void MasterNode::mainLoop()
{
    this->refreshOnlineStatus();
}
#endif  // MASTER_NODE_CPP