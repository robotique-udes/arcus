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
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&MasterNode::watchdog, this));
    _mainLoopTimer = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&MasterNode::mainLoop, this));

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

void MasterNode::watchdog()
{
    for (int i = 0; i < 10; i++)
    {
        if (rclcpp::Clock().now().nanoseconds() - this->heartbeats[i] > 100000000)
        {  // 0.1 second timeout
            RCLCPP_WARN(this->get_logger(), "No heartbeat from node %d", i);
            arcus_msgs::msg::ErrorCode error_msg;
            error_msg.source = i;
            error_msg.header.stamp = rclcpp::Clock().now();
            error_msg.error_code = arcus_msgs::msg::ErrorCode::OFFLINE;
            this->error_publisher_->publish(error_msg);
        }
    }
}

void MasterNode::errorCodeCallback(const arcus_msgs::msg::ErrorCode::SharedPtr msg)
{
    if (msg->source >= 10)
    {
        RCLCPP_ERROR(this->get_logger(), "Received error code from invalid source: %d", msg->source);
        return;
    }
    this->heartbeats[msg->source] = msg->header.stamp.nanosec;
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
    } else if (msg->source == arcus_msgs::msg::ErrorCode::SAFETY && msg->error_code == arcus_msgs::msg::ErrorCode::OK)
    {
        emergencyBrakeEngaged = false;
    }

}

void MasterNode::disparityDriveCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
    driveCommands[arcus_msg::msg::ErrorCode::DISPARITY] = *msg;
}

void MasterNode::safetyDriveCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
    driveCommands[arcus_msg::msg::ErrorCode::SAFETY] = *msg;
}

void MasterNode::purePursuitDriveCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
    driveCommands[arcus_msg::msg::ErrorCode::PURE_PURSUIT] = *msg;
}

void MasterNode::controllerDriveCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
    driveCommands[arcus_msg::msg::ErrorCode::CONTROLLER] = *msg;
}

void MasterNode::processDriveCommands()
{

    // If in emergency brake, only publish the safety drive command (which should be a brake command)
    if (emergencyBrakeEngaged)
    {
        _drivePublisher->publish(this->driveCommands[arcus_msgs::msg::ErrorCode::SAFETY]);
    } else {
        // otherwise, a simple priority system: if the safety node has a command, use it. Otherwise, use the controller command.
        if (this->driveCommands[arcus_msgs::msg::ErrorCode::SAFETY].drive.speed != 0 || this->driveCommands[arcus_msgs::msg::ErrorCode::SAFETY].drive.steering_angle != 0)
        {
            _drivePublisher->publish(this->driveCommands[arcus_msgs::msg::ErrorCode::SAFETY]);
        }
        else if (this->driveCommands[arcus_msgs::msg::ErrorCode::CONTROLLER].drive.speed != 0 || this->driveCommands[arcus_msgs::msg::ErrorCode::CONTROLLER].drive.steering_angle != 0)
        {
            _drivePublisher->publish(this->driveCommands[arcus_msgs::msg::ErrorCode::CONTROLLER]);
        } // if no safety or controller command, use the pure pursuit command if it exists
        else if (this->driveCommands[arcus_msgs::msg::ErrorCode::PURE_PURSUIT].drive.speed != 0 || this->driveCommands[arcus_msgs::msg::ErrorCode::PURE_PURSUIT].drive.steering_angle != 0)
        {
            _drivePublisher->publish(this->driveCommands[arcus_msgs::msg::ErrorCode::PURE_PURSUIT]);
        } // if no safety, controller, or pure pursuit command, use the disparity command if it exists
        else if (this->driveCommands[arcus_msgs::msg::ErrorCode::DISPARITY].drive.speed != 0 || this->driveCommands[arcus_msgs::msg::ErrorCode::DISPARITY].drive.steering_angle != 0)
        {
            _drivePublisher->publish(this->driveCommands[arcus_msgs::msg::ErrorCode::DISPARITY]);
        } else {
            // if no commands, publish a zero command to stop the car
            ackermann_msgs::msg::AckermannDriveStamped stopCmd;
            stopCmd.drive.speed = 0.0;
            stopCmd.drive.steering_angle = 0.0;
            _drivePublisher->publish(stopCmd);
        }
    }
}


void MasterNode::mainLoop()
{
    this->processDriveCommands();
}
#endif  // MASTER_NODE_CPP