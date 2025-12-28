#ifndef MASTER_NODE_CPP
#define MASTER_NODE_CPP
#include "rclcpp/rclcpp.hpp"
#include "arcus_msgs/msg/error_code.hpp"
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

    error_listener_ = this->create_subscription<arcus_msgs::msg::ErrorCode>(
        "/node_error_code",
        10,
        std::bind(&MasterNode::errorCodeCallback, this, std::placeholders::_1));

    error_publisher_ = this->create_publisher<arcus_msgs::msg::ErrorCode>("/master_error_code", 10);
}

void MasterNode::watchdog()
{
    for (int i = 0; i < 7; i++)
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
    if (msg->source < 0 || msg->source >= 7)
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
    }
}

#endif  // MASTER_NODE_CPP