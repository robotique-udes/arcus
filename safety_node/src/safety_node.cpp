#include "safety_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}

Safety::Safety():
    Node("safety_node")
{
    this->initRosElements();
}

void Safety::CB_positionSubscriber(const nav_msgs::msg::Odometry& msg_)
{
    _currentSpeed = msg_.twist.twist.linear.x;
}

void Safety::CB_scan(const sensor_msgs::msg::LaserScan& scanMsg_)
{
    std::vector<float> ranges = scanMsg_.ranges;

    for (size_t i = 0; i < ranges.size(); i++)
    {
        if (std::isnan(ranges[i]) || std::isinf(ranges[i]))
        {
            continue;
        }

        double rangeRate = _currentSpeed * std::cos(scanMsg_.angle_min + i * scanMsg_.angle_increment);

        if (rangeRate <= MIN_RANGE_RATE_MS)
        {
            continue;
        }

        double iTTC = ranges[i] / rangeRate;

        if (iTTC < TTC_THRESHOLD_S)
        {
            this->publishBrakeMessage();
            break;
        }
    }
}

void Safety::initRosElements(void)
{
    _driveCmdPublisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(DRIVE_CMD_TOPIC, QOS);

    _laserScanSubscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(LIDAR_SCAN_TOPIC,
                                                                                  QOS,
                                                                                  [this](const sensor_msgs::msg::LaserScan& msg)
                                                                                  {
                                                                                      this->CB_scan(msg);
                                                                                  });

    _positionSubscriber = this->create_subscription<nav_msgs::msg::Odometry>(POSITION_TOPIC,
                                                                             QOS,
                                                                             [this](const nav_msgs::msg::Odometry& msg)
                                                                             {
                                                                                 this->CB_positionSubscriber(msg);
                                                                             });
}

void Safety::publishBrakeMessage(void)
{
    ackermann_msgs::msg::AckermannDriveStamped driveCmd;
    driveCmd.drive.speed = 0.0;
    _driveCmdPublisher->publish(driveCmd);

    RCLCPP_INFO(this->get_logger(), "Emergency brake");
}