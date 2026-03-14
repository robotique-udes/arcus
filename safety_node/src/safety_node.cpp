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

    size_t fov_start = (-FOV - scanMsg_.angle_min) / scanMsg_.angle_increment;
    size_t fov_end = (FOV - scanMsg_.angle_min) / scanMsg_.angle_increment;

    for (size_t i = fov_start; i < fov_end; i++)
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
            _stopFlag = true;
            this->publishBrakeMessage();
            break;
        }

        if (i == (fov_end - 1) && std::abs(_currentSpeed) < 0.1)
        {
            _stopFlag = false;
        }
    }
}

void Safety::CB_spam_stop(void)
{
    if (_stopFlag)
    {
        ackermann_msgs::msg::AckermannDriveStamped driveCmd;
        driveCmd.drive.speed = 0.0;
        _driveCmdPublisher->publish(driveCmd);

        arcus_msgs::msg::ErrorCode error_msg;
        error_msg.source = arcus_msgs::msg::ErrorCode::SAFETY;
        error_msg.header.stamp = rclcpp::Clock().now();
        error_msg.error_code = arcus_msgs::msg::ErrorCode::EMERGENCY_BRAKE;
        this->_error_publisher->publish(error_msg);
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
    _spammingTimer = this->create_wall_timer(std::chrono::milliseconds(5), std::bind(&Safety::CB_spam_stop, this));

    _error_publisher = this->create_publisher<arcus_msgs::msg::ErrorCode>("/node_error_code", 10);
    _timer = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&Safety::heartbeat, this));
}

void Safety::heartbeat()
{
    if (!_stopFlag)
    {
        arcus_msgs::msg::ErrorCode error_msg;
        error_msg.source = arcus_msgs::msg::ErrorCode::SAFETY;
        error_msg.header.stamp = rclcpp::Clock().now();
        error_msg.error_code = arcus_msgs::msg::ErrorCode::OK;
        this->_error_publisher->publish(error_msg);
    }
}

void Safety::publishBrakeMessage(void)
{
    ackermann_msgs::msg::AckermannDriveStamped driveCmd;
    driveCmd.drive.speed = 0.0;
    _driveCmdPublisher->publish(driveCmd);

    RCLCPP_INFO(this->get_logger(), "Emergency brake");
}