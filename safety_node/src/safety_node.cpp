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
    _ttcThresholdS = this->declare_parameter<double>("ttc_threshold_s", 0.7);
    _minRangeRateMs = this->declare_parameter<double>("min_range_rate_ms", 0.0);
    _qos = static_cast<uint8_t>(this->declare_parameter<int>("qos", 1));
    _driveCmdTopic = this->declare_parameter<std::string>("drive_cmd_topic", "/safety/drive");
    _brakeCmdTopic = this->declare_parameter<std::string>("brake_cmd_topic", "/commands/motor/brake");
    _lidarScanTopic = this->declare_parameter<std::string>("lidar_scan_topic", "/scan");
    _positionTopic = this->declare_parameter<std::string>("position_topic", "/odometry/filtered");
    _fovRad = this->declare_parameter<double>("fov_rad", 2.0 / 180.0 * M_PI);
    _brakeForce = this->declare_parameter<double>("brake_force", 10.0);
    _spam_stop_period_ms = this->declare_parameter<int>("spam_stop_period_ms", 5);
    _heartbeat_period_ms = this->declare_parameter<int>("heartbeat_period_ms", 50);

    if (_spam_stop_period_ms <= 0)
    {
        RCLCPP_WARN(this->get_logger(), "spam_stop_period_ms must be > 0, forcing to 5");
        _spam_stop_period_ms = 5;
    }

    if (_heartbeat_period_ms <= 0)
    {
        RCLCPP_WARN(this->get_logger(), "heartbeat_period_ms must be > 0, forcing to 50");
        _heartbeat_period_ms = 50;
    }

    this->initRosElements();
}

void Safety::CB_positionSubscriber(const nav_msgs::msg::Odometry& msg_)
{
    _currentSpeed = msg_.twist.twist.linear.x;
}

void Safety::CB_scan(const sensor_msgs::msg::LaserScan& scanMsg_)
{
    std::vector<float> ranges = scanMsg_.ranges;

    size_t fov_start = (-_fovRad - scanMsg_.angle_min) / scanMsg_.angle_increment;
    size_t fov_end = (_fovRad - scanMsg_.angle_min) / scanMsg_.angle_increment;

    for (size_t i = fov_start; i < fov_end; i++)
    {
        if (std::isnan(ranges[i]) || std::isinf(ranges[i]))
        {
            continue;
        }

        double rangeRate = _currentSpeed * std::cos(scanMsg_.angle_min + i * scanMsg_.angle_increment);
        double iTTC = 100;

        if (rangeRate > _minRangeRateMs)
        {
            iTTC = ranges[i] / rangeRate;
        }


        if (iTTC < _ttcThresholdS)
        {
            _stopFlag = true;
            this->publishBrakeMessage();
            break;
        }

        if (i == (fov_end - 1))
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
        std_msgs::msg::Float64 brake;
        brake.data = _brakeForce;

        _brakePublisher->publish(brake);
        arcus_msgs::msg::ErrorCode error_msg;
        error_msg.source = arcus_msgs::msg::ErrorCode::SAFETY;
        error_msg.header.stamp = this->now();
        error_msg.error_code = arcus_msgs::msg::ErrorCode::EMERGENCY_BRAKE;
        this->_error_publisher->publish(error_msg);
    }
}

void Safety::initRosElements(void)
{
    _driveCmdPublisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(_driveCmdTopic, _qos);

    _brakePublisher = this->create_publisher<std_msgs::msg::Float64>(_brakeCmdTopic, _qos);

    _laserScanSubscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(_lidarScanTopic,
                                                                                  _qos,
                                                                                  [this](const sensor_msgs::msg::LaserScan& msg)
                                                                                  {
                                                                                      this->CB_scan(msg);
                                                                                  });

    _positionSubscriber = this->create_subscription<nav_msgs::msg::Odometry>(_positionTopic,
                                                                             _qos,
                                                                             [this](const nav_msgs::msg::Odometry& msg)
                                                                             {
                                                                                 this->CB_positionSubscriber(msg);
                                                                             });
    _spammingTimer = this->create_wall_timer(std::chrono::milliseconds(_spam_stop_period_ms),
                                             std::bind(&Safety::CB_spam_stop, this));

    _error_publisher = this->create_publisher<arcus_msgs::msg::ErrorCode>("/node_error_code", 10);
    _timer = this->create_wall_timer(std::chrono::milliseconds(_heartbeat_period_ms), std::bind(&Safety::heartbeat, this));
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