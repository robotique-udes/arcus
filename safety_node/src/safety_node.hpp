#ifndef SAFETY_NODE_HPP
#define SAFETY_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "arcus_msgs/msg/error_code.hpp"
#include "std_msgs/msg/float64.hpp"

#include <string>

class Safety : public rclcpp::Node
{
  public:
    Safety();

  private:
    void CB_positionSubscriber(const nav_msgs::msg::Odometry& msg_);
    void CB_scan(const sensor_msgs::msg::LaserScan& scanMsg_);
    void CB_spam_stop(void);

    void initRosElements(void);
    void publishBrakeMessage(void);
    void heartbeat(void);

    double _currentSpeed = 0.0;
    bool _stopFlag = false;

    double _ttcThresholdS = 0.7;
    double _minRangeRateMs = 0.0;
    uint8_t _qos = 1;
    std::string _driveCmdTopic = "/safety/drive";
    std::string _brakeCmdTopic = "/commands/motor/brake";
    std::string _lidarScanTopic = "/scan";
    std::string _positionTopic = "/odometry/filtered";
    double _fovRad = 2.0 / 180.0 * M_PI;
    double _brakeForce = 10.0;
    int _spam_stop_period_ms = 5;
    int _heartbeat_period_ms = 50;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr _driveCmdPublisher;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _brakePublisher;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _laserScanSubscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _positionSubscriber;
    rclcpp::TimerBase::SharedPtr _spammingTimer;

    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<arcus_msgs::msg::ErrorCode>::SharedPtr _error_publisher;
};
#endif  // SAFETY_NODE_HPP