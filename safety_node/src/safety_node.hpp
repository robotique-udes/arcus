#ifndef SAFETY_NODE_HPP
#define SAFETY_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "arcus_msgs/msg/error_code.hpp"

class Safety : public rclcpp::Node
{
    static constexpr double TTC_THRESHOLD_S = 1.0;
    static constexpr double MIN_RANGE_RATE_MS = 0.0;  // Minimum range rate in m/s

    static constexpr const uint8_t QOS = 1;
    static constexpr const char* DRIVE_CMD_TOPIC = "/drive";
    static constexpr const char* LIDAR_SCAN_TOPIC = "/scan";
    static constexpr const char* POSITION_TOPIC = "/odometry/filtered";

  public:
    Safety();

  private:
    void CB_positionSubscriber(const nav_msgs::msg::Odometry& msg_);
    void CB_scan(const sensor_msgs::msg::LaserScan& scanMsg_);
    void CB_spam_stop(void);

    void initRosElements(void);
    void publishBrakeMessage(void);

    double _currentSpeed = 0.0;
    bool _stopFlag = false;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr _driveCmdPublisher;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _laserScanSubscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _positionSubscriber;
    rclcpp::TimerBase::SharedPtr _spammingTimer;

    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<arcus_msgs::msg::ErrorCode>::SharedPtr _error_publisher;
};
#endif  // SAFETY_NODE_HPP