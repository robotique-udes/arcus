#ifndef SAFETY_NODE_HPP
#define SAFETY_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class Safety : public rclcpp::Node
{
    static constexpr double TTC_THRESHOLD = 1.0;

    static constexpr const uint8_t DEFAULT_QOS = 1;
    static constexpr const char* DEFAULT_DRIVE_CMD_TOPIC = "/drive";
    static constexpr const char* DEFAULT_LIDAR_SCAN_TOPIC = "/scan";
    static constexpr const char* DEFAULT_POSITION_TOPIC = "/ego_racecar/odom";

  public:
    Safety();

  private:
    void CB_positionSubscriber(const nav_msgs::msg::Odometry& msg_);
    void CB_scan(const sensor_msgs::msg::LaserScan& scanMsg_);

    void initRosElements(void);
    void publishBrakeMessage(void);

    std::string _driveCmdTopic = DEFAULT_DRIVE_CMD_TOPIC;
    std::string _lidarScanTopic = DEFAULT_LIDAR_SCAN_TOPIC;
    std::string _positionTopic = DEFAULT_POSITION_TOPIC;

    double _currentSpeed = 0.0;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr _driveCmdPublisher;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _laserScanSubscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _positionSubscriber;
};

#endif  // SAFETY_NODE_HPP