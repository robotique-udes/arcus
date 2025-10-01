#ifndef GAP_FOLLOW_HPP
#define GAP_FOLLOW_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#include <string>
#include <vector>
#include <algorithm>

class ReactiveGapFollow : public rclcpp::Node
{
    static constexpr uint16_t OVERSHOOT_FACTOR = 1U;
    static constexpr uint16_t ELIMINATE_EXTREMES_POSITION = 4U;

    static constexpr float PREPROCESSING_AVG_SAMPLES = 7.f;
    static constexpr float MAX_LIDAR_DISTANCE_M = 10.f;
    static constexpr float BUBBLE_RADIUS = 1.f;
    static constexpr float SPEED_DISTANCE_FACTOR = 1.f;

    static constexpr const char* LIDAR_SCAN_TOPIC = "/scan";
    static constexpr const char* DRIVE_TOPIC = "/drive";

    static constexpr uint16_t DEFAULT_QOS = 1U;

  public:
    ReactiveGapFollow();

  private:
    void preprocessLidar(std::vector<float>& ranges_);
    void lidar_CB(sensor_msgs::msg::LaserScan::SharedPtr scanMsg_);
    float setSpeedFromDistance(float distance_);

    bool _straight = false;

    float _targetAngle = 0.0f;

    uint32_t _targetIndex = 0;
    uint32_t _maxGapStartingIndex = 0;
    uint32_t _maxGapEndingIndex = 0;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr _directionPublisher;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _laserScanSubscriber;
};

#endif  // GAP_FOLLOW_HPP