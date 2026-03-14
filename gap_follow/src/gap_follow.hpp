#ifndef GAP_FOLLOW_HPP
#define GAP_FOLLOW_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "arcus_msgs/msg/error_code.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <string>
#include <vector>
#include <algorithm>
#include <deque>

class ReactiveGapFollow : public rclcpp::Node
{
    static constexpr float OVERSHOOT_FACTOR = 0.56f;
    static constexpr uint16_t ELIMINATE_EXTREMES_POSITION = 4U;

    static constexpr float BUBBLE_RADIUS = 0.25f;
    static constexpr float SPEED_DISTANCE_FACTOR = 0.8f;
    static constexpr float MAX_SPEED = 20.0f;
    static constexpr float DISPARITY_THRESHOLD = 0.1f;
    static constexpr float SAFE_TURNING_DISTANCE = 0.1f;

    static constexpr const char* LIDAR_SCAN_TOPIC = "/scan";
    static constexpr const char* DRIVE_TOPIC = "/drive";

    static constexpr uint16_t DEFAULT_QOS = 1U;

    static constexpr uint16_t ROLLING_AVERAGE_WINDOW = 3U;

  public:
    ReactiveGapFollow();

  private:
    void preprocessLidar(std::vector<float>& ranges_);
    void lidar_CB(sensor_msgs::msg::LaserScan::SharedPtr scanMsg_);
    void heartbeat();

    bool _straight = false;
    float setSpeedFromDistance(float distance_, float steeringAngle_);
    float computeRollingAverage(float newValue_);

    float _targetAngle = 0.0f;

    uint32_t _targetIndex = 0;
    uint32_t _maxGapStartingIndex = 0;
    uint32_t _maxGapEndingIndex = 0;

    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<arcus_msgs::msg::ErrorCode>::SharedPtr _error_publisher;
    
    std::deque<float> _targetAngleWindow;
    std::vector<float> _processedRanges;
    float _smoothedTargetAngle = 0.0f;
    float _rollingSum = 0.0f;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr _directionPublisher;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr _laserPublisher;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _laserScanSubscriber;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr _targetWaypointPublisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _vectorPublisher;
};

#endif  // GAP_FOLLOW_HPP