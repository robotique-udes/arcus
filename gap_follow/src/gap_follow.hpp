
#ifndef GAP_FOLLOW_HPP
#define GAP_FOLLOW_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "arcus_msgs/msg/error_code.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float32.hpp"

#include <string>
#include <vector>
#include <algorithm>
#include <deque>

class ReactiveGapFollow : public rclcpp::Node
{
  public:
    ReactiveGapFollow();

  private:
    void preprocess_lidar(std::vector<float>& ranges_, float range_max, float range_min, float angle_min, float angle_inc);
    void get_differences(std::vector<float> &ranges, std::vector<float> &differences);
    void get_disparities(std::vector<float> &differences, float threshold, std::vector<int> &disparities);
    int get_num_points(double width, double distance, double angle_inc);
    void cover_points(std::vector<float> &ranges, int cover_direction, int num_points, int start_index);
    void extend_disparities(std::vector<float> &ranges, std::vector<int> &disparities, double car_width, int extra_points, double angle_inc);
    void lidar_CB(sensor_msgs::msg::LaserScan::SharedPtr scanMsg_);
    void heartbeat();

    bool _straight = false;
    float setSpeedFromDistance(float distance_, float steeringAngle_);
    float setRiskFromSpeed(float speed_);
    float computeRollingAverage(float newValue_);

    float _targetAngle = 0.0f;
    float _wheelBase = 0.324;
    float _frictionCoeff = 0.75;
    float _bubbleRadius = 0.25f;
    float _speedDistanceFactor = 0.8f;
    float _riskSpeedFactor = 0.8f;
    float _maxSpeed = 20.0f;
    float _disparityThreshold = 0.1f;
    uint16_t _defaultQos = 1U;
    uint16_t _rollingAverageWindow = 3U;
    bool _debug = false;
    std::string _lidarScanTopic = "/scan";
    std::string _driveTopic = "/disparity/drive";
    std::string _riskTopic = "/disparity/risk";

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
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _riskPublisher;
};

#endif  // GAP_FOLLOW_HPP
