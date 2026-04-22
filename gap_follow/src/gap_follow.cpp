#include "gap_follow.hpp"
#include "arcus_msgs/msg/error_code.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveGapFollow>());
    rclcpp::shutdown();
    return 0;
}

ReactiveGapFollow::ReactiveGapFollow():
    Node("reactive_node")
{
    _overshootFactor = this->declare_parameter<double>("overshoot_factor", 0.56);
    _bubbleRadius = this->declare_parameter<double>("bubble_radius", 0.25);
    _speedDistanceFactor = this->declare_parameter<double>("speed_distance_factor", 0.8);
    _maxSpeed = this->declare_parameter<double>("max_speed", 20.0);
    _disparityThreshold = this->declare_parameter<double>("disparity_threshold", 0.1);
    _defaultQos = static_cast<uint16_t>(this->declare_parameter<int>("default_qos", 1));
    _rollingAverageWindow = static_cast<uint16_t>(this->declare_parameter<int>("rolling_average_window", 3));
    _debug = this->declare_parameter<bool>("debug", false);
    _lidarScanTopic = this->declare_parameter<std::string>("lidar_scan_topic", "/scan");
    _driveTopic = this->declare_parameter<std::string>("drive_topic", "/disparity/drive");

    if (_rollingAverageWindow == 0U)
    {
        RCLCPP_WARN(this->get_logger(), "rolling_average_window cannot be 0, forcing to 1");
        _rollingAverageWindow = 1U;
    }

    _directionPublisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(_driveTopic,
                                                                                               _defaultQos);

    _error_publisher = this->create_publisher<arcus_msgs::msg::ErrorCode>("/node_error_code", 10);
    _timer = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&ReactiveGapFollow::heartbeat, this));
    _laserPublisher = this->create_publisher<sensor_msgs::msg::LaserScan>("processed_scan", _defaultQos);

    _targetWaypointPublisher = this->create_publisher<geometry_msgs::msg::PointStamped>("target_waypoint", _defaultQos);
    _vectorPublisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("vector", _defaultQos);

    _laserScanSubscriber
        = this->create_subscription<sensor_msgs::msg::LaserScan>(_lidarScanTopic,
                                                                 _defaultQos,
                                                                 [this](const sensor_msgs::msg::LaserScan::SharedPtr msg)
                                                                 {
                                                                     this->lidar_CB(msg);
                                                                 });
}

void ReactiveGapFollow::heartbeat()
{
    arcus_msgs::msg::ErrorCode error_msg;
    error_msg.source = arcus_msgs::msg::ErrorCode::DISPARITY;
    error_msg.header.stamp = this->now();
    error_msg.error_code = arcus_msgs::msg::ErrorCode::OK;
    this->_error_publisher->publish(error_msg);
}

void ReactiveGapFollow::lidar_CB(sensor_msgs::msg::LaserScan::SharedPtr scanMsg_)
{
    if (!scanMsg_ || scanMsg_->ranges.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Invalid scan");
        return;
    }

    if (scanMsg_->angle_increment == 0.0f)
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid angle_increment");
        return;
    }
    std::vector<float>& ranges = scanMsg_->ranges;
    std::vector<float> extendedRanges;
    size_t size = ranges.size();
    extendedRanges.resize(size);
    const float angle_min = scanMsg_->angle_min;
    const float angle_inc = scanMsg_->angle_increment;
    const float range_max = scanMsg_->range_max;
    const float range_min = scanMsg_->range_min;
    float old_distance = ranges[0];  // Initialize to an invalid distance greater than range_max

    const float inv_angle_inc = 1.0f / angle_inc;

    if (_processedRanges.size() != size)
    {
        _processedRanges.resize(size, range_max + 1.0f);  // Initialize with invalid readings
    }

    // uint32_t pos90deg_index = (M_PI/2 - scanMsg_->angle_min) / scanMsg_->angle_increment;
    // uint32_t neg90deg_index = (-M_PI/2 - scanMsg_->angle_min) / scanMsg_->angle_increment;

    for (size_t i = 0; i < size; i++)
    {
        if (ranges[i] > range_max || std::isinf(ranges[i]) || std::isnan(ranges[i]))
        {
            ranges[i] = range_max + 1.0f;  // Set to a value greater than range_max to indicate invalid reading
        }
        else if (ranges[i] < range_min)
        {
            ranges[i] = range_min;
        }
        _processedRanges[i] = ranges[i];

        if (std::abs(ranges[i] - old_distance) > _disparityThreshold)
        {
            float closer_distance = std::min(ranges[i], old_distance);
            uint32_t bubble_distance = static_cast<uint32_t>(std::atan2(_bubbleRadius, closer_distance) * inv_angle_inc);
            for (int32_t j = -static_cast<int32_t>(bubble_distance); j <= static_cast<int32_t>(bubble_distance); j++)
            {
                int32_t index;
                index = static_cast<int32_t>(i) + j;

                if (index >= 0 && index < static_cast<int32_t>(size))
                {
                    _processedRanges[index] = std::min(_processedRanges[index], closer_distance);
                }
            }
        }
        else
        {
            _processedRanges[i] = std::min(_processedRanges[i], ranges[i]);
        }
        old_distance = ranges[i];
        extendedRanges[i] = _processedRanges[i];
        if (_processedRanges[i] > range_max)
        {
            _processedRanges[i] = 0.0f;
        }
    }

    uint32_t maxDistanceIndex
        = std::distance(_processedRanges.begin(), std::max_element(_processedRanges.begin(), _processedRanges.end()));
    //    = std::distance(_processedRanges.begin() + neg90deg_index, std::max_element(_processedRanges.begin() +
    //    neg90deg_index, _processedRanges.begin() + pos90deg_index));

    // maxDistanceIndex += neg90deg_index;

    float rawTargetAngle = angle_min + maxDistanceIndex * angle_inc;
    _smoothedTargetAngle = computeRollingAverage(rawTargetAngle);
    _targetAngle = _smoothedTargetAngle;

    // Check for obstacles on the side in the turning direction
    // Check left side (angles > 90 degrees)
    // if (_targetAngle > 0)
    // {
    //     for (size_t i = round(size / 2); i < size; i++)
    //     {
    //         float angle = scanMsg_->angle_min + i * scanMsg_->angle_increment;
    //         if (angle > M_PI / 2 && _processedRanges[i] < SAFE_TURNING_DISTANCE)
    //         {
    //             _targetAngle = 0.0f;  // Go straight
    //             break;
    //         }
    //     }
    // }
    // // Check right side (angles < -90 degrees)
    // else
    // {
    //     for (size_t i = 0; i < round(size / 2); i++)
    //     {
    //         float angle = scanMsg_->angle_min + i * scanMsg_->angle_increment;
    //         if (angle < -M_PI / 2 && _processedRanges[i] < SAFE_TURNING_DISTANCE)
    //         {
    //             _targetAngle = 0.0f;  // Go straight
    //             break;
    //         }
    //     }
    // }

    geometry_msgs::msg::PointStamped targetWaypointMsg;
    targetWaypointMsg.header = scanMsg_->header;
    targetWaypointMsg.point.x = _processedRanges[maxDistanceIndex] * std::cos(_targetAngle);
    targetWaypointMsg.point.y = _processedRanges[maxDistanceIndex] * std::sin(_targetAngle);
    _targetWaypointPublisher->publish(targetWaypointMsg);

    sensor_msgs::msg::LaserScan processedScan = *scanMsg_;
    processedScan.ranges = _processedRanges;
    _laserPublisher->publish(processedScan);

    // RCLCPP_INFO(this->get_logger(), "Target angle: %.2f degrees, index: %u, distance: %.2f", _targetAngle * 180.0 / M_PI,
    //             maxDistanceIndex, ranges[maxDistanceIndex]);

    ackermann_msgs::msg::AckermannDriveStamped newMsg = ackermann_msgs::msg::AckermannDriveStamped();

    newMsg.drive.steering_angle = _targetAngle * _overshootFactor;
    float targetSpeed = setSpeedFromDistance(extendedRanges[size / 2], _targetAngle);
    // RCLCPP_INFO(this->get_logger(), "Speed: %0.2f, from distance: %0.2f", targetSpeed, extendedRanges[size / 2]);
    newMsg.drive.speed = targetSpeed;

    _directionPublisher->publish(newMsg);

    if (_debug)
    {
        geometry_msgs::msg::PoseStamped vectorMsg;
        vectorMsg.header = scanMsg_->header;
        vectorMsg.pose.position.x = 0.0f;
        vectorMsg.pose.position.y = 0.0f;
        vectorMsg.pose.position.z = 0.0f;
        vectorMsg.pose.orientation.x = std::cos(_targetAngle / 2);
        vectorMsg.pose.orientation.y = std::sin(_targetAngle / 2);
        vectorMsg.pose.orientation.z = 0.0f;
        vectorMsg.pose.orientation.w = 0.0f;
        _vectorPublisher->publish(vectorMsg);
    }
};

float ReactiveGapFollow::setSpeedFromDistance(float distance_, float steeringAngle_)
{
    float speed = distance_ * _speedDistanceFactor
                  / (1.0f + (std::abs(steeringAngle_) / 4));  // Reduce speed when steering angle is large
    if (speed > _maxSpeed)
    {
        speed = _maxSpeed;
    }
    return speed;
}

float ReactiveGapFollow::computeRollingAverage(float newValue_)
{
    _targetAngleWindow.push_back(newValue_);
    _rollingSum += newValue_;
    if (_targetAngleWindow.size() > _rollingAverageWindow)
    {
        _rollingSum -= _targetAngleWindow.front();
        _targetAngleWindow.pop_front();
    }

    return _rollingSum / _targetAngleWindow.size();
}