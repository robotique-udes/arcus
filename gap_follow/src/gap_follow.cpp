#include "gap_follow.hpp"

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
    _directionPublisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(DRIVE_TOPIC, DEFAULT_QOS);

    _laserPublisher = this->create_publisher<sensor_msgs::msg::LaserScan>("processed_scan", DEFAULT_QOS);

    _targetWaypointPublisher = this->create_publisher<geometry_msgs::msg::PointStamped>("target_waypoint", DEFAULT_QOS);

    _laserScanSubscriber
        = this->create_subscription<sensor_msgs::msg::LaserScan>(LIDAR_SCAN_TOPIC,
                                                                 DEFAULT_QOS,
                                                                 [this](const sensor_msgs::msg::LaserScan::SharedPtr msg)
                                                                 {
                                                                     this->lidar_CB(msg);
                                                                 });

    _vectorPublisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("target_vector", DEFAULT_QOS);


    
}

void ReactiveGapFollow::lidar_CB(sensor_msgs::msg::LaserScan::SharedPtr scanMsg_)
{
    std::vector<float> ranges = scanMsg_->ranges;
    size_t rangesSize = ranges.size();
    std::vector<float> preprocessedRanges = ranges;
    std::vector<float> extendedRanges = ranges;

    float old_distance = scanMsg_->range_max;

    //uint32_t pos90deg_index = (M_PI/2 - scanMsg_->angle_min) / scanMsg_->angle_increment;
    //uint32_t neg90deg_index = (-M_PI/2 - scanMsg_->angle_min) / scanMsg_->angle_increment;

    for (size_t i = 0; i < rangesSize; i++)
    {
        if (ranges[i] > scanMsg_->range_max || std::isinf(ranges[i]) || std::isnan(ranges[i]))
        {
            ranges[i] = scanMsg_->range_max+1.0f;  // Set to a value greater than range_max to indicate invalid reading
        }
        else if (ranges[i] < scanMsg_->range_min )
        {
            ranges[i] = scanMsg_->range_min;
        }

        if (std::abs(ranges[i] - old_distance) > DISPARITY_THRESHOLD)
        {
            float closer_distance = std::min(ranges[i], old_distance);
            uint32_t bubble_distance = static_cast<uint32_t>(std::atan(BUBBLE_RADIUS / closer_distance) / scanMsg_->angle_increment);
            if (std::isnan(bubble_distance))
            {
                bubble_distance = static_cast<uint32_t>((M_PI / 2 - std::atan(closer_distance / BUBBLE_RADIUS)) / scanMsg_->angle_increment);
            }
            for (int32_t j = -static_cast<int32_t>(bubble_distance); j <= static_cast<int32_t>(bubble_distance); j++)
            {
                int32_t index;
                index = static_cast<int32_t>(i) + j;

                if (index >= 0 && index < static_cast<int32_t>(rangesSize))
                {
                    preprocessedRanges[index] = std::min(preprocessedRanges[index], closer_distance);
                }
            }
        } else
        {
            preprocessedRanges[i] = std::min(preprocessedRanges[i], ranges[i]);
        }
        old_distance = ranges[i];
        extendedRanges[i] = preprocessedRanges[i];
        if (preprocessedRanges[i] > scanMsg_->range_max)
        {
            preprocessedRanges[i] = 0.0f;
        }
    }

    uint32_t maxDistanceIndex = std::distance(preprocessedRanges.begin(), std::max_element(preprocessedRanges.begin(), preprocessedRanges.end()));
    //    = std::distance(preprocessedRanges.begin() + neg90deg_index, std::max_element(preprocessedRanges.begin() + neg90deg_index, preprocessedRanges.begin() + pos90deg_index));
    
    //maxDistanceIndex += neg90deg_index;

    float rawTargetAngle = scanMsg_->angle_min + maxDistanceIndex * scanMsg_->angle_increment;
    _smoothedTargetAngle = computeRollingAverage(rawTargetAngle);
    _targetAngle = _smoothedTargetAngle;


    // Check for obstacles on the side in the turning direction
    // Check left side (angles > 90 degrees)
    // if (_targetAngle > 0)
    // {
    //     for (size_t i = round(rangesSize / 2); i < rangesSize; i++)
    //     {
    //         float angle = scanMsg_->angle_min + i * scanMsg_->angle_increment;
    //         if (angle > M_PI / 2 && preprocessedRanges[i] < SAFE_TURNING_DISTANCE)
    //         {
    //             _targetAngle = 0.0f;  // Go straight
    //             break;
    //         }
    //     }
    // }
    // // Check right side (angles < -90 degrees)
    // else
    // {
    //     for (size_t i = 0; i < round(rangesSize / 2); i++)
    //     {
    //         float angle = scanMsg_->angle_min + i * scanMsg_->angle_increment;
    //         if (angle < -M_PI / 2 && preprocessedRanges[i] < SAFE_TURNING_DISTANCE)
    //         {
    //             _targetAngle = 0.0f;  // Go straight
    //             break;
    //         }
    //     }
    // }

    geometry_msgs::msg::PointStamped targetWaypointMsg;
    targetWaypointMsg.header = scanMsg_->header;
    targetWaypointMsg.point.x = preprocessedRanges[maxDistanceIndex] * std::cos(_targetAngle);
    targetWaypointMsg.point.y = preprocessedRanges[maxDistanceIndex] * std::sin(_targetAngle);
    _targetWaypointPublisher->publish(targetWaypointMsg);

    sensor_msgs::msg::LaserScan processedScan = *scanMsg_;
    processedScan.ranges = preprocessedRanges;
    _laserPublisher->publish(processedScan);

    // RCLCPP_INFO(this->get_logger(), "Target angle: %.2f degrees, index: %u, distance: %.2f", _targetAngle * 180.0 / M_PI,
    // maxDistanceIndex, ranges[maxDistanceIndex]);

    ackermann_msgs::msg::AckermannDriveStamped newMsg = ackermann_msgs::msg::AckermannDriveStamped();

    newMsg.drive.steering_angle = _targetAngle * OVERSHOOT_FACTOR;

    float targetSpeed = setSpeedFromDistance(extendedRanges[rangesSize / 2], _targetAngle);  // Use the distance directly in front of the robot to set the speed
    newMsg.drive.speed = targetSpeed;

    
    _directionPublisher->publish(newMsg);

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
    
};


float ReactiveGapFollow::setSpeedFromDistance(float distance_, float steeringAngle_)
{
    float speed = distance_ * SPEED_DISTANCE_FACTOR / (1.0f + 2.0f*std::abs(steeringAngle_));  // Reduce speed when steering angle is large
    if (speed > MAX_SPEED)
    {
        speed = MAX_SPEED;
    }
    return speed;
}

float ReactiveGapFollow::computeRollingAverage(float newValue_)
{
    _targetAngleWindow.push_back(newValue_);
    if (_targetAngleWindow.size() > ROLLING_AVERAGE_WINDOW)
    {
        _targetAngleWindow.pop_front();
    }
    
    float sum = 0.0f;
    for (float angle : _targetAngleWindow)
    {
        sum += angle;
    }
    return sum / _targetAngleWindow.size();
}