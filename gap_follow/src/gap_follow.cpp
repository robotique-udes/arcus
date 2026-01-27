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

    _laserScanSubscriber
        = this->create_subscription<sensor_msgs::msg::LaserScan>(LIDAR_SCAN_TOPIC,
                                                                 DEFAULT_QOS,
                                                                 [this](const sensor_msgs::msg::LaserScan::SharedPtr msg)
                                                                 {
                                                                     this->lidar_CB(msg);
                                                                 });
}

void ReactiveGapFollow::lidar_CB(sensor_msgs::msg::LaserScan::SharedPtr scanMsg_)
{
    std::vector<float> ranges = scanMsg_->ranges;
    size_t rangesSize = ranges.size();
    std::vector<float> preprocessedRanges = ranges;

    float old_distance = std::numeric_limits<float>::infinity();

    for (size_t i = 0; i < rangesSize; i++)
    {
        if (ranges[i] > scanMsg_->range_max || std::isinf(ranges[i]))
        {
            preprocessedRanges[i] = scanMsg_->range_max;
        }
        else if (ranges[i] < scanMsg_->range_min || std::isnan(ranges[i]))
        {
            preprocessedRanges[i] = scanMsg_->range_min;
        }
        else
        {
            preprocessedRanges[i] = ranges[i];
        }
    }

    for (size_t i = 0; i < rangesSize; i++)
    {
        if (std::abs(ranges[i] - old_distance) > DISPARITY_THRESHOLD)
        {
            uint32_t bubble_distance = static_cast<uint32_t>(std::asin(BUBBLE_RADIUS / ranges[i]) / scanMsg_->angle_increment);
            for (int32_t j = 0; j <= static_cast<int32_t>(bubble_distance); j++)
            {
                int32_t index;
                // Extend bubble only on the side of the sudden drop
                if ((ranges[i] < old_distance))
                {
                    index = static_cast<int32_t>(i) - j;
                }
                else
                {
                    index = static_cast<int32_t>(i) + j;
                }

                if (index >= 0 && index < static_cast<int32_t>(rangesSize))
                {
                    preprocessedRanges[index] = std::min(preprocessedRanges[index], old_distance);
                }
            }
        }
        old_distance = ranges[i];
    }

    uint32_t 90deg_index = (M_PI/2 - scanMsg_->angle_min) / scanMsg_->angle_increment;
    uint32_t neg90deg_index = (-M_PI/2 - scanMsg_->angle_min) / scanMsg_->angle_increment;
    

    uint32_t maxDistanceIndex
        = std::distance(preprocessedRanges.begin() + neg90deg_index, std::max_element(preprocessedRanges.begin() + neg90deg_index, preprocessedRanges.begin() + 90deg_index));

    _targetAngle = scanMsg_->angle_min + maxDistanceIndex * scanMsg_->angle_increment;

    // Check for obstacles on the side in the turning direction
    // Check left side (angles > 90 degrees)
    if (_targetAngle > 0)
    {
        for (size_t i = round(rangesSize / 2); i < rangesSize; i++)
        {
            float angle = scanMsg_->angle_min + i * scanMsg_->angle_increment;
            if (angle > M_PI / 2 && preprocessedRanges[i] < SAFE_TURNING_DISTANCE)
            {
                _targetAngle = 0.0f;  // Go straight
                break;
            }
        }
    }
    // Check right side (angles < -90 degrees)
    else
    {
        for (size_t i = 0; i < round(rangesSize / 2); i++)
        {
            float angle = scanMsg_->angle_min + i * scanMsg_->angle_increment;
            if (angle < -M_PI / 2 && preprocessedRanges[i] < SAFE_TURNING_DISTANCE)
            {
                _targetAngle = 0.0f;  // Go straight
                break;
            }
        }
    }

    sensor_msgs::msg::LaserScan processedScan = *scanMsg_;
    processedScan.ranges = preprocessedRanges;
    _laserPublisher->publish(processedScan);

    // RCLCPP_INFO(this->get_logger(), "Target angle: %.2f degrees, index: %u, distance: %.2f", _targetAngle * 180.0 / M_PI,
    // maxDistanceIndex, ranges[maxDistanceIndex]);

    ackermann_msgs::msg::AckermannDriveStamped newMsg = ackermann_msgs::msg::AckermannDriveStamped();

    newMsg.drive.steering_angle = _targetAngle * OVERSHOOT_FACTOR;

    float targetSpeed = setSpeedFromDistance(preprocessedRanges[maxDistanceIndex]);
    newMsg.drive.speed = targetSpeed;

    _directionPublisher->publish(newMsg);
};

float ReactiveGapFollow::setSpeedFromDistance(float distance_)
{
    float speed = distance_ * SPEED_DISTANCE_FACTOR;
    if (speed > MAX_SPEED)
    {
        speed = MAX_SPEED;
    }
    return speed;
}
