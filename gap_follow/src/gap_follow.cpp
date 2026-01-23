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

    float old_distance = 0.f;

    // if (!ranges.empty())
    // {
    //     min_value = *std::min_element(ranges.begin(), ranges.end());
    // }

    for (size_t i = 0; i < rangesSize; i++)
    {   
        if (std::abs(ranges[i] - old_distance) > 0.5)
        {
            uint32_t bubble_distance = static_cast<uint32_t>(2*std::asin(BUBBLE_RADIUS/2*ranges[i])/scanMsg_->angle_increment);
            for (int32_t j = -static_cast<int32_t>(bubble_distance); j <= static_cast<int32_t>(bubble_distance); j++)
            {
                int32_t index = static_cast<int32_t>(i) + j;
                if (index >= 0 && index < static_cast<int32_t>(rangesSize))
                {
                    ranges[index] = std::min(ranges[index], old_distance);
                }
            }
        }
        old_distance = ranges[i];
    }

    for (uint32_t i = 0; i < ELIMINATE_EXTREMES_POSITION; i++)
    {
        ranges[i] = 0.0;
    }

    for (uint32_t i = rangesSize - ELIMINATE_EXTREMES_POSITION; i < rangesSize; i++)
    {
        ranges[i] = 0.0;
    }

    uint32_t maxDistanceIndex = std::distance(ranges.begin(), std::max_element(ranges.begin(), ranges.end()));

    _targetAngle = scanMsg_->angle_min + maxDistanceIndex * scanMsg_->angle_increment;

    ackermann_msgs::msg::AckermannDriveStamped newMsg = ackermann_msgs::msg::AckermannDriveStamped();

    newMsg.drive.steering_angle = _targetAngle * OVERSHOOT_FACTOR;

    float targetSpeed = setSpeedFromDistance(ranges[maxDistanceIndex]);
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
