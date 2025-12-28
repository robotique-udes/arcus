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
    _directionPublisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(DRIVE_TOPIC, DEFAULT_QOS);

    _error_publisher = this->create_publisher<arcus_msgs::msg::ErrorCode>("/node_error_code", 10);
    _timer = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&ReactiveGapFollow::heartbeat, this));

    _laserScanSubscriber
        = this->create_subscription<sensor_msgs::msg::LaserScan>(LIDAR_SCAN_TOPIC,
                                                                 DEFAULT_QOS,
                                                                 [this](const sensor_msgs::msg::LaserScan::SharedPtr msg)
                                                                 {
                                                                     this->lidar_CB(msg);
                                                                 });
}

void ReactiveGapFollow::heartbeat()
{
    arcus_msgs::msg::ErrorCode error_msg;
    error_msg.source = arcus_msgs::msg::ErrorCode::GAP_FOLLOW;
    error_msg.header.stamp = rclcpp::Clock().now();
    error_msg.error_code = arcus_msgs::msg::ErrorCode::OK;
    this->_error_publisher->publish(error_msg);
}

void ReactiveGapFollow::preprocessLidar(std::vector<float>& ranges_)
{
    size_t rangesSize = ranges_.size();

    for (size_t i = ELIMINATE_EXTREMES_POSITION; i < rangesSize - ELIMINATE_EXTREMES_POSITION; i++)
    {
        ranges_[i]
            = (ranges_[i] + ranges_[i - 1] + ranges_[i - 2] + ranges_[i - 3] + ranges_[i + 1] + ranges_[i + 2] + ranges_[i + 3])
              / PREPROCESSING_AVG_SAMPLES;

        if (ranges_[i] > MAX_LIDAR_DISTANCE_M)
        {
            ranges_[i] = MAX_LIDAR_DISTANCE_M;
        }
    }

    return;
}

void ReactiveGapFollow::lidar_CB(sensor_msgs::msg::LaserScan::SharedPtr scanMsg_)
{
    preprocessLidar(scanMsg_->ranges);

    std::vector<float> ranges = scanMsg_->ranges;
    size_t rangesSize = ranges.size();

    if (!_straight)
    {
        float min_value = 0.f;

        if (!ranges.empty())
        {
            min_value = *std::min_element(ranges.begin(), ranges.end());
        }

        for (size_t i = 0; i < rangesSize; i++)
        {
            if (ranges[i] <= (min_value + BUBBLE_RADIUS))
            {
                ranges[i] = 0.0;
            }
        }

        for (uint32_t i = 0; i < ELIMINATE_EXTREMES_POSITION; i++)
        {
            ranges[i] = 0.0;
        }

        for (uint32_t i = rangesSize - ELIMINATE_EXTREMES_POSITION; i < rangesSize; i++)
        {
            ranges[i] = 0.0;
        }

        uint32_t maxGap = 0;
        uint32_t gap = 0;
        uint32_t startingGapIndex = 0;
        uint32_t endingGapIndex = 0;

        for (uint32_t i = 0; i < rangesSize; i++)
        {
            if (ranges[i] != 0.0)  // Non-zero value: part of a gap
            {
                if (gap == 0)
                {
                    // Start of a new gap
                    startingGapIndex = i;
                }
                gap++;
                endingGapIndex = i;
            }
            else
            {
                if (gap > maxGap)
                {
                    maxGap = gap;
                    _maxGapStartingIndex = startingGapIndex;
                    _maxGapEndingIndex = endingGapIndex;
                }
                gap = 0;
            }
        }

        _targetIndex = (_maxGapEndingIndex - _maxGapStartingIndex) / 2. + _maxGapStartingIndex;

        _targetAngle = scanMsg_->angle_min + _targetIndex * scanMsg_->angle_increment;
    }

    _straight = false;

    ackermann_msgs::msg::AckermannDriveStamped newMsg = ackermann_msgs::msg::AckermannDriveStamped();

    newMsg.drive.steering_angle = _targetAngle * OVERSHOOT_FACTOR;

    float targetSpeed = setSpeedFromDistance(ranges[_targetIndex]);
    newMsg.drive.speed = targetSpeed;

    _directionPublisher->publish(newMsg);
};

float ReactiveGapFollow::setSpeedFromDistance(float distance_)
{
    float speed = distance_ * SPEED_DISTANCE_FACTOR;
    return speed;
}
