#include "gap_follow.hpp"
#include "arcus_msgs/msg/error_code.hpp"
#include <algorithm>

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

// Preprocess lidar points to remove invalid points
void ReactiveGapFollow::preprocess_lidar(std::vector<float> &ranges, float range_max, float range_min)
{
        for(size_t i = 0; i < ranges.size(); i++)
        {
            if (ranges[i] > range_max || std::isinf(ranges[i]) || std::isnan(ranges[i]))
            {
                ranges[i] = range_max + 1.0f;  // Set to a value greater than range_max to indicate invalid reading
            }
            else if (ranges[i] < range_min)
            {
                ranges[i] = range_min;
            }
        }

        // TODO: Remove Lidar point behind car for disparity analysis
}

// Compute the difference between each points and the previous points
void ReactiveGapFollow::get_differences(std::vector<float> &ranges, std::vector<float> &differences)
{
    differences[0] = 0.0;
    for (size_t i = 1; i < ranges.size(); i++)
    {
        differences.push_back(std::abs(ranges[i]-ranges[i-1]));
    }
}

// Get the disparity index according to a certain threshold
void ReactiveGapFollow::get_disparities(std::vector<float> &differences, float threshold, std::vector<int> &disparities)
{
    for (size_t i = 0; i< differences.size(); i++)
    {
        if (differences[i] > threshold)
        {
            disparities.push_back(i);
        }
    }
}

// Compute the number of points representing a certain width at a certain distance frome the car
int ReactiveGapFollow::get_num_points(double width, double distance, double angle_inc)
{
    double angle = std::atan2(width, 2.0*distance);
    return std::ceil(angle/angle_inc);
}

// Cover a certain number of points when extending disparities
void ReactiveGapFollow::cover_points(std::vector<float> &ranges, int cover_direction, int num_points, int start_index)
{
    double new_distance = ranges[start_index];
    if (cover_direction > 0)
    {
        for (int i = 0; i < num_points; i++)
        {
            double next_index = start_index + 1 + i;
            if (next_index >= ranges.size()) break;
            if (ranges[next_index] > new_distance)
            {
                ranges[next_index] = new_distance;
            }
        }
    } else 
    {
        for (int i = 0; i < num_points; i++)
        {
            double next_index = start_index - 1 - i;
            if (next_index < 0) break;
            if (ranges[next_index] > new_distance)
            {
                ranges[next_index] = new_distance;
            }
        }
    }
}

// Extend all disparity to make sure the car doesn't hit walls
void ReactiveGapFollow::extend_disparities(std::vector<float> &ranges, std::vector<int> &disparities, double car_width, int extra_points, double angle_inc)
{
    int index = 0;
    for (size_t i = 0; i < disparities.size(); i++)
    {
        index = disparities[i]-1;
        std::vector<double> slice(ranges.begin()+index, ranges.begin()+index+2);
        auto min_it = std::min_element(slice.begin(), slice.end());
        auto max_it = std::max_element(slice.begin(), slice.end());
        int argmin = std::distance(slice.begin(), min_it);
        int argmax = std::distance(slice.begin(), max_it);
        int close_index = index + argmin;
        int far_index = index + argmax;
        double close_distance = ranges[close_index];

        int num_points = this->get_num_points(car_width, close_distance, angle_inc);
        int direction = close_index - far_index;
        this->cover_points(ranges, direction, num_points + extra_points, close_index);
    }
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
    const float angle_min = scanMsg_->angle_min;
    const float angle_inc = scanMsg_->angle_increment;
    const float range_max = scanMsg_->range_max;
    const float range_min = scanMsg_->range_min;
    
    this->preprocess_lidar(ranges, range_max, range_min);
    std::vector<float> differences;
    std::vector<int> disparities;
    this->get_differences(ranges, differences);
    this->get_disparities(differences, _disparityThreshold, disparities);
    this->extend_disparities(ranges, disparities, _bubbleRadius, 0, angle_inc);

    ackermann_msgs::msg::AckermannDriveStamped newMsg = ackermann_msgs::msg::AckermannDriveStamped();

    int maxDistanceIndex = std::distance(ranges.begin(), std::max_element(ranges.begin(), ranges.end()));
    _targetAngle = angle_min + maxDistanceIndex*angle_inc;

    newMsg.drive.steering_angle = _targetAngle;
    float targetSpeed = setSpeedFromDistance(ranges[ranges.size() / 2.0], _targetAngle);
    // RCLCPP_INFO(this->get_logger(), "Speed: %0.2f, from distance: %0.2f", targetSpeed, extendedRanges[size / 2]);
    newMsg.drive.speed = targetSpeed;

    _directionPublisher->publish(newMsg);

    //if (_debug)
    //{
    //    geometry_msgs::msg::PoseStamped vectorMsg;
    //    vectorMsg.header = scanMsg_->header;
    //    vectorMsg.pose.position.x = 0.0f;
    //    vectorMsg.pose.position.y = 0.0f;
    //    vectorMsg.pose.position.z = 0.0f;
    //    vectorMsg.pose.orientation.x = std::cos(_targetAngle / 2);
    //    vectorMsg.pose.orientation.y = std::sin(_targetAngle / 2);
    //    vectorMsg.pose.orientation.z = 0.0f;
    //    vectorMsg.pose.orientation.w = 0.0f;
    //    _vectorPublisher->publish(vectorMsg);
    //}
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
