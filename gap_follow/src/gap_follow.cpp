#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <algorithm>
/// CHECK: include needed ROS msg type headers and libraries

class ReactiveFollowGap : public rclcpp::Node
{
  public:
    ReactiveFollowGap():
        Node("reactive_node")
    {
        _direction_publisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
        _laserScan_subscriber
            = this->create_subscription<sensor_msgs::msg::LaserScan>(lidarscan_topic,
                                                                     10,
                                                                     [this](const sensor_msgs::msg::LaserScan::SharedPtr msg)
                                                                     {
                                                                         this->lidar_callback(msg);
                                                                     });
    }

  private:
    int overshootFactor = 1;

    bool STRAIGHT = false;

    float past_best_angle = 0;
    int consecutive_target = 0;

    float best_angle = 0;

    uint32_t MAXstartingGapIndex = 0;
    uint32_t MAXendingGapIndex = 0;

    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr _direction_publisher;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _laserScan_subscriber;
    /// TODO: create ROS subscribers and publishers

    void preprocess_lidar(std::vector<float>& ranges)
    {
        uint32_t rangesSize = ranges.size();
        for (uint32_t i = 4; i < rangesSize - 4; ++i)
        {
            ranges[i]
                = (ranges[i] + ranges[i - 1] + ranges[i - 2] + ranges[i - 3] + ranges[i + 1] + ranges[i + 2] + ranges[i + 3])
                  / 7.;

            if (ranges[i] > 10)
                ranges[i] = 10;
        }
        return;
    }

    void lidar_callback(sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        preprocess_lidar(scan_msg->ranges);
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        /// TODO:
        // Find closest point to LiDAR
        std::vector<float> ranges = scan_msg->ranges;
        uint32_t rangesSize = ranges.size();

        if (!STRAIGHT)
        {
            float min_value = 0;

            if (!ranges.empty())
            {
                min_value = *std::min_element(ranges.begin(), ranges.end());
                // RCLCPP_INFO(this->get_logger(), "min_val_range: %f", min_value);
            }
            // Eliminate all points inside 'bubble' (set them to zero)
            uint32_t bubble_radius = 1;

            for (uint32_t i = 0; i < rangesSize; i++)
            {
                if (ranges[i] <= (min_value + bubble_radius))
                {
                    // RCLCPP_INFO(this->get_logger(), "removed index: %d", i);
                    ranges[i] = 0.0;  // Mark as invalid (cleared)
                }
            }

            for (uint32_t i = 0; i < 4; i++)
            {
                ranges[i] = 0.0;
            }

            for (uint32_t i = rangesSize - 4; i < rangesSize; i++)
            {
                ranges[i] = 0.0;
            }
            // Find max length gap
            uint32_t max_gap = 0;
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
                    endingGapIndex = i;  // Increase the gap count
                }
                else
                {
                    if (gap > max_gap)
                    {
                        max_gap = gap;
                        MAXstartingGapIndex = startingGapIndex;
                        MAXendingGapIndex = endingGapIndex;
                    }
                    gap = 0;  // Reset gap counter for the next potential gap
                }
            }

            if (gap > max_gap)
            {
                max_gap = gap;
                MAXstartingGapIndex = startingGapIndex;
                MAXendingGapIndex = endingGapIndex;
            }

            uint32_t best_index = (MAXendingGapIndex - MAXstartingGapIndex) / 2. + MAXstartingGapIndex;

            best_angle = scan_msg->angle_min + best_index * scan_msg->angle_increment;

            past_best_angle = best_angle;
        }

        STRAIGHT = false;

        ackermann_msgs::msg::AckermannDriveStamped new_msg = ackermann_msgs::msg::AckermannDriveStamped();

        new_msg.drive.steering_angle = best_angle * overshootFactor;
        new_msg.drive.speed = 6;  // 6 for montreal

        _direction_publisher->publish(new_msg);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}