#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class DriveController : public rclcpp::Node
{
  public:
    DriveController():
        Node("motor_sweep_test"),
        count_(0)
    {
        _drive_publisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        _joy_sub
            = create_subscription<sensor_msgs::msg::Joy>("/joy",
                                                         rclcpp::QoS{10},
                                                         std::bind(&DriveController::joy_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "DriveController node has been started.");
    }

  private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
    {
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        // Binding for ps4 controller, temporary comment to say axe 5 is right trigger and axe 2 is left trigger, axe 3 is left
        // stick horizontal
        float throttle = (joy_msg->axes[5] - joy_msg->axes[2]) / 2.0;  // Assuming axes[5] is throttle and axes[2] is brake
        float steering = (-joy_msg->axes[3] + 1.0) / 2.0;
        drive_msg.drive.speed = throttle;
        drive_msg.drive.steering_angle = steering;
        _drive_publisher->publish(drive_msg);
    }

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr _drive_publisher;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joy_sub;

    size_t count_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriveController>());
    rclcpp::shutdown();
    return 0;
}
