#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "arcus_msgs/msg/controller_type.hpp"
#include "arcus_msgs/msg/error_code.hpp"

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
        _controller_type_subscription = this->create_subscription<arcus_msgs::msg::ControllerType>(
            "/controller_type",
            rclcpp::QoS{10},
            std::bind(&DriveController::controller_type_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "DriveController node has been started.");
    }

  private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
    {
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        float throttle = (joy_msg->axes[this->right_trigger] - joy_msg->axes[this->left_trigger]) / 2.0;
        float steering = (-joy_msg->axes[this->left_axis_x] + 1.0) / 2.0;
        drive_msg.drive.speed = throttle;
        drive_msg.drive.steering_angle = steering;
        _drive_publisher->publish(drive_msg);
    }

    void controller_type_callback(const arcus_msgs::msg::ControllerType::SharedPtr controller_type_msg)
    {
        switch (controller_type_msg->controller_enum)
        {
            case arcus_msgs::msg::ControllerType::LOGITECH:
                /* code */
                this->left_axis_x = 0;  // Not sure about this mapping
                this->left_axis_y = 1;  // Not sure about this mapping
                this->right_axis_x = 3;
                this->right_axis_y = 4;  // Not sure about this mapping
                this->left_trigger = 2;
                this->right_trigger = 5;
                break;
            case arcus_msgs::msg::ControllerType::PS4:
                /* code */
                RCLCPP_ERROR(this->get_logger(), "Mapping not defined for PS4 controller yet.");
                break;
            case arcus_msgs::msg::ControllerType::XBOX:
                /* code */
                RCLCPP_ERROR(this->get_logger(), "Mapping not defined for XBOX controller yet.");
                break;

            default:
                break;
        };
    }

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr _drive_publisher;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joy_sub;
    rclcpp::Subscription<arcus_msgs::msg::ControllerType>::SharedPtr _controller_type_subscription;

    size_t count_;
    uint8_t left_axis_x;
    uint8_t left_axis_y;
    uint8_t right_axis_x;
    uint8_t right_axis_y;
    uint8_t left_trigger;
    uint8_t right_trigger;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DriveController>());
    rclcpp::shutdown();
    return 0;
}
