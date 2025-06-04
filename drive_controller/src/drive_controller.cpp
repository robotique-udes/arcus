#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joy.hpp"


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class DriveController : public rclcpp::Node
{
  public:
    DriveController()
    : Node("motor_sweep_test"), count_(0)
    {
      speed_publisher_ = this->create_publisher<std_msgs::msg::Float64>("commands/motor/duty_cycle", 10);
      steering_publisher_ = this->create_publisher<std_msgs::msg::Float64>("commands/servo/position", 10);
      joy_sub_ = create_subscription<sensor_msgs::msg::Joy>("/joy", rclcpp::QoS{10}, std::bind(&DriveController::joy_callback, this, std::placeholders::_1));

    }

  private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
    {
      auto throttle_msg = std_msgs::msg::Float64();
      auto steering_msg = std_msgs::msg::Float64();

      float throttle = joy_msg->axes[5] - joy_msg->axes[2];
      float steering = (joy_msg->axes[3]+1.0)/2.0;
      throttle_msg.data = throttle;
      steering_msg.data = steering;
      speed_publisher_->publish(throttle_msg);
      steering_publisher_->publish(steering_msg);
    }


    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DriveController>());
  rclcpp::shutdown();
  return 0;
}
