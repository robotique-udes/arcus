#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MotorSweepTest : public rclcpp::Node
{
  public:
    MotorSweepTest():
        Node("motor_sweep_test"),
        count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("commands/motor/duty_cycle", 10);
        publisher2_ = this->create_publisher<std_msgs::msg::Float64>("commands/servo/position", 10);
        timer_ = this->create_wall_timer(4000ms, std::bind(&MotorSweepTest::timer_callback, this));
    }

  private:
    void timer_callback()
    {
        auto message = std_msgs::msg::Float64();
        auto servo_msg = std_msgs::msg::Float64();
        for (int i = 0; i < 10; i++)
        {
            message.data = -i / 100.0;
            publisher_->publish(message);
            servo_msg.data = i / 10.0;
            publisher2_->publish(servo_msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

        for (int i = 10; i >= 0; i--)
        {
            message.data = -i / 100.0;
            publisher_->publish(message);
            servo_msg.data = i / 10.0;
            publisher2_->publish(servo_msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher2_;

    size_t count_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorSweepTest>());
    rclcpp::shutdown();
    return 0;
}
