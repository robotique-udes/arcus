#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/header.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class FakeImuPublisher : public rclcpp::Node
{
public:
    FakeImuPublisher()
        : Node("fake_imu_publisher"),
          prev_vx_(0.0), prev_vy_(0.0), yaw_(0.0)
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&FakeImuPublisher::twist_callback, this, std::placeholders::_1));

        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);

        prev_time_ = this->now();

        // Timer to publish IMU at 50 Hz (every 20ms)
        timer_ = this->create_wall_timer(
            20ms, std::bind(&FakeImuPublisher::publish_imu, this));
    }

private:
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Save the latest twist command
        last_twist_ = *msg;
        last_twist_time_ = this->now();
    }

    void publish_imu()
    {
        rclcpp::Time now = this->now();
        double dt = (now - prev_time_).seconds();

        if (dt <= 0.0)
            return;

        // Use last known velocities
        double vx = last_twist_.linear.x;
        double vy = last_twist_.linear.y;
        double wz = last_twist_.angular.z;

        // Finite difference acceleration
        double ax = (vx - prev_vx_) / dt;
        double ay = (vy - prev_vy_) / dt;

        // Integrate yaw
        yaw_ += wz * dt;

        // Convert to quaternion
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw_);
        q.normalize();

        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = now;
        imu_msg.header.frame_id = "ego_racecar/base_link";

        // Orientation
        imu_msg.orientation = tf2::toMsg(q);
        imu_msg.orientation_covariance = {
            0.001, 0.0, 0.0,
            0.0, 0.001, 0.0,
            0.0, 0.0, 0.001};

        // Angular velocity
        imu_msg.angular_velocity.x = 0.0;
        imu_msg.angular_velocity.y = 0.0;
        imu_msg.angular_velocity.z = wz;
        imu_msg.angular_velocity_covariance = {
            0.001, 0.0, 0.0,
            0.0, 0.001, 0.0,
            0.0, 0.0, 0.001};

        // Linear acceleration
        imu_msg.linear_acceleration.x = ax;
        imu_msg.linear_acceleration.y = ay;
        imu_msg.linear_acceleration.z = 0.0;
        imu_msg.linear_acceleration_covariance = {
            0.001, 0.0, 0.0,
            0.0, 0.001, 0.0,
            0.0, 0.0, 0.001};

        imu_publisher_->publish(imu_msg);

        // Update previous values
        prev_vx_ = vx;
        prev_vy_ = vy;
        prev_time_ = now;
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Twist last_twist_;
    rclcpp::Time last_twist_time_;

    double prev_vx_, prev_vy_, yaw_;
    rclcpp::Time prev_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeImuPublisher>());
    rclcpp::shutdown();
    return 0;
}
