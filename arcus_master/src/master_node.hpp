#include "rclcpp/rclcpp.hpp"
#include "arcus_msgs/msg/error_code.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class MasterNode : public rclcpp::Node
{
    static constexpr const char* DISPARITY_DRIVE_TOPIC = "/disparity/drive";
    static constexpr const char* CONTROLLER_DRIVE_TOPIC = "/controller/drive";
    static constexpr const char* SAFETY_DRIVE_TOPIC = "/safety/drive";
    static constexpr const char* PURE_PURSUIT_DRIVE_TOPIC = "/pure_pursuit/drive";

    static constexpr const char* DRIVE_TOPIC = "/drive";

  public:
    MasterNode();

  private:
    void watchdog();
    void mainLoop();
    void errorCodeCallback(const arcus_msgs::msg::ErrorCode::SharedPtr msg);
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr _mainLoopTimer;
    rclcpp::Subscription<arcus_msgs::msg::ErrorCode>::SharedPtr error_listener_;
    rclcpp::Publisher<arcus_msgs::msg::ErrorCode>::SharedPtr error_publisher_;
    uint32_t heartbeats[10];
    ackermann_msgs::msg::AckermannDriveStamped driveCommands[10];
    void processDriveCommands();

    bool emergencyBrakeEngaged = false;

    void disparityDriveCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
    void safetyDriveCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
    void purePursuitDriveCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
    void controllerDriveCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr _drivePublisher;

    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr _disparityDriveSubscriber;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr _safetyDriveSubscriber;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr _purePursuitDriveSubscriber;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr _controllerDriveSubscriber;
};