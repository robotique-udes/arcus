#include "rclcpp/rclcpp.hpp"
#include "arcus_msgs/msg/error_code.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include <array>

class MasterNode : public rclcpp::Node
{
    static constexpr const char* DISPARITY_DRIVE_TOPIC = "/disparity/drive";
    static constexpr const char* CONTROLLER_DRIVE_TOPIC = "/controller/drive";
    static constexpr const char* SAFETY_DRIVE_TOPIC = "/safety/drive";
    static constexpr const char* PURE_PURSUIT_DRIVE_TOPIC = "/pure_pursuit/drive";

    static constexpr const char* DRIVE_TOPIC = "/drive";
    static constexpr const char* DEADMAN_TOPIC = "/deadman_active";

  public:
    MasterNode();

  private:
    enum class DriveState
    {
        NONE,
        SAFETY_EMERGENCY,
        SAFETY_OVERRIDE,
        CONTROLLER,
        PURE_PURSUIT,
        DISPARITY,
    };

    void watchdog();
    void mainLoop();
    void refreshOnlineStatus();
    void errorCodeCallback(const arcus_msgs::msg::ErrorCode::SharedPtr msg);
    void tryPublishDriveCommand();
    DriveState determineDriveState() const;
    bool hasCommand(const ackermann_msgs::msg::AckermannDriveStamped& cmd) const;
    std::array<uint32_t, 10> _errorCodeLatch; // Latchs the error until condition goes away;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr _mainLoopTimer;
    rclcpp::Subscription<arcus_msgs::msg::ErrorCode>::SharedPtr error_listener_;
    rclcpp::Publisher<arcus_msgs::msg::ErrorCode>::SharedPtr error_publisher_;
    std::array<uint64_t, 10> _lastHeartbeatNs{};
    std::array<bool, 10> _nodeOnline{};
    std::array<ackermann_msgs::msg::AckermannDriveStamped, 10> driveCommands{};
    ackermann_msgs::msg::AckermannDriveStamped _emptyMsg;
    bool _hasLastNonEmergencySteering = false;
    double _lastNonEmergencySteering = 0.0;

    bool emergencyBrakeEngaged = false;
    bool ppRecoveryEngaged = false;
    bool _deadmanActive = false;

    void disparityDriveCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
    void safetyDriveCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
    void purePursuitDriveCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
    void controllerDriveCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
    void deadmanCallback(const std_msgs::msg::Bool::SharedPtr msg);

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr _drivePublisher;

    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr _disparityDriveSubscriber;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr _safetyDriveSubscriber;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr _purePursuitDriveSubscriber;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr _controllerDriveSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _deadmanSubscriber;
};