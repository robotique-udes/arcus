#include "rclcpp/rclcpp.hpp"
#include "arcus_msgs/msg/error_code.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include <array>
#include <string>

class MasterNode : public rclcpp::Node
{
  static constexpr std::size_t SOURCE_COUNT = 10U;

    static constexpr const char* DEFAULT_DISPARITY_DRIVE_TOPIC = "/disparity/drive";
    static constexpr const char* DEFAULT_CONTROLLER_DRIVE_TOPIC = "/controller/drive";
    static constexpr const char* DEFAULT_SAFETY_DRIVE_TOPIC = "/safety/drive";
    static constexpr const char* DEFAULT_PURE_PURSUIT_DRIVE_TOPIC = "/pure_pursuit/drive";

    static constexpr const char* DEFAULT_DRIVE_TOPIC = "/drive";
    static constexpr const char* DEFAULT_DEADMAN_TOPIC = "/deadman_active";
    static constexpr const char* DEFAULT_NODE_ERROR_TOPIC = "/node_error_code";
    static constexpr const char* DEFAULT_MASTER_ERROR_TOPIC = "/master_error_code";
    static constexpr const char* DEFAULT_MASTER_HEARTBEAT_TOPIC = "/master_heartbeat";
    static constexpr const char* DEFAULT_SPEED_LIMIT_TOPIC = "/track_manager/speed_limit";
    static constexpr const char* DEFAULT_FORCE_ALGO_TOPIC = "/track_manager/forced_algo";

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

    void mainLoop();
    void refreshOnlineStatus();
    void errorCodeCallback(const arcus_msgs::msg::ErrorCode::SharedPtr msg);
    void tryPublishDriveCommand();
    DriveState determineDriveState() const;
    bool forcedAlgoToState(const std::string& algo, DriveState& state) const;
    bool hasCommand(const ackermann_msgs::msg::AckermannDriveStamped& cmd) const;
    std::array<uint32_t, SOURCE_COUNT> _errorCodeLatch{};  // Latches latest error per source.
    std::array<uint32_t, SOURCE_COUNT> _lastPublishedErrorCode{};
    bool _lastPublishedNoErrors = false;

    rclcpp::TimerBase::SharedPtr _mainLoopTimer;
    rclcpp::Subscription<arcus_msgs::msg::ErrorCode>::SharedPtr error_listener_;
    rclcpp::Publisher<arcus_msgs::msg::ErrorCode>::SharedPtr error_publisher_;
    std::array<uint64_t, SOURCE_COUNT> _lastHeartbeatNs{};
    std::array<bool, SOURCE_COUNT> _nodeOnline{};
    std::array<ackermann_msgs::msg::AckermannDriveStamped, SOURCE_COUNT> driveCommands;
    ackermann_msgs::msg::AckermannDriveStamped _emptyMsg;
    bool _hasLastNonEmergencySteering = false;
    double _lastNonEmergencySteering = 0.0;

    int _priorityController = 0;
    int _prioritySafetyOverride = 1;
    int _priorityPurePursuit = 2;
    int _priorityDisparity = 3;

    std::string _disparityDriveTopic = DEFAULT_DISPARITY_DRIVE_TOPIC;
    std::string _controllerDriveTopic = DEFAULT_CONTROLLER_DRIVE_TOPIC;
    std::string _safetyDriveTopic = DEFAULT_SAFETY_DRIVE_TOPIC;
    std::string _purePursuitDriveTopic = DEFAULT_PURE_PURSUIT_DRIVE_TOPIC;
    std::string _driveTopic = DEFAULT_DRIVE_TOPIC;
    std::string _deadmanTopic = DEFAULT_DEADMAN_TOPIC;
    std::string _nodeErrorTopic = DEFAULT_NODE_ERROR_TOPIC;
    std::string _masterErrorTopic = DEFAULT_MASTER_ERROR_TOPIC;
    std::string _masterHeartbeatTopic = DEFAULT_MASTER_HEARTBEAT_TOPIC;
    std::string _speedLimitTopic = DEFAULT_SPEED_LIMIT_TOPIC;
    std::string _forceAlgoTopic = DEFAULT_FORCE_ALGO_TOPIC;

    int _sectionOverrideTimeoutMs = 500;
    double _forcedMaxSpeed = 0.0;
    std::string _forcedAlgo;
    uint64_t _lastSpeedLimitNs = 0U;
    uint64_t _lastForceAlgoNs = 0U;

    bool emergencyBrakeEngaged = false;
    bool ppRecoveryEngaged = false;
    bool _deadmanActive = false;

    void disparityDriveCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
    void safetyDriveCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
    void purePursuitDriveCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
    void controllerDriveCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
    void deadmanCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void speedLimitCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void forceAlgoCallback(const std_msgs::msg::String::SharedPtr msg);

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr _drivePublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _masterHeartbeatPublisher;

    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr _disparityDriveSubscriber;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr _safetyDriveSubscriber;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr _purePursuitDriveSubscriber;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr _controllerDriveSubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _deadmanSubscriber;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr _speedLimitSubscriber;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _forceAlgoSubscriber;
};