#ifndef MASTER_NODE_CPP
#define MASTER_NODE_CPP

#include "master_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MasterNode>());
    rclcpp::shutdown();
    return 0;
}

MasterNode::MasterNode():
    Node("master_node")
{
    _errorCodeLatch.fill(arcus_msgs::msg::ErrorCode::OK);
    _lastPublishedErrorCode.fill(arcus_msgs::msg::ErrorCode::OK);

    _priorityController = this->declare_parameter<int>("priority_controller", 0);
    _prioritySafetyOverride = this->declare_parameter<int>("priority_safety_override", 1);
    _priorityPurePursuit = this->declare_parameter<int>("priority_pure_pursuit", 2);
    _priorityDisparity = this->declare_parameter<int>("priority_disparity", 3);

    _disparityDriveTopic = this->declare_parameter<std::string>("disparity_drive_topic", DEFAULT_DISPARITY_DRIVE_TOPIC);
    _controllerDriveTopic = this->declare_parameter<std::string>("controller_drive_topic", DEFAULT_CONTROLLER_DRIVE_TOPIC);
    _safetyDriveTopic = this->declare_parameter<std::string>("safety_drive_topic", DEFAULT_SAFETY_DRIVE_TOPIC);
    _purePursuitDriveTopic = this->declare_parameter<std::string>("pure_pursuit_drive_topic", DEFAULT_PURE_PURSUIT_DRIVE_TOPIC);
    _driveTopic = this->declare_parameter<std::string>("drive_topic", DEFAULT_DRIVE_TOPIC);
    _deadmanTopic = this->declare_parameter<std::string>("deadman_topic", DEFAULT_DEADMAN_TOPIC);
    _nodeErrorTopic = this->declare_parameter<std::string>("node_error_topic", DEFAULT_NODE_ERROR_TOPIC);
    _masterErrorTopic = this->declare_parameter<std::string>("master_error_topic", DEFAULT_MASTER_ERROR_TOPIC);
    _speedLimitTopic = this->declare_parameter<std::string>("speed_limit_topic", DEFAULT_SPEED_LIMIT_TOPIC);
    _forceAlgoTopic = this->declare_parameter<std::string>("force_algo_topic", DEFAULT_FORCE_ALGO_TOPIC);
    _trajectoryRiskTopic = this->declare_parameter<std::string>("trajectory_risk_topic", DEFAULT_TRAJECTORY_RISK_TOPIC);
    _sectionOverrideTimeoutMs = this->declare_parameter<int>("section_override_timeout_ms", 500);
    MAX_ACCEPTED_RISK = this->declare_parameter<double>("max_accepted_risk", 0.1);

    if (_sectionOverrideTimeoutMs < 0)
    {
        RCLCPP_WARN(this->get_logger(), "section_override_timeout_ms must be >= 0, forcing to 0");
        _sectionOverrideTimeoutMs = 0;
    }

    _mainLoopTimer = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&MasterNode::mainLoop, this));

    error_listener_ = this->create_subscription<arcus_msgs::msg::ErrorCode>(
        _nodeErrorTopic,
        10,
        std::bind(&MasterNode::errorCodeCallback, this, std::placeholders::_1));

    error_publisher_ = this->create_publisher<arcus_msgs::msg::ErrorCode>(_masterErrorTopic, 10);

    _drivePublisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(_driveTopic, 10);

    _masterHeartbeatPublisher = this->create_publisher<std_msgs::msg::Bool>(_masterHeartbeatTopic, 10);

    _disparityDriveSubscriber = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        _disparityDriveTopic,
        10,
        std::bind(&MasterNode::disparityDriveCallback, this, std::placeholders::_1));

    _safetyDriveSubscriber = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        _safetyDriveTopic,
        10,
        std::bind(&MasterNode::safetyDriveCallback, this, std::placeholders::_1));

    _purePursuitDriveSubscriber = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        _purePursuitDriveTopic,
        10,
        std::bind(&MasterNode::purePursuitDriveCallback, this, std::placeholders::_1));

    _controllerDriveSubscriber = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        _controllerDriveTopic,
        10,
        std::bind(&MasterNode::controllerDriveCallback, this, std::placeholders::_1));

    _deadmanSubscriber
        = this->create_subscription<std_msgs::msg::Bool>(_deadmanTopic,
                                                         10,
                                                         std::bind(&MasterNode::deadmanCallback, this, std::placeholders::_1));

    _speedLimitSubscriber = this->create_subscription<std_msgs::msg::Float64>(
        _speedLimitTopic,
        10,
        std::bind(&MasterNode::speedLimitCallback, this, std::placeholders::_1));

    _forceAlgoSubscriber = this->create_subscription<std_msgs::msg::String>(
        _forceAlgoTopic,
        10,
        std::bind(&MasterNode::forceAlgoCallback, this, std::placeholders::_1));

    _trajectoryRiskSubscriber = this->create_subscription<std_msgs::msg::Float32>(
        _trajectoryRiskTopic,
        10,
        std::bind(&MasterNode::trajectoryRiskCallback, this, std::placeholders::_1));
}

void MasterNode::errorCodeCallback(const arcus_msgs::msg::ErrorCode::SharedPtr msg)
{
    if (msg->source >= SOURCE_COUNT)
    {
        RCLCPP_ERROR(this->get_logger(), "Received error code from invalid source: %d", msg->source);
        return;
    }

    // Use local receive time so online tracking is immune to clock skew between nodes.
    this->_lastHeartbeatNs[msg->source] = static_cast<uint64_t>(this->now().nanoseconds());
    this->_nodeOnline[msg->source] = msg->error_code != arcus_msgs::msg::ErrorCode::OFFLINE;

    _errorCodeLatch[msg->source] = msg->error_code;

    if (msg->source == arcus_msgs::msg::ErrorCode::SAFETY && msg->error_code == arcus_msgs::msg::ErrorCode::EMERGENCY_BRAKE)
    {
        emergencyBrakeEngaged = true;
    }
    else if (msg->source == arcus_msgs::msg::ErrorCode::SAFETY && msg->error_code == arcus_msgs::msg::ErrorCode::OK)
    {
        emergencyBrakeEngaged = false;
    }

    if (msg->source == arcus_msgs::msg::ErrorCode::PURE_PURSUIT && msg->error_code == arcus_msgs::msg::ErrorCode::EMERGENCY_BRAKE)
    {
        ppRecoveryEngaged = true;
    }
    else if (msg->source == arcus_msgs::msg::ErrorCode::PURE_PURSUIT && msg->error_code == arcus_msgs::msg::ErrorCode::OK)
    {
        ppRecoveryEngaged = false;
    }

    this->tryPublishDriveCommand();
}

void MasterNode::disparityDriveCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
    driveCommands[arcus_msgs::msg::ErrorCode::DISPARITY] = *msg;
    this->tryPublishDriveCommand();
}

void MasterNode::safetyDriveCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
    driveCommands[arcus_msgs::msg::ErrorCode::SAFETY] = *msg;
    this->tryPublishDriveCommand();
}

void MasterNode::purePursuitDriveCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
    driveCommands[arcus_msgs::msg::ErrorCode::PURE_PURSUIT] = *msg;
    this->tryPublishDriveCommand();
}

void MasterNode::controllerDriveCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
    driveCommands[arcus_msgs::msg::ErrorCode::CONTROLLER] = *msg;
    this->tryPublishDriveCommand();
}

void MasterNode::deadmanCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    _deadmanActive = msg->data;
}

void MasterNode::speedLimitCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    _forcedMaxSpeed = msg->data;
    _lastSpeedLimitNs = static_cast<uint64_t>(this->now().nanoseconds());
}

void MasterNode::forceAlgoCallback(const std_msgs::msg::String::SharedPtr msg)
{
    _forcedAlgo = msg->data;
    _lastForceAlgoNs = static_cast<uint64_t>(this->now().nanoseconds());
}

void MasterNode::trajectoryRiskCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    if (msg->data >= MAX_ACCEPTED_RISK)
    {
        RCLCPP_WARN(this->get_logger(), "Received high trajectory risk: %.2f, switching to disparity", msg->data);
        _riskTresholdExceeded = true;
    }
}

bool MasterNode::forcedAlgoToState(const std::string& algo, DriveState& state) const
{
    if (algo == "controller")
    {
        state = DriveState::CONTROLLER;
        return true;
    }
    if (algo == "safety")
    {
        state = DriveState::SAFETY_OVERRIDE;
        return true;
    }
    if (algo == "pure_pursuit")
    {
        state = DriveState::PURE_PURSUIT;
        return true;
    }
    if (algo == "disparity")
    {
        state = DriveState::DISPARITY;
        return true;
    }
    return false;
}

void MasterNode::refreshOnlineStatus()
{
    uint64_t now_ns = this->now().nanoseconds();
    constexpr uint64_t timeout_ns = 300000000;  // 0.3 second timeout for scheduler/network jitter

    for (std::size_t i = 0; i < _nodeOnline.size(); ++i)
    {
        if ((now_ns - _lastHeartbeatNs[i]) > timeout_ns)
        {
            _nodeOnline[i] = false;
        }
    }
}

bool MasterNode::hasCommand(const ackermann_msgs::msg::AckermannDriveStamped& cmd) const
{
    return cmd.drive.speed != 0.0 || cmd.drive.steering_angle != 0.0;
}

MasterNode::DriveState MasterNode::determineDriveState() const
{
    if (!_deadmanActive)
    {
        return DriveState::SAFETY_EMERGENCY;
    }

    if (ppRecoveryEngaged && _nodeOnline[arcus_msgs::msg::ErrorCode::PURE_PURSUIT]
        && hasCommand(driveCommands[arcus_msgs::msg::ErrorCode::PURE_PURSUIT]))
    {
        return DriveState::PURE_PURSUIT;
    }

    if (emergencyBrakeEngaged && _nodeOnline[arcus_msgs::msg::ErrorCode::SAFETY])
    {
        return DriveState::SAFETY_EMERGENCY;
    }

    const bool controller_ready = _nodeOnline[arcus_msgs::msg::ErrorCode::CONTROLLER]
                                  && hasCommand(driveCommands[arcus_msgs::msg::ErrorCode::CONTROLLER]);
    const bool safety_ready = _nodeOnline[arcus_msgs::msg::ErrorCode::SAFETY]
                              && hasCommand(driveCommands[arcus_msgs::msg::ErrorCode::SAFETY]);
    const bool pp_ready = _nodeOnline[arcus_msgs::msg::ErrorCode::PURE_PURSUIT]
                          && hasCommand(driveCommands[arcus_msgs::msg::ErrorCode::PURE_PURSUIT]);
    const bool disparity_ready = _nodeOnline[arcus_msgs::msg::ErrorCode::DISPARITY]
                                 && hasCommand(driveCommands[arcus_msgs::msg::ErrorCode::DISPARITY]);

    const uint64_t now_ns = static_cast<uint64_t>(this->now().nanoseconds());
    const uint64_t override_timeout_ns = static_cast<uint64_t>(_sectionOverrideTimeoutMs) * 1000000ULL;
    const bool force_algo_active = (now_ns - _lastForceAlgoNs) <= override_timeout_ns;

    if (_riskTresholdExceeded && disparity_ready)
    {
        return DriveState::DISPARITY;
    }

    if (force_algo_active)
    {
        DriveState forced_state;
        if (forcedAlgoToState(_forcedAlgo, forced_state))
        {
            if (forced_state == DriveState::CONTROLLER && controller_ready)
            {
                return DriveState::CONTROLLER;
            }
            if (forced_state == DriveState::SAFETY_OVERRIDE && safety_ready)
            {
                return DriveState::SAFETY_OVERRIDE;
            }
            if (forced_state == DriveState::PURE_PURSUIT && pp_ready)
            {
                return DriveState::PURE_PURSUIT;
            }
            if (forced_state == DriveState::DISPARITY && disparity_ready)
            {
                return DriveState::DISPARITY;
            }
        }
    }

    int best_priority = _priorityController;
    DriveState best_state = DriveState::NONE;
    bool found = false;

    if (controller_ready)
    {
        best_priority = _priorityController;
        best_state = DriveState::CONTROLLER;
        found = true;
    }

    if (safety_ready && (!found || _prioritySafetyOverride < best_priority))
    {
        best_priority = _prioritySafetyOverride;
        best_state = DriveState::SAFETY_OVERRIDE;
        found = true;
    }

    if (pp_ready && (!found || _priorityPurePursuit < best_priority))
    {
        best_priority = _priorityPurePursuit;
        best_state = DriveState::PURE_PURSUIT;
        found = true;
    }

    if (disparity_ready && (!found || _priorityDisparity < best_priority))
    {
        best_state = DriveState::DISPARITY;
        found = true;
    }

    return found ? best_state : DriveState::NONE;
}

void MasterNode::tryPublishDriveCommand()
{
    DriveState state = this->determineDriveState();
    ackermann_msgs::msg::AckermannDriveStamped selected_cmd;

    switch (state)
    {
        case DriveState::SAFETY_EMERGENCY:
        {
            selected_cmd = this->driveCommands[arcus_msgs::msg::ErrorCode::SAFETY];
            if (_hasLastNonEmergencySteering)
            {
                selected_cmd.drive.steering_angle = _lastNonEmergencySteering;
            }
            break;
        }
        case DriveState::SAFETY_OVERRIDE:
            selected_cmd = this->driveCommands[arcus_msgs::msg::ErrorCode::SAFETY];
            _lastNonEmergencySteering = this->driveCommands[arcus_msgs::msg::ErrorCode::SAFETY].drive.steering_angle;
            _hasLastNonEmergencySteering = true;
            break;
        case DriveState::CONTROLLER:
            selected_cmd = this->driveCommands[arcus_msgs::msg::ErrorCode::CONTROLLER];
            _lastNonEmergencySteering = this->driveCommands[arcus_msgs::msg::ErrorCode::CONTROLLER].drive.steering_angle;
            _hasLastNonEmergencySteering = true;
            break;
        case DriveState::PURE_PURSUIT:
            selected_cmd = this->driveCommands[arcus_msgs::msg::ErrorCode::PURE_PURSUIT];
            _lastNonEmergencySteering = this->driveCommands[arcus_msgs::msg::ErrorCode::PURE_PURSUIT].drive.steering_angle;
            _hasLastNonEmergencySteering = true;
            break;
        case DriveState::DISPARITY:
            selected_cmd = this->driveCommands[arcus_msgs::msg::ErrorCode::DISPARITY];
            _lastNonEmergencySteering = this->driveCommands[arcus_msgs::msg::ErrorCode::DISPARITY].drive.steering_angle;
            _hasLastNonEmergencySteering = true;
            break;
        case DriveState::NONE:
            selected_cmd = _emptyMsg;
            selected_cmd.drive.speed = 0.0;
            selected_cmd.drive.steering_angle = 0.0;
            break;
    }

    const uint64_t now_ns = static_cast<uint64_t>(this->now().nanoseconds());
    const uint64_t override_timeout_ns = static_cast<uint64_t>(_sectionOverrideTimeoutMs) * 1000000ULL;
    const bool speed_limit_active = (now_ns - _lastSpeedLimitNs) <= override_timeout_ns;
    if (speed_limit_active && selected_cmd.drive.speed > _forcedMaxSpeed)
    {
        selected_cmd.drive.speed = _forcedMaxSpeed;
    }

    _drivePublisher->publish(selected_cmd);
}

void MasterNode::mainLoop()
{
    this->refreshOnlineStatus();

    // Publish error updates only when they change to reduce bus/log spam.
    bool noErrors = true;

    for (size_t i = 0; i < _errorCodeLatch.size(); ++i)
    {
        const uint32_t latched_error = _errorCodeLatch[i];
        if (latched_error != arcus_msgs::msg::ErrorCode::OK)
        {
            noErrors = false;

            if (_lastPublishedErrorCode[i] != latched_error)
            {
                arcus_msgs::msg::ErrorCode error_msg;
                error_msg.source = i;
                error_msg.header.stamp = this->now();
                error_msg.error_code = latched_error;

                this->error_publisher_->publish(error_msg);
                _lastPublishedErrorCode[i] = latched_error;
            }
        }
        else
        {
            _lastPublishedErrorCode[i] = arcus_msgs::msg::ErrorCode::OK;
        }
    }

    if (noErrors && !_lastPublishedNoErrors)
    {
        arcus_msgs::msg::ErrorCode error_msg;
        error_msg.source = arcus_msgs::msg::ErrorCode::MASTER;
        error_msg.header.stamp = this->now();
        error_msg.error_code = arcus_msgs::msg::ErrorCode::OK;

        this->error_publisher_->publish(error_msg);
    }

    _lastPublishedNoErrors = noErrors;

    _masterHeartbeatPublisher->publish(std_msgs::msg::Bool().set__data(true));
}
#endif  // MASTER_NODE_CPP
