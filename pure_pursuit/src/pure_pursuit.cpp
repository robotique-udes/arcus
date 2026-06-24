#include "pure_pursuit.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}

PurePursuit::PurePursuit():
    Node("pure_pursuit"),
    _tfBuffer(this->get_clock()),
    _tfListener(_tfBuffer)
{
    this->handleRosParam();
    this->initRosElements();
    this->initParamCallbackHandle();
    this->loadWaypointsFromCSV();
    this->calculateSpeed();
}

void PurePursuit::CB_mainDecisionLoop(void)
{
    if (_waypoints.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "No waypoints loaded. Cannot publish drive command.");
        return;
    }

    double lookAheadDistance = lookaheadGain * _currentSpeed;
    double riskLookaheadDistance = riskLookaheadGain * _currentSpeed;

    double clippedLookAheadDistance = this->clipLookaheadDistance(lookAheadDistance);

    waypoint_t lookaheadPoint = this->getLookaheadPoint(clippedLookAheadDistance);

    // Calculate and publish trajectory risk
    double trajectoryRisk = this->calculateTrajectoryRisk(riskLookaheadDistance);

    // Gatekeep for publishing trajectory risk to avoid flooding the topic with similar trajectory
    if (!_hasLastTrajectoryRisk || std::abs(trajectoryRisk - _lastTrajectoryRisk) > EPS)
    {
        this->CB_publishTrajectoryRisk();
        _lastTrajectoryRisk = trajectoryRisk;
        _hasLastTrajectoryRisk = true;
    }

    // Publish the path segment used for risk calculation and target waypoint for visualization
    if (debug)
    {
        this->CB_publishRiskPathSegment();
        this->CB_publishTargetWaypoint(lookaheadPoint.point);
    }

    ackermann_msgs::msg::AckermannDriveStamped purePursuitDriveCmd = this->getPurePursuitDriveCommand(lookaheadPoint);
    double steeringAngle = purePursuitDriveCmd.drive.steering_angle;

    // Decision tree for recovery mode, will overwrite 'purePursuitDriveCmd' if needed
    if (_recoveryActive)
    {
        RCLCPP_DEBUG(this->get_logger(),
                     "Recovery Active: steeringAngle=%.4f, threshold=%.4f",
                     std::abs(steeringAngle),
                     recoveryDisengageSteerRad);

        if (std::abs(steeringAngle) < recoveryDisengageSteerRad)
        {
            _recoveryActive = false;
            RCLCPP_INFO(this->get_logger(), "Recovery Mode DISENGAGED: steering angle now below threshold");
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(),
                         "Recovery Mode: Sending reverse command (speed=%.2f, steeringAngle=%.4f)",
                         -recoveryReverseSpeed,
                         _recoverySteeringAngle);
            purePursuitDriveCmd.drive.steering_angle = -_recoverySteeringAngle;
            purePursuitDriveCmd.drive.speed = -recoveryReverseSpeed;
            _driveCmdPublisher->publish(purePursuitDriveCmd);
            return;
        }
    }

    if (_recoveryArmed && (_currentSpeed < recoveryTriggerSpeed))
    {
        RCLCPP_WARN(this->get_logger(),
                    "Recovery Mode ACTIVATED: carHasEverMoved=%d, currentSpeed=%.4f, triggerSpeed=%.4f",
                    _carHasEverMoved,
                    _currentSpeed,
                    recoveryTriggerSpeed);
        _recoveryActive = true;
        _recoveryArmed = false;
        _recoverySteeringAngle = steeringAngle;

        RCLCPP_INFO(this->get_logger(),
                    "Sending initial recovery reverse command (speed=%.2f, steeringAngle=%.4f)",
                    -recoveryReverseSpeed,
                    _recoverySteeringAngle);
        purePursuitDriveCmd.drive.steering_angle = _recoverySteeringAngle;
        purePursuitDriveCmd.drive.speed = -recoveryReverseSpeed;
        _driveCmdPublisher->publish(purePursuitDriveCmd);
        return;
    }

    _driveCmdPublisher->publish(purePursuitDriveCmd);
}

ackermann_msgs::msg::AckermannDriveStamped PurePursuit::getPurePursuitDriveCommand(const waypoint_t& _waypoint)
{
    float targetSpeed = _waypoint.speed;

    double dx = _waypoint.point.pose.position.x - _currentX;
    double dy = _waypoint.point.pose.position.y - _currentY;
    double lookaheadDistanceActual = std::sqrt(dx * dx + dy * dy);

    // Transform (dx, dy) from world to vehicle local frame
    double localX = std::cos(-_currentYaw) * dx - std::sin(-_currentYaw) * dy;
    double localY = std::sin(-_currentYaw) * dx + std::cos(-_currentYaw) * dy;

    double alpha = std::atan2(localY, localX);
    double steeringAngle = std::atan2(2.0 * wheelbase * std::sin(alpha), lookaheadDistanceActual);

    ackermann_msgs::msg::AckermannDriveStamped driveCmd;
    driveCmd.header.stamp = this->now();
    driveCmd.header.frame_id = "base_link";
    driveCmd.drive.steering_angle = steeringAngle;
    driveCmd.drive.speed = targetSpeed;
    return driveCmd;
}

void PurePursuit::CB_positionSubscriber(const nav_msgs::msg::Odometry& msg)
{
    _currentX = msg.pose.pose.position.x;
    _currentY = msg.pose.pose.position.y;

    // Orientation (quaternion -> yaw)
    const auto& q = msg.pose.pose.orientation;

    _currentYaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    _currentSpeed = msg.twist.twist.linear.x;

    // Track if the car has ever moved to prevent recovery from activating at startup
    if (_currentSpeed > 0.0 && !_carHasEverMoved)
    {
        _carHasEverMoved = true;
        RCLCPP_INFO(this->get_logger(), "Car has started moving for the first time! Speed: %.4f m/s", _currentSpeed);
    }

    if (_carHasEverMoved && (_currentSpeed > recoveryRearmSpeed))
    {
        if (!_recoveryArmed)
        {
            _recoveryArmed = true;
            RCLCPP_DEBUG(this->get_logger(),
                         "Recovery Mode ARMED: speed (%.4f) > rearm threshold (%.4f)",
                         _currentSpeed,
                         recoveryRearmSpeed);
        }
    }
}

void PurePursuit::CB_costmapSubscriber(const nav_msgs::msg::OccupancyGrid& msg)
{
    std::lock_guard<std::mutex> lock(_costmapMutex);
    _costmapData = msg.data;
    _costmapWidth = msg.info.width;
    _costmapHeight = msg.info.height;
    _costmapResolution = msg.info.resolution;
    _costmapOriginX = msg.info.origin.position.x;
    _costmapOriginY = msg.info.origin.position.y;
    _costmapFrameId = msg.header.frame_id;
}

void PurePursuit::CB_publishTargetWaypoint(const geometry_msgs::msg::PoseStamped& msg)
{
    double x = msg.pose.position.x;
    double y = msg.pose.position.y;
    geometry_msgs::msg::PointStamped newMsg;
    newMsg.point.x = x;
    newMsg.point.y = y;
    newMsg.header.frame_id = "map";
    newMsg.header.stamp = this->get_clock()->now();
    _targetWaypointPublisher->publish(newMsg);
}

void PurePursuit::CB_publishRiskPathSegment()
{
    nav_msgs::msg::Path pathMsg;
    pathMsg.header.frame_id = "map";
    pathMsg.header.stamp = this->get_clock()->now();
    pathMsg.poses = _riskPathWaypoints;
    _riskPathPublisher->publish(pathMsg);
}

void PurePursuit::CB_publishTrajectoryRisk()
{
    std_msgs::msg::Float32 riskMsg;
    riskMsg.data = static_cast<float>(_lastTrajectoryRisk);
    _trajectoryRiskPublisher->publish(riskMsg);
}

void PurePursuit::handleRosParam(void)
{
    this->declare_parameter<std::string>("waypoints_file_path", waypointsFilePath);
    this->declare_parameter<std::string>("position_topic", positionTopic);
    this->declare_parameter<std::string>("drive_command_topic", driveCmdTopic);
    this->declare_parameter<std::string>("target_waypoint_topic", targetWaypointTopic);
    this->declare_parameter<std::string>("costmap_topic", costmapTopic);
    this->declare_parameter<std::string>("trajectory_risk_topic", trajectoryRiskTopic);
    this->declare_parameter<std::string>("error_topic", errorTopic);
    this->declare_parameter<std::string>("risk_path_topic", riskPathTopic);
    this->declare_parameter("debug", debug);

    this->declare_parameter("max_lookahead_distance_m", maxLookahead);
    this->declare_parameter("min_lookahead_distance_m", minLookahead);
    this->declare_parameter("lookahead_distance_gain", lookaheadGain);
    this->declare_parameter("risk_lookahead_gain", riskLookaheadGain);
    this->declare_parameter("ttc_decay_rate", ttcDecayRate);
    this->declare_parameter("min_ttc_speed_mps", ttcMinSpeed);
    this->declare_parameter("risk_interpolation_step_m", riskInterpolationStep);
    this->declare_parameter("reloc_distance_m", relocalizeDistance);                  // !!!!!!!!!
    this->declare_parameter("max_lookahead_fraction_of_path", maxLookaheadFraction);  // !!!!!!!!!
    this->declare_parameter("loop_frequency_hz", loopFrequency);
    this->declare_parameter("wheelbase_m", wheelbase);
    this->declare_parameter("speed_min", speedMin);
    this->declare_parameter("speed_max", speedMax);
    this->declare_parameter("a_lat_max", latAccelMax);
    this->declare_parameter("a_accel_max", longAccelMax);
    this->declare_parameter("a_brake_max", longBrakeMax);

    try
    {
        waypointsFilePath = this->get_parameter("waypoints_file_path").as_string();
        positionTopic = this->get_parameter("position_topic").as_string();
        driveCmdTopic = this->get_parameter("drive_command_topic").as_string();
        targetWaypointTopic = this->get_parameter("target_waypoint_topic").as_string();
        costmapTopic = this->get_parameter("costmap_topic").as_string();
        trajectoryRiskTopic = this->get_parameter("trajectory_risk_topic").as_string();
        errorTopic = this->get_parameter("error_topic").as_string();
        riskPathTopic = this->get_parameter("risk_path_topic").as_string();
        debug = this->get_parameter("debug").as_bool();

        maxLookahead = this->get_parameter("max_lookahead_distance_m").as_double();
        minLookahead = this->get_parameter("min_lookahead_distance_m").as_double();
        lookaheadGain = this->get_parameter("lookahead_distance_gain").as_double();
        riskLookaheadGain = this->get_parameter("risk_lookahead_gain").as_double();
        ttcDecayRate = this->get_parameter("ttc_decay_rate").as_double();
        ttcMinSpeed = this->get_parameter("min_ttc_speed_mps").as_double();
        riskInterpolationStep = this->get_parameter("risk_interpolation_step_m").as_double();
        relocalizeDistance = this->get_parameter("reloc_distance_m").as_double();                  // !!
        maxLookaheadFraction = this->get_parameter("max_lookahead_fraction_of_path").as_double();  // !!
        loopFrequency = this->get_parameter("loop_frequency_hz").as_double();
        wheelbase = this->get_parameter("wheelbase_m").as_double();
        speedMin = this->get_parameter("speed_min").as_double();
        speedMax = this->get_parameter("speed_max").as_double();
        latAccelMax = this->get_parameter("a_lat_max").as_double();
        longAccelMax = this->get_parameter("a_accel_max").as_double();
        longBrakeMax = this->get_parameter("a_brake_max").as_double();
    }
    catch (const rclcpp::ParameterTypeException& ex)
    {
        RCLCPP_ERROR(this->get_logger(), "Parameter type error: %s", ex.what());
    }
}

void PurePursuit::loadWaypointsFromCSV(void)
{
    std::ifstream inputFile(waypointsFilePath);

    if (!inputFile.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "Could not open specified file for waypoints : '%s'", waypointsFilePath.c_str());
        return;
    }

    if (inputFile.peek() == std::ifstream::traits_type::eof())
    {
        RCLCPP_ERROR(this->get_logger(), "Specified file containing waypoints is empty : '%s'", waypointsFilePath.c_str());
        RCLCPP_INFO(this->get_logger(), "Make sure the waypoints file exists at: %s", waypointsFilePath.c_str());
        return;
    }

    std::string line;

    while (std::getline(inputFile, line))
    {
        std::stringstream ss(line);

        std::string xPos;
        std::string yPos;

        std::getline(ss, xPos, ',');
        std::getline(ss, yPos, ',');

        geometry_msgs::msg::PoseStamped poseStamped;
        poseStamped.pose.position.x = std::stod(xPos);
        poseStamped.pose.position.y = std::stod(yPos);
        poseStamped.pose.orientation.w = 1.0;  // Neutral orientation

        poseStamped.header.frame_id = "map";
        poseStamped.header.stamp = this->now();

        waypoint_t pointRead = {poseStamped, 0.f};
        _waypoints.push_back(pointRead);
    }
    inputFile.close();
}

void PurePursuit::calculateSpeed(void)
{
    // Populate waypoints vector with speed values
    size_t n = _waypoints.size();

    if (n == 0)
        return;

    std::vector<double> x(n), y(n);
    std::vector<double> ds(n), dx(n), dy(n), ddx(n), ddy(n), kappa(n);
    std::vector<double> v_curve(n), v(n);

    auto wrap = [&](int i)
    {
        return (i + n) % n;
    };

    // Extract positions
    for (size_t i = 0; i < n; i++)
    {
        x[i] = _waypoints[i].point.pose.position.x;
        y[i] = _waypoints[i].point.pose.position.y;
    }

    if (n < 100)
    {
        for (size_t i = 0; i < n; i++)
        {
            _waypoints[i].speed = speedMin;
        }
        return;
    }

    // ds
    for (size_t i = 0; i < n; i++)
    {
        double dx_ = x[wrap(i + 1)] - x[i];
        double dy_ = y[wrap(i + 1)] - y[i];
        ds[i] = std::max(std::hypot(dx_, dy_), EPS);
    }

    // curvature
    for (size_t i = 0; i < n; i++)
    {
        dx[i] = 0.5 * (x[wrap(i + 1)] - x[wrap(i - 1)]);
        dy[i] = 0.5 * (y[wrap(i + 1)] - y[wrap(i - 1)]);
        ddx[i] = x[wrap(i + 1)] - 2.0 * x[i] + x[wrap(i - 1)];
        ddy[i] = y[wrap(i + 1)] - 2.0 * y[i] + y[wrap(i - 1)];

        double num = std::abs(dx[i] * ddy[i] - dy[i] * ddx[i]);

        double tmp = dx[i] * dx[i] + dy[i] * dy[i];
        double den = tmp * std::sqrt(tmp) + EPS;

        kappa[i] = num / den;
    }

    // curvature speed limit
    for (size_t i = 0; i < n; i++)
    {
        double vtmp = std::sqrt(latAccelMax / std::max(kappa[i], EPS));
        v_curve[i] = std::clamp(vtmp, (double)speedMin, (double)speedMax);
    }

    // anchor (slowest point)
    size_t anchor = std::min_element(v_curve.begin(), v_curve.end()) - v_curve.begin();

    // roll
    std::vector<double> v_roll(n), ds_roll(n);
    for (size_t i = 0; i < n; i++)
    {
        v_roll[i] = v_curve[(i + anchor) % n];
        ds_roll[i] = ds[(i + anchor) % n];
    }

    v = v_roll;

    // forward pass (accel)
    for (size_t i = 1; i < n; i++)
    {
        double v_allow = std::sqrt(std::max(v[i - 1] * v[i - 1] + 2.0 * longAccelMax * ds_roll[i - 1], 0.0));
        v[i] = std::min(v[i], v_allow);
    }

    // backward pass (brake)
    for (int i = n - 2; i >= 0; i--)
    {
        double v_allow = std::sqrt(std::max(v[i + 1] * v[i + 1] + 2.0 * longBrakeMax * ds_roll[i], 0.0));
        v[i] = std::min(v[i], v_allow);
    }

    // unroll + assign
    for (size_t i = 0; i < n; i++)
    {
        double v_final = std::clamp(v[i], (double)speedMin, (double)speedMax);
        size_t idx = (i + anchor) % n;
        _waypoints[idx].speed = static_cast<float>(v_final);
    }
}

void PurePursuit::initRosElements(void)
{
    const auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)).best_effort();

    _loopTimer = this->create_wall_timer(std::chrono::milliseconds(static_cast<size_t>(1000 / loopFrequency)),
                                         [this](void)
                                         {
                                             this->CB_mainDecisionLoop();
                                         });

    _positionSubscriber = this->create_subscription<nav_msgs::msg::Odometry>(positionTopic,
                                                                             odom_qos,
                                                                             [this](const nav_msgs::msg::Odometry& msg)
                                                                             {
                                                                                 this->CB_positionSubscriber(msg);
                                                                             });

    _costmapSubscriber = this->create_subscription<nav_msgs::msg::OccupancyGrid>(costmapTopic,
                                                                                 rclcpp::SensorDataQoS(),
                                                                                 [this](const nav_msgs::msg::OccupancyGrid& msg)
                                                                                 {
                                                                                     this->CB_costmapSubscriber(msg);
                                                                                 });

    _driveCmdPublisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(driveCmdTopic, DEFAULT_QOS);
    _targetWaypointPublisher = this->create_publisher<geometry_msgs::msg::PointStamped>(targetWaypointTopic, DEFAULT_QOS);
    _trajectoryRiskPublisher = this->create_publisher<std_msgs::msg::Float32>(trajectoryRiskTopic, DEFAULT_QOS);
    _riskPathPublisher = this->create_publisher<nav_msgs::msg::Path>(riskPathTopic, DEFAULT_QOS);

    _errorPublisher = this->create_publisher<arcus_msgs::msg::ErrorCode>(errorTopic, 10);

    _heartbeatTimer = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&PurePursuit::heartbeat, this));
}

void PurePursuit::initParamCallbackHandle(void)
{
    // Callback allowing dynamic parameter updates at runtime with GUI
    _paramCallbackHandle = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params)
        {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;

            bool needsSpeedRecalc = false;

            for (const auto& param : params)
            {
                const std::string& name = param.get_name();

                if (name == "max_lookahead_distance_m")
                {
                    maxLookahead = param.as_double();
                }
                else if (name == "min_lookahead_distance_m")
                {
                    minLookahead = param.as_double();
                }
                else if (name == "lookahead_distance_gain")
                {
                    lookaheadGain = param.as_double();
                }
                else if (name == "risk_lookahead_gain")
                {
                    riskLookaheadGain = param.as_double();
                }
                else if (name == "ttc_decay_rate")
                {
                    ttcDecayRate = param.as_double();
                }
                else if (name == "min_ttc_speed_mps")
                {
                    ttcMinSpeed = param.as_double();
                }
                else if (name == "reloc_distance_m")
                {
                    relocalizeDistance = param.as_double();
                }
                else if (name == "max_lookahead_fraction_of_path")
                {
                    maxLookaheadFraction = param.as_double();
                }
                else if (name == "wheelbase_m")
                {
                    wheelbase = param.as_double();
                }
                else if (name == "speed_min")
                {
                    speedMin = param.as_double();
                    needsSpeedRecalc = true;
                }
                else if (name == "speed_max")
                {
                    speedMax = param.as_double();
                    needsSpeedRecalc = true;
                }
                else if (name == "a_lat_max")
                {
                    latAccelMax = param.as_double();
                    needsSpeedRecalc = true;
                }
                else if (name == "a_accel_max")
                {
                    longAccelMax = param.as_double();
                    needsSpeedRecalc = true;
                }
                else if (name == "a_brake_max")
                {
                    longBrakeMax = param.as_double();
                    needsSpeedRecalc = true;
                }
                else if (name == "debug")
                {
                    debug = param.as_bool();
                }

                RCLCPP_INFO(this->get_logger(), "Parameter updated: %s", name.c_str());
            }

            if (needsSpeedRecalc)
            {
                RCLCPP_INFO(this->get_logger(), "Speed profile parameters changed, recalculating...");
                this->calculateSpeed();
            }

            return result;
        });
}

void PurePursuit::heartbeat()
{
    arcus_msgs::msg::ErrorCode error_msg;
    error_msg.source = arcus_msgs::msg::ErrorCode::PURE_PURSUIT;
    error_msg.header.stamp = this->now();
    if (!_recoveryActive)
    {
        error_msg.error_code = arcus_msgs::msg::ErrorCode::OK;
    }
    else
    {
        error_msg.error_code = arcus_msgs::msg::ErrorCode::EMERGENCY_BRAKE;
    }
    this->_errorPublisher->publish(error_msg);
}

double PurePursuit::clipLookaheadDistance(double lookAheadDistance_) const
{
    double clippedLookaheadDistance = lookAheadDistance_;
    if (lookAheadDistance_ < minLookahead)
    {
        clippedLookaheadDistance = minLookahead;
    }
    else if (lookAheadDistance_ > maxLookahead)
    {
        clippedLookaheadDistance = maxLookahead;
    }

    return clippedLookaheadDistance;
}

PurePursuit::waypoint_t PurePursuit::getLookaheadPoint(const double lookAheadDistance_)
{
    //!!!!!!!!!!!!!!!!
    double minDistanceDifference = std::numeric_limits<double>::max();
    double minDistance = std::numeric_limits<double>::max();
    size_t bestIndex = 0;

    size_t maxIterationCount = static_cast<size_t>(_waypoints.size() * maxLookaheadFraction);

    if (!_firstTargetWaypointLocked)
    {
        maxIterationCount = _waypoints.size();
        _firstTargetWaypointLocked = true;
    }

    for (size_t i = 0; i < maxIterationCount; i++)
    {
        size_t wrappingIndex = (i + _previousWaypointIndex) % _waypoints.size();

        double dx = _waypoints[wrappingIndex].point.pose.position.x - _currentX;
        double dy = _waypoints[wrappingIndex].point.pose.position.y - _currentY;
        double distance = std::sqrt(dx * dx + dy * dy);
        double distanceDifference = std::abs(distance - lookAheadDistance_);

        if (distance < minDistance)
        {
            minDistance = distance;
        }

        if (distanceDifference <= minDistanceDifference)
        {
            minDistanceDifference = distanceDifference;
            bestIndex = wrappingIndex;
        }
    }

    if (minDistance > relocalizeDistance)
    {
        minDistanceDifference = std::numeric_limits<double>::max();
        bestIndex = 0;
        for (size_t i = 0; i < _waypoints.size(); i++)
        {
            double dx = _waypoints[i].point.pose.position.x - _currentX;
            double dy = _waypoints[i].point.pose.position.y - _currentY;
            double distance = std::sqrt(dx * dx + dy * dy);
            double distanceDifference = std::abs(distance - lookAheadDistance_);

            if (distanceDifference <= minDistanceDifference)
            {
                minDistanceDifference = distanceDifference;
                bestIndex = i;
            }
        }
    }

    _previousWaypointIndex = bestIndex;
    return _waypoints.at(bestIndex);
}

void PurePursuit::evaluatePointRisk(double x, double y, double cumulativeDistance, double& riskMax)
{
    // Transform point to costmap grid coordinates
    int gridX = static_cast<int>(std::round((x - _costmapOriginX) / _costmapResolution));
    int gridY = static_cast<int>(std::round((y - _costmapOriginY) / _costmapResolution));

    // Check bounds
    if (gridX >= 0 && gridX < _costmapWidth && gridY >= 0 && gridY < _costmapHeight)
    {
        int idx = gridY * _costmapWidth + gridX;
        if (idx >= 0 && idx < static_cast<int>(_costmapData.size()))
        {
            int8_t cost = _costmapData[idx];
            if (cost >= 0)  // -1 = unknown
            {
                double closingSpeed = std::max(std::abs(_currentSpeed), ttcMinSpeed);
                double ttc = cumulativeDistance / closingSpeed;
                double weight = std::exp(-ttc / ttcDecayRate);
                double weightedRisk = weight * static_cast<double>(cost);
                riskMax = std::max(riskMax, weightedRisk);
            }
        }
    }
}

double PurePursuit::calculateTrajectoryRisk(double riskLookaheadDistance)
{
    std::lock_guard<std::mutex> lock(_costmapMutex);

    if (_costmapData.empty() || _costmapWidth == 0 || _costmapHeight == 0)
    {
        return 0.0;  // No costmap data yet
    }

    if (_costmapFrameId.empty())
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(),
                             *this->get_clock(),
                             5000,
                             "Costmap frame_id is empty; cannot transform risk samples.");
        return 0.0;
    }

    geometry_msgs::msg::TransformStamped mapToCostmap;
    try
    {
        mapToCostmap = _tfBuffer.lookupTransform(_costmapFrameId, "map", tf2::TimePointZero);
    }
    catch (const tf2::TransformException& ex)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(),
                             *this->get_clock(),
                             2000,
                             "TF lookup failed (map -> %s): %s",
                             _costmapFrameId.c_str(),
                             ex.what());
        return 0.0;
    }

    // Clear and populate risk path waypoints only in debug mode
    _riskPathWaypoints.clear();
    geometry_msgs::msg::PoseStamped currentPose;

    if (debug)
    {
        // Add starting position (current vehicle position)
        currentPose.header.frame_id = "map";
        currentPose.header.stamp = this->get_clock()->now();
        currentPose.pose.position.x = _currentX;
        currentPose.pose.position.y = _currentY;
        currentPose.pose.position.z = 0.0;
        currentPose.pose.orientation.w = 1.0;
        _riskPathWaypoints.push_back(currentPose);
    }

    // Check waypoints along the raceline within lookahead distance
    double riskMax = 0.0;
    double cumulativeDistance = 0.0;
    double prevX = _currentX;
    double prevY = _currentY;

    for (size_t i = 0; i < _waypoints.size(); i++)
    {
        size_t waypointIdx = (_previousWaypointIndex + i) % _waypoints.size();
        const auto& waypoint = _waypoints[waypointIdx];

        double wpX = waypoint.point.pose.position.x;
        double wpY = waypoint.point.pose.position.y;

        double dx = wpX - prevX;
        double dy = wpY - prevY;
        double segmentLength = std::sqrt(dx * dx + dy * dy);

        double stepDistance = riskInterpolationStep;
        int numSamples = std::max(1, static_cast<int>(std::ceil(segmentLength / stepDistance)));
        double sampleDelta = segmentLength / static_cast<double>(numSamples);

        for (int sampleIndex = 1; sampleIndex <= numSamples; sampleIndex++)
        {
            double t = static_cast<double>(sampleIndex) / static_cast<double>(numSamples);
            double sampleX = prevX + t * (wpX - prevX);
            double sampleY = prevY + t * (wpY - prevY);
            cumulativeDistance += sampleDelta;

            if (cumulativeDistance > riskLookaheadDistance)
            {
                break;
            }

            // Add sampled point to risk path (debug only)
            if (debug)
            {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = "map";
                pose.header.stamp = this->get_clock()->now();
                pose.pose.position.x = sampleX;
                pose.pose.position.y = sampleY;
                pose.pose.position.z = 0.0;
                pose.pose.orientation.w = 1.0;
                _riskPathWaypoints.push_back(pose);
            }

            geometry_msgs::msg::PointStamped mapPoint;
            geometry_msgs::msg::PointStamped costmapPoint;
            mapPoint.header.frame_id = "map";
            mapPoint.point.x = sampleX;
            mapPoint.point.y = sampleY;
            mapPoint.point.z = 0.0;
            tf2::doTransform(mapPoint, costmapPoint, mapToCostmap);

            this->evaluatePointRisk(costmapPoint.point.x, costmapPoint.point.y, cumulativeDistance, riskMax);
        }

        if (cumulativeDistance > riskLookaheadDistance)
        {
            break;
        }

        prevX = wpX;
        prevY = wpY;
    }

    // Return max weighted occupancy (0-100)
    if (cumulativeDistance <= 1e-9)
    {
        return 0.0;
    }

    return std::clamp(riskMax, 0.0, 100.0);
}