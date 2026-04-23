#include "pure_pursuit.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}

PurePursuit::PurePursuit():
    Node("pure_pursuit")
{
    this->handleRosParam();
    this->initRosElements();
    this->loadWaypointsFromCSV();
    this->calculateSpeed();
}

void PurePursuit::CB_publishDriveCmd(void)
{
    if (_waypoints.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "No waypoints loaded. Cannot publish drive command.");
        return;
    }

    double lookAheadDistance = LOOKAHEAD_GAIN * _currentSpeed;
    double clippedLookAheadDistance = this->clipLookaheadDistance(lookAheadDistance);
    double riskLookaheadDistance = RISK_LOOKAHEAD_GAIN * _currentSpeed;

    Waypoint lookaheadPoint = this->getLookaheadPoint(clippedLookAheadDistance);
    this->CB_publishTargetWaypoint(lookaheadPoint.point);  // For visualization purposes only

    // Calculate and publish trajectory risk
    double trajectoryRisk = this->calculateTrajectoryRisk(riskLookaheadDistance);
    std_msgs::msg::Float32 riskMsg;
    riskMsg.data = static_cast<float>(trajectoryRisk);
    _trajectoryRiskPublisher->publish(riskMsg);

    /* RCLCPP_DEBUG(this->get_logger(),
                 "Lookahead Point: (%.2f, %.2f), Current Position: (%.2f, %.2f), Lookahead Distance: %.2f",
                 lookaheadPoint.pose.position.x,
                 lookaheadPoint.pose.position.y,
                 _currentX,
                 _currentY,
                 clippedLookAheadDistance); */

    // Find the actual lookahead distance based on the selected lookahead point

    float targetSpeed = lookaheadPoint.speed;

    double dx = lookaheadPoint.point.pose.position.x - _currentX;
    double dy = lookaheadPoint.point.pose.position.y - _currentY;
    double lookaheadDistanceActual = std::sqrt(dx * dx + dy * dy);

    // Transform (dx, dy) from world to vehicle local frame
    double localX = std::cos(-_currentYaw) * dx - std::sin(-_currentYaw) * dy;
    double localY = std::sin(-_currentYaw) * dx + std::cos(-_currentYaw) * dy;

    double alpha = std::atan2(localY, localX);
    double steeringAngle = std::atan2(2.0 * WHEELBASE_M * std::sin(alpha), lookaheadDistanceActual);

    ackermann_msgs::msg::AckermannDriveStamped driveCmd;
    driveCmd.header.stamp = this->now();
    driveCmd.header.frame_id = "base_link";
    driveCmd.drive.steering_angle = steeringAngle;

    if (_recoveryActive)
    {
        RCLCPP_DEBUG(this->get_logger(),
                     "Recovery Active: steeringAngle=%.4f, threshold=%.4f",
                     std::abs(steeringAngle),
                     RECOVERY_DISENGAGE_STEER_RAD);
        if (std::abs(steeringAngle) < RECOVERY_DISENGAGE_STEER_RAD)
        {
            _recoveryActive = false;
            RCLCPP_INFO(this->get_logger(), "Recovery Mode DISENGAGED: steering angle now below threshold");
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(),
                         "Recovery Mode: Sending reverse command (speed=%.2f, steeringAngle=%.4f)",
                         -RECOVERY_REVERSE_SPEED_MS,
                         _recoverySteeringAngle);
            driveCmd.drive.steering_angle = -_recoverySteeringAngle;
            driveCmd.drive.speed = -RECOVERY_REVERSE_SPEED_MS;
            _driveCmdPublisher->publish(driveCmd);
            return;
        }
    }

    if (_recoveryArmed && (_currentSpeed < RECOVERY_TRIGGER_SPEED_MS))
    {
        RCLCPP_WARN(this->get_logger(),
                    "Recovery Mode ACTIVATED: carHasEverMoved=%d, currentSpeed=%.4f, triggerSpeed=%.4f",
                    _carHasEverMoved,
                    _currentSpeed,
                    RECOVERY_TRIGGER_SPEED_MS);
        _recoveryActive = true;
        _recoveryArmed = false;
        _recoverySteeringAngle = steeringAngle;

        RCLCPP_INFO(this->get_logger(),
                    "Sending initial recovery reverse command (speed=%.2f, steeringAngle=%.4f)",
                    -RECOVERY_REVERSE_SPEED_MS,
                    _recoverySteeringAngle);
        driveCmd.drive.steering_angle = _recoverySteeringAngle;
        driveCmd.drive.speed = -RECOVERY_REVERSE_SPEED_MS;
        _driveCmdPublisher->publish(driveCmd);
        return;
    }

    driveCmd.drive.speed = targetSpeed;  // SMARTER WA

    _driveCmdPublisher->publish(driveCmd);
}

void PurePursuit::CB_positionSubscriber(const nav_msgs::msg::Odometry& msg)
{
    _currentX = msg.pose.pose.position.x;
    _currentY = msg.pose.pose.position.y;

    // Orientation (quaternion -> yaw)
    const auto& q = msg.pose.pose.orientation;

    _currentYaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    _currentSpeed = msg.twist.twist.linear.x;

    // Track if the car has ever moved
    if (_currentSpeed > 0.0 && !_carHasEverMoved)
    {
        _carHasEverMoved = true;
        RCLCPP_INFO(this->get_logger(), "Car has started moving for the first time! Speed: %.4f m/s", _currentSpeed);
    }

    if (_carHasEverMoved && (_currentSpeed > RECOVERY_REARM_SPEED_MS))
    {
        if (!_recoveryArmed)
        {
            _recoveryArmed = true;
            RCLCPP_DEBUG(this->get_logger(),
                         "Recovery Mode ARMED: speed (%.4f) > rearm threshold (%.4f)",
                         _currentSpeed,
                         RECOVERY_REARM_SPEED_MS);
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

void PurePursuit::handleRosParam(void)
{
    this->declare_parameter<std::string>("waypoints_file_path", DEFAULT_WAYPOINTS_CSV_FILE_NAME);
    this->declare_parameter<std::string>("position_topic", DEFAULT_POSITION_TOPIC);
    this->declare_parameter<std::string>("drive_command_topic", DEFAULT_DRIVE_CMD_TOPIC);
    this->declare_parameter<std::string>("target_waypoint_topic", TARGET_WAYPOINT_TOPIC);
    this->declare_parameter<std::string>("costmap_topic", DEFAULT_COSTMAP_TOPIC);
    this->declare_parameter<std::string>("trajectory_risk_topic", DEFAULT_TRAJECTORY_RISK_TOPIC);
    this->declare_parameter<std::string>("error_topic", DEFAULT_ERROR_TOPIC);

    this->declare_parameter("max_lookahead_distance_m", MAX_LOOKAHEAD_M);
    this->declare_parameter("min_lookahead_distance_m", MIN_LOOKAHEAD_M);
    this->declare_parameter("lookahead_distance_gain", LOOKAHEAD_GAIN);
    this->declare_parameter("risk_lookahead_gain", RISK_LOOKAHEAD_GAIN);
    this->declare_parameter("ttc_decay_rate", TTC_DECAY_RATE);
    this->declare_parameter("min_ttc_speed_mps", MIN_TTC_SPEED_MS);
    this->declare_parameter("max_lookahead_fraction_of_path", MAX_LOOKAHEAD_FRACTION);
    this->declare_parameter("loop_frequency_hz", LOOP_FREQUENCY_HZ);
    this->declare_parameter("wheelbase_m", WHEELBASE_M);
    this->declare_parameter("speed_min", SPEED_MIN);
    this->declare_parameter("speed_max", SPEED_MAX);
    this->declare_parameter("a_lat_max", A_LAT_MAX);
    this->declare_parameter("a_accel_max", A_ACCEL_MAX);
    this->declare_parameter("a_brake_max", A_BRAKE_MAX);
    this->declare_parameter("speed_eps", SPEED_EPS);

    _waypointsFilePath = this->get_parameter("waypoints_file_path").as_string();
    _positionTopic = this->get_parameter("position_topic").as_string();
    _driveCmdTopic = this->get_parameter("drive_command_topic").as_string();
    _targetWaypointTopic = this->get_parameter("target_waypoint_topic").as_string();
    _costmapTopic = this->get_parameter("costmap_topic").as_string();
    _trajectoryRiskTopic = this->get_parameter("trajectory_risk_topic").as_string();
    _errorTopic = this->get_parameter("error_topic").as_string();

    try
    {
        MAX_LOOKAHEAD_M = this->get_parameter("max_lookahead_distance_m").as_double();
        MIN_LOOKAHEAD_M = this->get_parameter("min_lookahead_distance_m").as_double();
        LOOKAHEAD_GAIN = this->get_parameter("lookahead_distance_gain").as_double();
        RISK_LOOKAHEAD_GAIN = this->get_parameter("risk_lookahead_gain").as_double();
        TTC_DECAY_RATE = this->get_parameter("ttc_decay_rate").as_double();
        MIN_TTC_SPEED_MS = this->get_parameter("min_ttc_speed_mps").as_double();
        MAX_LOOKAHEAD_FRACTION = this->get_parameter("max_lookahead_fraction_of_path").as_double();
        LOOP_FREQUENCY_HZ = this->get_parameter("loop_frequency_hz").as_double();
        WHEELBASE_M = this->get_parameter("wheelbase_m").as_double();
        SPEED_MIN = this->get_parameter("speed_min").as_double();
        SPEED_MAX = this->get_parameter("speed_max").as_double();
        A_LAT_MAX = this->get_parameter("a_lat_max").as_double();
        A_ACCEL_MAX = this->get_parameter("a_accel_max").as_double();
        A_BRAKE_MAX = this->get_parameter("a_brake_max").as_double();
        SPEED_EPS = this->get_parameter("speed_eps").as_double();
    }
    catch (const rclcpp::ParameterTypeException& ex)
    {
        RCLCPP_ERROR(this->get_logger(), "Parameter type error: %s", ex.what());
    }

    RCLCPP_INFO(this->get_logger(), "--- Pure Pursuit Parameters ---");
    RCLCPP_INFO(this->get_logger(), "  max_lookahead_distance_m:       %.3f", MAX_LOOKAHEAD_M);
    RCLCPP_INFO(this->get_logger(), "  min_lookahead_distance_m:       %.3f", MIN_LOOKAHEAD_M);
    RCLCPP_INFO(this->get_logger(), "  lookahead_distance_gain:        %.3f", LOOKAHEAD_GAIN);
    RCLCPP_INFO(this->get_logger(), "  risk_lookahead_gain:            %.3f", RISK_LOOKAHEAD_GAIN);
    RCLCPP_INFO(this->get_logger(), "  ttc_decay_rate:                 %.3f", TTC_DECAY_RATE);
    RCLCPP_INFO(this->get_logger(), "  min_ttc_speed_mps:              %.3f", MIN_TTC_SPEED_MS);
    RCLCPP_INFO(this->get_logger(), "  max_lookahead_fraction_of_path: %.3f", MAX_LOOKAHEAD_FRACTION);
    RCLCPP_INFO(this->get_logger(), "  loop_frequency_hz:              %.1f", LOOP_FREQUENCY_HZ);
    RCLCPP_INFO(this->get_logger(), "  wheelbase_m:                    %.3f", WHEELBASE_M);
    RCLCPP_INFO(this->get_logger(), "  speed_min:                      %.3f", SPEED_MIN);
    RCLCPP_INFO(this->get_logger(), "  speed_max:                      %.3f", SPEED_MAX);
    RCLCPP_INFO(this->get_logger(), "  a_lat_max:                      %.3f", A_LAT_MAX);
    RCLCPP_INFO(this->get_logger(), "  a_accel_max:                    %.3f", A_ACCEL_MAX);
    RCLCPP_INFO(this->get_logger(), "  a_brake_max:                    %.3f", A_BRAKE_MAX);
    RCLCPP_INFO(this->get_logger(), "  speed_eps:                      %.2e", SPEED_EPS);
    RCLCPP_INFO(this->get_logger(), "-------------------------------");
    RCLCPP_INFO(this->get_logger(), "Waypoints file path: %s", _waypointsFilePath.c_str());
    RCLCPP_INFO(this->get_logger(), "Position topic: %s", _positionTopic.c_str());
    RCLCPP_INFO(this->get_logger(), "Drive command topic: %s", _driveCmdTopic.c_str());
    RCLCPP_INFO(this->get_logger(), "Target waypoint topic: %s", _targetWaypointTopic.c_str());
    RCLCPP_INFO(this->get_logger(), "Costmap topic: %s", _costmapTopic.c_str());
    RCLCPP_INFO(this->get_logger(), "Trajectory risk topic: %s", _trajectoryRiskTopic.c_str());
    RCLCPP_INFO(this->get_logger(), "Error topic: %s", _errorTopic.c_str());
}

void PurePursuit::loadWaypointsFromCSV(void)
{
    std::ifstream inputFile(_waypointsFilePath);

    if (!inputFile.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "Could not open specified file for waypoints : '%s'", _waypointsFilePath.c_str());
        return;
    }

    if (inputFile.peek() == std::ifstream::traits_type::eof())
    {
        RCLCPP_ERROR(this->get_logger(), "Specified file containing waypoints is empty : '%s'", _waypointsFilePath.c_str());
        RCLCPP_INFO(this->get_logger(), "Make sure the waypoints file exists at: %s", _waypointsFilePath.c_str());
        return;
    }

    std::string line;

    while (std::getline(inputFile, line))
    {
        std::stringstream ss(line);

        std::string xPos;
        std::string yPos;
        // std::string speed;

        std::getline(ss, xPos, ',');
        std::getline(ss, yPos, ',');
        // std::getline(ss, speed, ',');

        geometry_msgs::msg::PoseStamped poseStamped;
        poseStamped.pose.position.x = std::stod(xPos);
        poseStamped.pose.position.y = std::stod(yPos);
        poseStamped.pose.orientation.w = 1.0;  // Neutral orientation

        poseStamped.header.frame_id = "map";
        poseStamped.header.stamp = this->now();

        // double speedDouble = std::stod(speed);

        Waypoint pointRead = {poseStamped, 0.f};
        _waypoints.push_back(pointRead);
    }
    inputFile.close();
}

void PurePursuit::calculateSpeed(void)
{
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

    // If too small → constant speed
    if (n < 100)
    {
        for (size_t i = 0; i < n; i++)
        {
            _waypoints[i].speed = SPEED_MIN;
        }
        return;
    }

    // ds
    for (size_t i = 0; i < n; i++)
    {
        double dx_ = x[wrap(i + 1)] - x[i];
        double dy_ = y[wrap(i + 1)] - y[i];
        ds[i] = std::max(std::hypot(dx_, dy_), SPEED_EPS);
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
        double den = tmp * std::sqrt(tmp) + SPEED_EPS;

        kappa[i] = num / den;
    }

    // curvature speed limit
    for (size_t i = 0; i < n; i++)
    {
        double vtmp = std::sqrt(A_LAT_MAX / std::max(kappa[i], SPEED_EPS));
        v_curve[i] = std::clamp(vtmp, (double)SPEED_MIN, (double)SPEED_MAX);
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
        double v_allow = std::sqrt(std::max(v[i - 1] * v[i - 1] + 2.0 * A_ACCEL_MAX * ds_roll[i - 1], 0.0));
        v[i] = std::min(v[i], v_allow);
    }

    // backward pass (brake)
    for (int i = n - 2; i >= 0; i--)
    {
        double v_allow = std::sqrt(std::max(v[i + 1] * v[i + 1] + 2.0 * A_BRAKE_MAX * ds_roll[i], 0.0));
        v[i] = std::min(v[i], v_allow);
    }

    // unroll + assign
    for (size_t i = 0; i < n; i++)
    {
        double v_final = std::clamp(v[i], (double)SPEED_MIN, (double)SPEED_MAX);
        size_t idx = (i + anchor) % n;
        _waypoints[idx].speed = static_cast<float>(v_final);
        /*if (idx % 10 == 0) {
        RCLCPP_INFO(this->get_logger(),
                    "Waypoint %zu: (%.2f, %.2f), Curvature: %.4f, Speed: %.2f",
                    idx, x[idx], y[idx], kappa[idx], v_final);
        }*/
    }
}

void PurePursuit::initRosElements(void)
{
    const auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(DEFAULT_QOS)).best_effort();

    _loopTimer = this->create_wall_timer(std::chrono::milliseconds(static_cast<size_t>(1000 / LOOP_FREQUENCY_HZ)),
                                         [this](void)
                                         {
                                             this->CB_publishDriveCmd();
                                         });

    _positionSubscriber = this->create_subscription<nav_msgs::msg::Odometry>(_positionTopic,
                                                                             odom_qos,
                                                                             [this](const nav_msgs::msg::Odometry& msg)
                                                                             {
                                                                                 this->CB_positionSubscriber(msg);
                                                                             });

    _costmapSubscriber = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        _costmapTopic,
        rclcpp::SensorDataQoS(),
        [this](const nav_msgs::msg::OccupancyGrid& msg)
        {
            this->CB_costmapSubscriber(msg);
        });

    _driveCmdPublisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(_driveCmdTopic, DEFAULT_QOS);
    _targetWaypointPublisher = this->create_publisher<geometry_msgs::msg::PointStamped>(_targetWaypointTopic, DEFAULT_QOS);
    _trajectoryRiskPublisher = this->create_publisher<std_msgs::msg::Float32>(_trajectoryRiskTopic, DEFAULT_QOS);

    _errorPublisher = this->create_publisher<arcus_msgs::msg::ErrorCode>(_errorTopic, 10);

    _heartbeatTimer = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&PurePursuit::heartbeat, this));
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
    if (lookAheadDistance_ < MIN_LOOKAHEAD_M)
    {
        clippedLookaheadDistance = MIN_LOOKAHEAD_M;
    }
    else if (lookAheadDistance_ > MAX_LOOKAHEAD_M)
    {
        clippedLookaheadDistance = MAX_LOOKAHEAD_M;
    }

    return clippedLookaheadDistance;
}

PurePursuit::Waypoint PurePursuit::getLookaheadPoint(const double lookAheadDistance_)
{
    double minDistanceDifference = std::numeric_limits<double>::max();
    size_t bestIndex = 0;

    size_t maxIterationCount = static_cast<size_t>(_waypoints.size() * MAX_LOOKAHEAD_FRACTION);

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

        if (distanceDifference <= minDistanceDifference)
        {
            minDistanceDifference = distanceDifference;
            bestIndex = wrappingIndex;
        }
    }

    _previousWaypointIndex = bestIndex;
    return _waypoints.at(bestIndex);
}

double PurePursuit::calculateTrajectoryRisk(double riskLookaheadDistance)
{
    std::lock_guard<std::mutex> lock(_costmapMutex);

    if (_costmapData.empty() || _costmapWidth == 0 || _costmapHeight == 0)
    {
        return 0.0;  // No costmap data yet
    }

    // Check waypoints along the raceline within lookahead distance
    double riskSum = 0;
    int waypointsChecked = 0;
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

        cumulativeDistance += segmentLength;

        if (cumulativeDistance > riskLookaheadDistance)
        {
            break;
        }

        // Transform waypoint to costmap grid coordinates
        int gridX = static_cast<int>(std::round((wpX - _costmapOriginX) / _costmapResolution));
        int gridY = static_cast<int>(std::round((wpY - _costmapOriginY) / _costmapResolution));

        // Check bounds
        if (gridX >= 0 && gridX < _costmapWidth && gridY >= 0 && gridY < _costmapHeight)
        {
            int idx = gridY * _costmapWidth + gridX;
            if (idx >= 0 && idx < static_cast<int>(_costmapData.size()))
            {
                int8_t cost = _costmapData[idx];
                if (cost >= 0)  // -1 = unknown
                {
                    // Time-to-collision weighting: same distance gets higher risk at higher speed.
                    double closingSpeed = std::max(std::abs(_currentSpeed), MIN_TTC_SPEED_MS);
                    double ttc = cumulativeDistance / closingSpeed;
                    double weight = std::exp(-TTC_DECAY_RATE * ttc);
                    riskSum += weight * static_cast<double>(cost) * segmentLength;
                    waypointsChecked++;
                }
            }
        }

        prevX = wpX;
        prevY = wpY;
    }

    // Return average occupancy (0-100)
    if (waypointsChecked > 0)
    {
        return static_cast<double>(riskSum) / waypointsChecked;
    }

    return 0.0;
}