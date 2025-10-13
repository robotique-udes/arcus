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
    this->initRosElements();
    this->loadWaypointsFromCSV();
    // Reverse waypoints if they are in the wrong order
    // We will need to implement a more robust method of checking this in the future
    std::reverse(_waypoints.begin(), _waypoints.end());
}

void PurePursuit::CB_publishDriveCmd(void)
{
    _lookAheadDistance = LOOKAHEAD_DISTANCE_GAIN * _currentSpeed;
    this->clipLookaheadDistance(_lookAheadDistance);

    geometry_msgs::msg::PoseStamped lookaheadPoint = this->getLookaheadPoint(_currentX, _currentY, _lookAheadDistance);
    this->CB_publishTargetWaypoint(lookaheadPoint);  // For visualization purposes only

    RCLCPP_INFO(this->get_logger(),
                "Lookahead Point: (%.2f, %.2f), Current Position: (%.2f, %.2f), Lookahead Distance: %.2f",
                lookaheadPoint.pose.position.x,
                lookaheadPoint.pose.position.y,
                _currentX,
                _currentY,
                _lookAheadDistance);
    double dx = lookaheadPoint.pose.position.x - _currentX;
    double dy = lookaheadPoint.pose.position.y - _currentY;

    double lookaheadDistanceActual = std::sqrt(dx * dx + dy * dy);

    // Transform (dx, dy) from world to vehicle local frame
    double localX = std::cos(-_currentYaw) * dx - std::sin(-_currentYaw) * dy;
    double localY = std::sin(-_currentYaw) * dx + std::cos(-_currentYaw) * dy;

    _alpha = std::atan2(localY, localX);

    double steeringAngle = std::atan2(2.0 * WHEELBASE_M * std::sin(_alpha), lookaheadDistanceActual);

    ackermann_msgs::msg::AckermannDriveStamped driveCmd;
    driveCmd.header.stamp = this->now();
    driveCmd.header.frame_id = "base_link";
    driveCmd.drive.steering_angle = steeringAngle;
    driveCmd.drive.speed = this->speedFromWheelAngle(steeringAngle);

    _driveCmdPublisher->publish(driveCmd);
}

void PurePursuit::CB_positionSubscriber(const nav_msgs::msg::Odometry& msg)
{
    _currentX = msg.pose.pose.position.x;
    _currentY = msg.pose.pose.position.y;
    _currentSpeed = msg.twist.twist.linear.x;

    // converting quaternion to euler angles (yaw)
    double qw = msg.pose.pose.orientation.w;
    double qx = msg.pose.pose.orientation.x;
    double qy = msg.pose.pose.orientation.y;
    double qz = msg.pose.pose.orientation.z;

    _currentYaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
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

void PurePursuit::loadWaypointsFromCSV()
{
    std::ifstream inputFile(WAYPOINTS_CSV_FILE_NAME);
    if (!inputFile.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "Could not open the file - '%s'", WAYPOINTS_CSV_FILE_NAME);
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

        _waypoints.push_back(poseStamped);
    }
    inputFile.close();
}

void PurePursuit::initRosElements(void)
{
    _loopTimer = this->create_wall_timer(std::chrono::milliseconds(static_cast<size_t>(1000 / LOOP_FREQUENCY_HZ)),
                                         [this](void)
                                         {
                                             this->CB_publishDriveCmd();
                                         });

    _positionSubscriber = this->create_subscription<nav_msgs::msg::Odometry>(CURRENT_POSITION_TOPIC,
                                                                             DEFAULT_QOS,
                                                                             [this](const nav_msgs::msg::Odometry& msg)
                                                                             {
                                                                                 this->CB_positionSubscriber(msg);
                                                                             });
    _driveCmdPublisher = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(DRIVE_CMD_TOPIC, DEFAULT_QOS);

    _targetWaypointPublisher = this->create_publisher<geometry_msgs::msg::PointStamped>(TARGET_WAYPOINT_TOPIC, DEFAULT_QOS);
}

void PurePursuit::clipLookaheadDistance(double& _lookaheadDistance)
{
    if (_lookaheadDistance < MIN_LOOKAHEAD_DISTANCE_M)
    {
        _lookaheadDistance = MIN_LOOKAHEAD_DISTANCE_M;
    }
    else if (_lookaheadDistance > MAX_LOOKAHEAD_DISTANCE_M)
    {
        _lookaheadDistance = MAX_LOOKAHEAD_DISTANCE_M;
    }
}

geometry_msgs::msg::PoseStamped PurePursuit::getLookaheadPoint(double currentX, double currentY, double lookAheadDistance)
{
    double minDistanceDifference = std::numeric_limits<double>::max();
    size_t bestIndex = 0;

    size_t maxIterationCount = static_cast<size_t>(_waypoints.size() * MAX_LOOKAHEAD_FRACTION_OF_TRACK);

    if (!_firstTargetLocked)
    {
        maxIterationCount = _waypoints.size();
        _firstTargetLocked = true;
    }

    for (size_t i = 0; i < maxIterationCount; i++)
    {
        size_t wrappingIndex = (i + _previousBestIndex) % _waypoints.size();

        double dx = _waypoints[wrappingIndex].pose.position.x - currentX;
        double dy = _waypoints[wrappingIndex].pose.position.y - currentY;
        double distance = std::sqrt(dx * dx + dy * dy);
        double distanceDifference = std::abs(distance - lookAheadDistance);

        if (distanceDifference <= minDistanceDifference)
        {
            minDistanceDifference = distanceDifference;
            bestIndex = wrappingIndex;
        }
    }
#warning nullopt return to catch error
    _previousBestIndex = bestIndex;
    return _waypoints[bestIndex];
}

double PurePursuit::speedFromWheelAngle(double& wheelAngle)
{
    RCLCPP_INFO(this->get_logger(), "%f", wheelAngle);
    /* Mapping angles where 1 represents straight ahead and a 90 degree turn represents 0.
    This transform will allow us to easily get a higher speed for smaller angles. An exponential
    is also added to increase breaking in tight turns   */

    // a*e^(b*(c-x/(pi/2)))

    double speed = EXPONENTIAL_A_CTE * pow(e, EXPONENTIAL_B_CTE * (EXPONENTIAL_C_CTE - abs(wheelAngle) / (PI / 2)));
    return speed;
}
