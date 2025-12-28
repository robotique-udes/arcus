#ifndef PURE_PURSUIT_HPP
#define PURE_PURSUIT_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "arcus_msgs/msg/error_code.hpp"

#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <algorithm>

class PurePursuit : public rclcpp::Node
{
    // Below are parameters that need to be tweaked for better performance
    static constexpr double MAX_LOOKAHEAD_DISTANCE_M = 3.5;
    static constexpr double MIN_LOOKAHEAD_DISTANCE_M = 0.4;
    static constexpr double LOOKAHEAD_DISTANCE_GAIN = 0.25;
    static constexpr double MAX_LOOKAHEAD_FRACTION_OF_PATH = 0.05;
    static constexpr double LOOP_FREQUENCY_HZ = 20.0;
    static constexpr double WHEELBASE_M = 0.325;      // Distance between front and rear axles
    static constexpr double CONSTANT_SPEED_MS = 5.0;  // Constant speed in m/s

    // Topic, input file names and QoS
    static constexpr const uint8_t DEFAULT_QOS = 1;
    static constexpr const char* DEFAULT_DRIVE_CMD_TOPIC = "/drive";
    static constexpr const char* TARGET_WAYPOINT_TOPIC = "/target_waypoint";
    static constexpr const char* DEFAULT_POSITION_TOPIC = "/ego_racecar/odom";
    static constexpr const char* DEFAULT_WAYPOINTS_CSV_FILE_NAME = "/sim_ws/src/arcus/resources/waypoints/waypoints.csv";

    // Mathematical constants
    static constexpr const double PI = 3.14159;
    static constexpr const double e = 2.71828;

  public:
    PurePursuit();

  private:
    void CB_publishDriveCmd(void);
    void CB_publishTargetWaypoint(const geometry_msgs::msg::PoseStamped& msg_);
    void CB_positionSubscriber(const nav_msgs::msg::Odometry& msg_);

    void handleRosParam(void);
    void loadWaypointsFromCSV(void);
    void initRosElements(void);
    void heartbeat();

    double clipLookaheadDistance(double lookAheadDistance_) const;
    geometry_msgs::msg::PoseStamped getLookaheadPoint(const double lookAheadDistance);

    std::string _waypointsFilePath = DEFAULT_WAYPOINTS_CSV_FILE_NAME;
    std::string _positionTopic = DEFAULT_POSITION_TOPIC;
    std::string _driveCmdTopic = DEFAULT_DRIVE_CMD_TOPIC;

    double _currentSpeed = 0.0;
    double _currentX = 0.0;
    double _currentY = 0.0;
    double _currentYaw = 0.0;

    size_t _previousWaypointIndex = 0;
    bool _firstTargetWaypointLocked = false;

    std::vector<geometry_msgs::msg::PoseStamped> _waypoints;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _positionSubscriber;
    rclcpp::TimerBase::SharedPtr _loopTimer;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr _driveCmdPublisher;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr _targetWaypointPublisher;
    rclcpp::TimerBase::SharedPtr _heartbeatTimer;
    rclcpp::Publisher<arcus_msgs::msg::ErrorCode>::SharedPtr _errorPublisher;
};

#endif  // PURE_PURSUIT_HPP