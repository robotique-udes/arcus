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
#include <cmath>

class PurePursuit : public rclcpp::Node
{
    struct Waypoint
    {
        geometry_msgs::msg::PoseStamped point;
        double speed;
    };

    // fallback constants and also holds constants after param loaded
    double MAX_LOOKAHEAD_M = 3.5;
    double MIN_LOOKAHEAD_M = 0.2;
    double LOOKAHEAD_GAIN = 0.5;
    double MAX_LOOKAHEAD_FRACTION = 0.05;
    double LOOP_FREQUENCY_HZ = 38.0;
    double WHEELBASE_M = 0.325;
    double SPEED_MIN = 0.5;
    double SPEED_MAX = 7.0;
    double A_LAT_MAX = 2.0;
    double A_ACCEL_MAX = 4.0;
    double A_BRAKE_MAX = 3.0;
    double SPEED_EPS = 1.0e-6;

    double PI = 3.14159;
    double RECOVERY_TRIGGER_SPEED_MS = 0.06;
    double RECOVERY_REVERSE_SPEED_MS = 0.7;
    double RECOVERY_DISENGAGE_STEER_RAD = PI / 12.0;
    double RECOVERY_REARM_SPEED_MS = 1.0;

    // Topic, input file names and QoS
    static constexpr const uint8_t DEFAULT_QOS = 1;
    static constexpr const char* DEFAULT_DRIVE_CMD_TOPIC = "/pure_pursuit/drive";
    static constexpr const char* TARGET_WAYPOINT_TOPIC = "/target_waypoint";
    static constexpr const char* DEFAULT_POSITION_TOPIC = "/pf/pose/odom";
    static constexpr const char* DEFAULT_WAYPOINTS_CSV_FILE_NAME = "/home/arcus/arcus/resources/waypoints/waypoints.csv";

  public:
    PurePursuit();

  private:
    void CB_publishDriveCmd(void);
    void CB_publishTargetWaypoint(const geometry_msgs::msg::PoseStamped& msg_);
    void CB_positionSubscriber(const nav_msgs::msg::Odometry& msg_);

    void handleRosParam(void);
    void loadWaypointsFromCSV(void);
    void calculateSpeed(void);
    void initRosElements(void);
    void initParamCallbackHandle(void);
    void heartbeat();

    double clipLookaheadDistance(double lookAheadDistance_) const;
    Waypoint getLookaheadPoint(const double lookAheadDistance);

    std::string _waypointsFilePath = DEFAULT_WAYPOINTS_CSV_FILE_NAME;
    std::string _positionTopic = DEFAULT_POSITION_TOPIC;
    std::string _driveCmdTopic = DEFAULT_DRIVE_CMD_TOPIC;

    double _currentSpeed = 0.0;
    double _currentX = 0.0;
    double _currentY = 0.0;
    double _currentYaw = 0.0;

    size_t _previousWaypointIndex = 0;
    bool _firstTargetWaypointLocked = false;
    bool _recoveryActive = false;
    bool _recoveryArmed = false;
    bool _carHasEverMoved = false;
    double _recoverySteeringAngle = 0.0;

    std::vector<Waypoint> _waypoints;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _positionSubscriber;
    rclcpp::TimerBase::SharedPtr _loopTimer;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr _driveCmdPublisher;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr _targetWaypointPublisher;
    rclcpp::TimerBase::SharedPtr _heartbeatTimer;
    rclcpp::Publisher<arcus_msgs::msg::ErrorCode>::SharedPtr _errorPublisher;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr _paramCallbackHandle;

};

#endif  // PURE_PURSUIT_HPP