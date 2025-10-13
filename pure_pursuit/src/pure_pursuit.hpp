#ifndef PURE_PURSUIT_HPP
#define PURE_PURSUIT_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include <fstream>
#include <string>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

class PurePursuit : public rclcpp::Node
{
    // BELOW ARE PARAMETERS THAT CAN BE TUNED FOR BETTER PERFORMANCE
    static constexpr double MAX_LOOKAHEAD_DISTANCE_M = 3.5;
    static constexpr double MIN_LOOKAHEAD_DISTANCE_M = 0.4;
    static constexpr double LOOKAHEAD_DISTANCE_GAIN = 0.25; // speed multiplier for lookahead distance
    static constexpr double LOOP_FREQUENCY_HZ = 20.0;
    static constexpr double WHEELBASE_M = 0.325; // wheelbase of the vehicle in meters
    static constexpr double MAX_LOOKAHEAD_FRACTION_OF_TRACK = 0.05;
    static constexpr double EXPONENTIAL_A_CTE = 5.f; // wheelbase of the vehicle in meters
    static constexpr double EXPONENTIAL_B_CTE = 5.f;   // INPUTS TOPIC AND PATHS
    static constexpr double EXPONENTIAL_C_CTE = 0.1f;
    /* This obviously just works in simulation for now. We will want to subscribe to a topic giving us accurate position based 
    on our chosen localization method(s) */
    static constexpr const uint8_t DEFAULT_QOS = 1;
    static constexpr const char* CURRENT_POSITION_TOPIC = "/ego_racecar/odom";
    static constexpr const char* DRIVE_CMD_TOPIC = "/drive";
    static constexpr const char* TARGET_WAYPOINT_TOPIC = "/target_waypoint";
    static constexpr const char* WAYPOINTS_CSV_FILE_NAME = "/sim_ws/src/arcus/resources/waypoints/Spielberg_center.csv";

    static constexpr const double PI = 3.14159;
    static constexpr const double e = 2.71828;

  public:
    PurePursuit();

  private:
    void CB_publishDriveCmd(void);
    void CB_positionSubscriber(const nav_msgs::msg::Odometry& msg);
    void CB_publishTargetWaypoint(const geometry_msgs::msg::PoseStamped& msg);

    void loadWaypointsFromCSV(void);
    void initRosElements(void);
    
    void clipLookaheadDistance(double& _lookAheadDistance);
    geometry_msgs::msg::PoseStamped getLookaheadPoint(double currentX, double currentY, double lookAheadDistance);

    double speedFromWheelAngle(double& wheelAngle);

    double _currentSpeed = 0.0;
    double _currentX = 0.0;
    double _currentY = 0.0;
    double _currentYaw = 0.0;

    size_t _previousBestIndex = 0.0;
    bool _firstTargetLocked = false;

    double _lookAheadDistance = 0.0;
    double _alpha = 0.0;
    std::vector<geometry_msgs::msg::PoseStamped> _waypoints;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _positionSubscriber;
    rclcpp::TimerBase::SharedPtr _loopTimer;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr _driveCmdPublisher;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr _targetWaypointPublisher;

};

#endif  // PURE_PURSUIT_HPP