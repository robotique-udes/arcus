#ifndef PURE_PURSUIT_HPP
#define PURE_PURSUIT_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "arcus_msgs/msg/error_code.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <mutex>

class PurePursuit : public rclcpp::Node
{
    struct waypoint_t
    {
        geometry_msgs::msg::PoseStamped point;
        double speed;
    };

    static constexpr const uint8_t DEFAULT_QOS = 1;
    static constexpr const double EPS = 1.0e-6;
    static constexpr const double PI = 3.14159;

    // Fallback values for ROS2 param, will hold param value after it is loaded
    std::string waypointsFilePath = "/home/arcus/arcus/resources/waypoints/waypoints.csv";

    std::string positionTopic = "/pf/pose/odom";
    std::string driveCmdTopic = "/pure_pursuit/drive";
    std::string targetWaypointTopic = "/target_waypoint";
    std::string costmapTopic = "/local_costmap";
    std::string trajectoryRiskTopic = "/pure_pursuit/trajectory_risk";
    std::string errorTopic = "/node_error_code";
    std::string riskPathTopic = "/pure_pursuit/risk_path_segment";

    double maxLookahead = 3.5;           // Maximum lookahead distance in meters
    double minLookahead = 0.2;           // Minimum lookahead distance in meters
    double lookaheadGain = 0.5;          // Gain applied to calculated lookahead distance based on current speed
    double maxLookaheadFraction = 0.05;  // @TODO REMOVEE!!!
    double loopFrequency = 38.0;         // Frequency at which the main loop runs, in Hz
    double wheelbase = 0.325;            // Wheelbase of the vehicle in meters
    double speedMin = 0.5;               // Minimum speed in meters per second
    double speedMax = 7.0;               // Maximum speed in meters per second
    double latAccelMax = 2.0;            // Maximum lateral acceleration in meters per second squared
    double longAccelMax = 4.0;           // Maximum acceleration in meters per second squared
    double longBrakeMax = 3.0;           // Maximum braking deceleration in meters per second squared

    double riskLookaheadGain = 1.0;  // Gain applied to calculated risk lookahead distance based on current speed
    double ttcDecayRate = 1.0;       // Time-to-collision exponential decay rate for risk weighting
    double ttcMinSpeed = 0.3;  // Minimum speed in meters per second for ttc (will clip to this value if current speed is lower)
    double riskInterpolationStep
        = 0.1;  // Step size in meters for interpolating points along the path segment when calculating risk

    double relocalizeDistance = 4.0;  // TODO, might not by needed anymore !!!!!!!!!

    double recoveryTriggerSpeed
        = 0.06;  // Speed below which recovery mode is triggered (relies on risk mechanism slowing the vehicle down)
    double recoveryReverseSpeed = 0.7;             // Speed used when reversing during recovery
    double recoveryDisengageSteerRad = PI / 12.0;  // Steering angle at which to disengage during recovery
    double recoveryRearmSpeed = 1.0;               // Speed at which to rearm recovery after disengaging

    bool debug = false;

  public:
    PurePursuit();

  private:
    void CB_mainDecisionLoop(void);
    void CB_publishTargetWaypoint(const geometry_msgs::msg::PoseStamped& msg_);
    void CB_publishRiskPathSegment(void);
    void CB_publishTrajectoryRisk(void);

    void CB_positionSubscriber(const nav_msgs::msg::Odometry& msg_);
    void CB_costmapSubscriber(const nav_msgs::msg::OccupancyGrid& msg_);

    void handleRosParam(void);
    void initRosElements(void);
    void initParamCallbackHandle(void);
    void heartbeat(void);

    void loadWaypointsFromCSV(void);
    void calculateSpeed(void);

    double clipLookaheadDistance(double lookAheadDistance_) const;
    waypoint_t getLookaheadPoint(const double lookAheadDistance);
    ackermann_msgs::msg::AckermannDriveStamped getPurePursuitDriveCommand(const waypoint_t& _waypoint);
    double calculateTrajectoryRisk(double lookaheadDistance);
    void evaluatePointRisk(double x, double y, double cumulativeDistance, double& riskMax);

    double _currentSpeed = 0.0;
    double _currentX = 0.0;
    double _currentY = 0.0;
    double _currentYaw = 0.0;

    size_t _previousWaypointIndex = 0;

    bool _firstTargetWaypointLocked = false;
    bool _recoveryActive = false;
    bool _recoveryArmed = false;
    bool _carHasEverMoved = false;
    bool _hasLastTrajectoryRisk = false;

    double _recoverySteeringAngle = 0.0;
    double _lastTrajectoryRisk = 0.0;

    // Costmap data
    std::vector<int8_t> _costmapData;
    int _costmapWidth = 0;
    int _costmapHeight = 0;
    double _costmapResolution = 0.05;
    double _costmapOriginX = 0.0;
    double _costmapOriginY = 0.0;
    std::string _costmapFrameId;
    std::mutex _costmapMutex;

    std::vector<waypoint_t> _waypoints;
    std::vector<geometry_msgs::msg::PoseStamped> _riskPathWaypoints;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _positionSubscriber;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr _costmapSubscriber;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr _driveCmdPublisher;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr _targetWaypointPublisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _trajectoryRiskPublisher;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _riskPathPublisher;
    rclcpp::Publisher<arcus_msgs::msg::ErrorCode>::SharedPtr _errorPublisher;

    rclcpp::TimerBase::SharedPtr _loopTimer;
    rclcpp::TimerBase::SharedPtr _heartbeatTimer;

    tf2_ros::Buffer _tfBuffer;
    tf2_ros::TransformListener _tfListener;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr _paramCallbackHandle;
};

#endif  // PURE_PURSUIT_HPP