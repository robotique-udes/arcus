#ifndef COSTMAP_MAKER_HPP
#define COSTMAP_MAKER_HPP

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

class CostmapMaker : public rclcpp::Node
{
public:
    CostmapMaker();

private:
    static constexpr std::size_t DEFAULT_QOS_DEPTH = 10;

    // Parameters (defaults before parameter load)
    std::string _scanTopic = "/scan";
    std::string _costmapTopic = "/local_costmap";
    std::string _robotFrame = "ego_racecar/base_link";
    std::string _globalMapTopic = "/map";
    std::string _poseTopic = "/pf/pose/odom";

    double _resolutionM = 0.05;
    double _coneRangeM = 7.0;
    double _updateRateHz = 30.0;

    double _minValidRangeM = 0.05;
    double _coneFovDeg = 90.0;

    double _inflationRadiusM = 0.30;
    double _inscribedRadiusM = 0.18;
    double _costScalingFactor = 10.0;
    bool _onlyUnmappedObstacles = true;
    double _globalObstacleNeighborhoodRadiusM = 0.20;

    int8_t _unknownCost = -1;
    int8_t _freeCost = 0;
    int8_t _lethalCost = 100;
    int8_t _globalObstacleThreshold = 65;

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _scanSub;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr _globalMapSub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _poseSub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr _costmapPub;
    rclcpp::TimerBase::SharedPtr _updateTimer;

    // Shared runtime state from callbacks
    std::mutex _dataMutex;
    sensor_msgs::msg::LaserScan _latestScan;
    nav_msgs::msg::OccupancyGrid _globalMap;
    bool _hasScan = false;
    bool _hasGlobalMap = false;
    bool _hasPose = false;
    double _robotX = 0.0;
    double _robotY = 0.0;
    double _robotYaw = 0.0;

    // Costmap storage and metadata
    nav_msgs::msg::OccupancyGrid _gridMsg;
    std::vector<int8_t> _costmapData;
    std::vector<int> _obstacleIndices;
    double _widthM = 0.0;
    double _heightM = 0.0;
    int _widthCells = 0;
    int _heightCells = 0;
    double _originX = 0.0;
    double _originY = 0.0;

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void globalMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void updateTimerCallback();

    void declareAndLoadParameters();
    void initializeGrid();
    void resetGrid();
    bool localToMap(double local_x, double local_y, int& mx, int& my) const;
    bool beamInsideForwardCone(double scan_angle) const;
    void setCell(int mx, int my, int8_t value);
    void markObstaclesFromScan(const sensor_msgs::msg::LaserScan& scan,
                               const nav_msgs::msg::OccupancyGrid* globalMap,
                               double robotX,
                               double robotY,
                               double robotYaw);
    void inflateObstacles();
    int8_t distanceToCost(double distance_m) const;
    bool worldToGlobalMap(const nav_msgs::msg::OccupancyGrid& globalMap,
                          double worldX,
                          double worldY,
                          int& mx,
                          int& my) const;
    bool shouldKeepObstacle(double localX,
                            double localY,
                            const nav_msgs::msg::OccupancyGrid* globalMap,
                            double robotX,
                            double robotY,
                            double robotYaw) const;
    void publishCostmap();
};

#endif // COSTMAP_MAKER_HPP