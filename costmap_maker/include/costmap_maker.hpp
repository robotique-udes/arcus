#ifndef COSTMAP_MAKER_HPP
#define COSTMAP_MAKER_HPP

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
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
    std::string scan_topic_ = "/scan";
    std::string costmap_topic_ = "/local_costmap";
    std::string robot_frame_ = "ego_racecar/base_link";

    double resolution_m_ = 0.05;
    double cone_range_m_ = 7.0;
    double update_rate_hz_ = 30.0;

    double obstacle_range_m_ = 5.0;
    double min_valid_range_m_ = 0.05;
    double cone_fov_deg_ = 90.0;

    double inflation_radius_m_ = 0.30;
    double inscribed_radius_m_ = 0.18;
    double cost_scaling_factor_ = 10.0;

    int8_t unknown_cost_ = -1;
    int8_t free_cost_ = 0;
    int8_t lethal_cost_ = 100;

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::TimerBase::SharedPtr update_timer_;

    // Shared runtime state from callbacks
    std::mutex data_mutex_;
    sensor_msgs::msg::LaserScan latest_scan_;
    bool has_scan_ = false;

    // Costmap storage and metadata
    nav_msgs::msg::OccupancyGrid grid_msg_;
    std::vector<int8_t> costmap_data_;
    std::vector<int> obstacle_indices_;
    double width_m_ = 0.0;
    double height_m_ = 0.0;
    int width_cells_ = 0;
    int height_cells_ = 0;
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void updateTimerCallback();

    void declareAndLoadParameters();
    void initializeGrid();
    void resetGrid();
    bool localToMap(double local_x, double local_y, int& mx, int& my) const;
    bool beamInsideForwardCone(double scan_angle) const;
    void setCell(int mx, int my, int8_t value);
    void markObstaclesFromScan(const sensor_msgs::msg::LaserScan& scan);
    void inflateObstacles();
    int8_t distanceToCost(double distance_m) const;
    void publishCostmap();
};

#endif // COSTMAP_MAKER_HPP