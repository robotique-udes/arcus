#ifndef RACELINE_MANAGER_HPP
#define RACELINE_MANAGER_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include <string>

class RacelineManager : public rclcpp::Node
{
public:
    RacelineManager();

private:
    static constexpr const char* DEFAULT_WAYPOINTS_CSV_FILE_NAME =
        "/home/arcus/arcus/resources/waypoints/waypoints.csv";
    static constexpr const char* DEFAULT_WAYPOINTS_TOPIC = "/waypoints";
    static constexpr const uint8_t DEFAULT_QOS = 1;

    void handleRosParam();
    void loadWaypointsFromCSV();
    void initRosElements();
    void publishRaceline();

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raceline_publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    std::string _waypointsFilePath = DEFAULT_WAYPOINTS_CSV_FILE_NAME;
    std::string _waypointsTopic = DEFAULT_WAYPOINTS_TOPIC;
    nav_msgs::msg::Path raceline_;
};

#endif  // RACELINE_MANAGER_HPP