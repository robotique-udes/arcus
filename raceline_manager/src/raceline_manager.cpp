#include "raceline_manager.hpp"

#include <chrono>
#include <fstream>
#include <sstream>

RacelineManager::RacelineManager()
: rclcpp::Node("raceline_manager")
{
    handleRosParam();
    initRosElements();
    loadWaypointsFromCSV();
    publishRaceline();
}

void RacelineManager::handleRosParam()
{
    this->declare_parameter<std::string>(
        "waypoints_file_path",
        DEFAULT_WAYPOINTS_CSV_FILE_NAME);
    this->declare_parameter<std::string>("waypoints_topic", DEFAULT_WAYPOINTS_TOPIC);
    this->declare_parameter<std::string>("frame_id", "map");

    _waypointsFilePath = this->get_parameter("waypoints_file_path").as_string();
    _waypointsTopic = this->get_parameter("waypoints_topic").as_string();
    raceline_.header.frame_id = this->get_parameter("frame_id").as_string();

    RCLCPP_INFO(this->get_logger(), "Waypoints file path: %s", _waypointsFilePath.c_str());
    RCLCPP_INFO(this->get_logger(), "Waypoints topic: %s", _waypointsTopic.c_str());
}

void RacelineManager::initRosElements()
{
    rclcpp::QoS qos(rclcpp::KeepLast(DEFAULT_QOS));
    qos.transient_local();
    raceline_publisher_ =
        this->create_publisher<nav_msgs::msg::Path>(_waypointsTopic, qos);
    publish_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        [this]() { publishRaceline(); });
}

void RacelineManager::loadWaypointsFromCSV()
{
    std::ifstream inputFile(_waypointsFilePath);
    if (!inputFile.is_open())
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Could not open specified file for waypoints : '%s'",
            _waypointsFilePath.c_str());
        return;
    }

    if (inputFile.peek() == std::ifstream::traits_type::eof())
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Specified file containing waypoints is empty : '%s'",
            _waypointsFilePath.c_str());
        RCLCPP_INFO(
            this->get_logger(),
            "Make sure the waypoints file exists at: %s",
            _waypointsFilePath.c_str());
        return;
    }

    std::string line;
    while (std::getline(inputFile, line))
    {
        if (line.empty())
        {
            continue;
        }

        std::stringstream ss(line);
        std::string xPos;
        std::string yPos;
        std::getline(ss, xPos, ',');
        std::getline(ss, yPos, ',');

        try
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = raceline_.header.frame_id;
            pose.header.stamp = this->now();
            pose.pose.position.x = std::stod(xPos);
            pose.pose.position.y = std::stod(yPos);
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            raceline_.poses.push_back(pose);
        }
        catch (const std::exception&)
        {
            RCLCPP_WARN(this->get_logger(), "Skipping invalid waypoint row: '%s'", line.c_str());
        }
    }
    inputFile.close();

    RCLCPP_INFO(
        this->get_logger(),
        "Loaded %zu waypoints from '%s'",
        raceline_.poses.size(),
        _waypointsFilePath.c_str());
}

void RacelineManager::publishRaceline()
{
    raceline_.header.stamp = this->now();
    for (auto& pose : raceline_.poses)
    {
        pose.header.stamp = raceline_.header.stamp;
        pose.header.frame_id = raceline_.header.frame_id;
    }
    raceline_publisher_->publish(raceline_);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RacelineManager>());
    rclcpp::shutdown();
    return 0;
}
