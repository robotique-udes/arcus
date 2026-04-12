#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

class TrackZoneManager : public rclcpp::Node
{
  public:
    // Initialize parameters, publishers, CSV-backed zone tables, and odometry subscription.
    TrackZoneManager()
    : Node("track_zone_manager")
    {
        odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/pf/pose/odom");
        speed_limit_topic_ = this->declare_parameter<std::string>("speed_limit_topic", "/speed_limit");
        force_algo_topic_ = this->declare_parameter<std::string>("force_algo_topic", "/force_algo");
        speed_zones_csv_path_ = this->declare_parameter<std::string>(
            "speed_zones_csv_path", "/home/arcus/arcus/resources/speed_zones.csv");
        algos_csv_path_ = this->declare_parameter<std::string>(
            "algos_csv_path", "/home/arcus/arcus/resources/algos.csv");

        speed_pub_ = this->create_publisher<std_msgs::msg::Float64>(speed_limit_topic_, 10);
        algo_pub_ = this->create_publisher<std_msgs::msg::String>(force_algo_topic_, 10);

        loadSpeedZones(speed_zones_csv_path_);
        loadAlgoZones(algos_csv_path_);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_,
            10,
            std::bind(&TrackZoneManager::odomCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "TrackZoneManager ready. Odom topic: %s", odom_topic_.c_str());
    }

  private:
    struct Point
    {
        double x;
        double y;
    };

    struct IndexedPoint
    {
        int vertex_index;
        Point point;
    };

    struct SpeedZone
    {
        int polygon_id;
        double max_speed;
        std::vector<Point> polygon;
    };

    struct AlgoZone
    {
        int polygon_id;
        std::string algorithm;
        std::vector<Point> polygon;
    };

    // Remove leading and trailing whitespace from a CSV token.
    static std::string trim(const std::string & input)
    {
        size_t start = 0;
        size_t end = input.size();

        while (start < end && std::isspace(static_cast<unsigned char>(input[start])))
        {
            ++start;
        }
        while (end > start && std::isspace(static_cast<unsigned char>(input[end - 1])))
        {
            --end;
        }

        return input.substr(start, end - start);
    }

    // Split one CSV row into comma-separated fields and trim each field.
    static std::vector<std::string> splitCsvLine(const std::string & line)
    {
        std::vector<std::string> fields;
        std::stringstream ss(line);
        std::string field;

        while (std::getline(ss, field, ','))
        {
            fields.push_back(trim(field));
        }

        return fields;
    }

    // Ray-casting point-in-polygon test: returns true when point p is inside polygon.
    static bool pointInPolygon(const Point & p, const std::vector<Point> & polygon)
    {
        if (polygon.size() < 3)
        {
            return false;
        }

        bool inside = false;
        // Traverse each edge as (pj -> pi). `j = i++` assigns the old i to j (previous vertex)
        // and then increments i (current vertex), so edges wrap as last->0, 0->1, 1->2, ...
        for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++)
        {
            const Point & pi = polygon[i];
            const Point & pj = polygon[j];

            const bool intersects = ((pi.y > p.y) != (pj.y > p.y)) &&
                (p.x < ((pj.x - pi.x) * (p.y - pi.y) / ((pj.y - pi.y) + std::numeric_limits<double>::epsilon()) + pi.x));

            if (intersects)
            {
                inside = !inside;
            }
        }

        return inside;
    }

    // Load speed polygons from CSV and store them ordered by polygon_id and vertex_index.
    void loadSpeedZones(const std::string & csv_path)
    {
        std::ifstream file(csv_path);
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Could not open speed zones CSV: %s", csv_path.c_str());
            return;
        }

        std::string line;
        if (!std::getline(file, line))
        {
            RCLCPP_ERROR(this->get_logger(), "Speed zones CSV is empty: %s", csv_path.c_str());
            return;
        }

        std::unordered_map<int, std::vector<IndexedPoint>> points_by_polygon;
        std::unordered_map<int, double> speed_by_polygon;

        while (std::getline(file, line))
        {
            if (trim(line).empty())
            {
                continue;
            }

            std::vector<std::string> fields = splitCsvLine(line);
            if (fields.size() < 5)
            {
                RCLCPP_WARN(this->get_logger(), "Skipping malformed speed zone row: %s", line.c_str());
                continue;
            }

            int polygon_id = std::stoi(fields[0]);
            int vertex_index = std::stoi(fields[1]);
            double x = std::stod(fields[2]);
            double y = std::stod(fields[3]);
            double max_speed = std::stod(fields[4]);

            points_by_polygon[polygon_id].push_back(IndexedPoint{vertex_index, Point{x, y}});
            speed_by_polygon[polygon_id] = max_speed;
        }

        // Rebuild one SpeedZone per polygon_id from the grouped CSV rows.
        for (auto & kv : points_by_polygon)
        {
            int polygon_id = kv.first;
            std::vector<IndexedPoint> & indexed_points = kv.second;
            // Vertex order from CSV is not guaranteed in the map, so sort by vertex_index first.
            std::sort(indexed_points.begin(), indexed_points.end(),
                [](const IndexedPoint & a, const IndexedPoint & b)
                {
                    return a.vertex_index < b.vertex_index;
                });

            SpeedZone zone;
            zone.polygon_id = polygon_id;
            zone.max_speed = speed_by_polygon[polygon_id];
            // Reserve once to avoid repeated allocations while copying points.
            zone.polygon.reserve(indexed_points.size());
            for (const auto & ip : indexed_points)
            {
                zone.polygon.push_back(ip.point);
            }
            speed_zones_.push_back(zone);
        }

        // Keep deterministic matching priority across runs (unordered_map iteration order is arbitrary).
        std::sort(speed_zones_.begin(), speed_zones_.end(),
            [](const SpeedZone & a, const SpeedZone & b)
            {
                return a.polygon_id < b.polygon_id;
            });

        RCLCPP_INFO(this->get_logger(), "Loaded %zu speed polygons from %s", speed_zones_.size(), csv_path.c_str());
    }

    // Load algorithm polygons from CSV and store them ordered by polygon_id and vertex_index.
    void loadAlgoZones(const std::string & csv_path)
    {
        std::ifstream file(csv_path);
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Could not open algo zones CSV: %s", csv_path.c_str());
            return;
        }

        std::string line;
        if (!std::getline(file, line))
        {
            RCLCPP_ERROR(this->get_logger(), "Algo zones CSV is empty: %s", csv_path.c_str());
            return;
        }

        std::unordered_map<int, std::vector<IndexedPoint>> points_by_polygon;
        std::unordered_map<int, std::string> algo_by_polygon;

        while (std::getline(file, line))
        {
            if (trim(line).empty())
            {
                continue;
            }

            std::vector<std::string> fields = splitCsvLine(line);
            if (fields.size() < 5)
            {
                RCLCPP_WARN(this->get_logger(), "Skipping malformed algo row: %s", line.c_str());
                continue;
            }

            int polygon_id = std::stoi(fields[0]);
            int vertex_index = std::stoi(fields[1]);
            double x = std::stod(fields[2]);
            double y = std::stod(fields[3]);
            std::string algorithm = fields[4];

            points_by_polygon[polygon_id].push_back(IndexedPoint{vertex_index, Point{x, y}});
            algo_by_polygon[polygon_id] = algorithm;
        }

        for (auto & kv : points_by_polygon)
        {
            int polygon_id = kv.first;
            std::vector<IndexedPoint> & indexed_points = kv.second;
            std::sort(indexed_points.begin(), indexed_points.end(),
                [](const IndexedPoint & a, const IndexedPoint & b)
                {
                    return a.vertex_index < b.vertex_index;
                });

            AlgoZone zone;
            zone.polygon_id = polygon_id;
            zone.algorithm = algo_by_polygon[polygon_id];
            zone.polygon.reserve(indexed_points.size());
            for (const auto & ip : indexed_points)
            {
                zone.polygon.push_back(ip.point);
            }
            algo_zones_.push_back(zone);
        }

        std::sort(algo_zones_.begin(), algo_zones_.end(),
            [](const AlgoZone & a, const AlgoZone & b)
            {
                return a.polygon_id < b.polygon_id;
            });

        RCLCPP_INFO(this->get_logger(), "Loaded %zu algo polygons from %s", algo_zones_.size(), csv_path.c_str());
    }

    // On each odometry update, publish matching speed limit and forced algorithm for current XY.
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        Point p{msg->pose.pose.position.x, msg->pose.pose.position.y};

        bool speed_found = false;
        for (const auto & zone : speed_zones_)
        {
            if (pointInPolygon(p, zone.polygon))
            {
                std_msgs::msg::Float64 speed_msg;
                speed_msg.data = zone.max_speed;
                speed_pub_->publish(speed_msg);
                speed_found = true;
                break;
            }
        }

        bool algo_found = false;
        for (const auto & zone : algo_zones_)
        {
            if (pointInPolygon(p, zone.polygon))
            {
                std_msgs::msg::String algo_msg;
                algo_msg.data = zone.algorithm;
                algo_pub_->publish(algo_msg);
                algo_found = true;
                break;
            }
        }

        if (!speed_found)
        {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "Position outside all speed polygons");
        }

        if (!algo_found)
        {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "Position outside all algo polygons");
        }
    }

    std::string odom_topic_;
    std::string speed_limit_topic_;
    std::string force_algo_topic_;
    std::string speed_zones_csv_path_;
    std::string algos_csv_path_;

    std::vector<SpeedZone> speed_zones_;
    std::vector<AlgoZone> algo_zones_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr algo_pub_;
};

int main(int argc, char ** argv)
{
    // Standard ROS 2 node lifecycle.
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrackZoneManager>());
    rclcpp::shutdown();
    return 0;
}
