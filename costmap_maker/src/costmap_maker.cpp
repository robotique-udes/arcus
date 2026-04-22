#include "costmap_maker.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>

namespace
{
inline int toIndex(int mx, int my, int width)
{
	return my * width + mx;
}
}  // namespace

CostmapMaker::CostmapMaker()
: Node("costmap_maker")
{
	declareAndLoadParameters();
	initializeGrid();

	costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(costmap_topic_, DEFAULT_QOS_DEPTH);

	scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
		scan_topic_, rclcpp::SensorDataQoS(),
		std::bind(&CostmapMaker::scanCallback, this, std::placeholders::_1));

	const auto update_period = std::chrono::duration<double>(1.0 / std::max(update_rate_hz_, 1.0));
	update_timer_ = create_wall_timer(
		std::chrono::duration_cast<std::chrono::nanoseconds>(update_period),
		std::bind(&CostmapMaker::updateTimerCallback, this));

	RCLCPP_INFO(
		get_logger(),
		"CostmapMaker started: scan='%s', pose='%s', out='%s', cone_range=%.2fm, res=%.3fm, cone=%.1fdeg",
		scan_topic_.c_str(), "(unused)", costmap_topic_.c_str(), cone_range_m_, resolution_m_, cone_fov_deg_);
}

void CostmapMaker::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
	std::lock_guard<std::mutex> lock(data_mutex_);
	latest_scan_ = *msg;
	has_scan_ = true;
}
void CostmapMaker::updateTimerCallback()
{
	sensor_msgs::msg::LaserScan scan;
	{
		std::lock_guard<std::mutex> lock(data_mutex_);
		if (!has_scan_) {
			return;
		}
		scan = latest_scan_;
	}

	resetGrid();
	markObstaclesFromScan(scan);
	inflateObstacles();
	publishCostmap();
}

void CostmapMaker::declareAndLoadParameters()
{
	declare_parameter<std::string>("scan_topic", scan_topic_);
	declare_parameter<std::string>("costmap_topic", costmap_topic_);
	declare_parameter<std::string>("robot_frame", robot_frame_);

	declare_parameter<double>("resolution_m", resolution_m_);
	declare_parameter<double>("cone_range_m", cone_range_m_);
	declare_parameter<double>("update_rate_hz", update_rate_hz_);

	declare_parameter<double>("obstacle_range_m", obstacle_range_m_);
	declare_parameter<double>("min_valid_range_m", min_valid_range_m_);
	declare_parameter<double>("cone_fov_deg", cone_fov_deg_);

	declare_parameter<double>("inflation_radius_m", inflation_radius_m_);
	declare_parameter<double>("inscribed_radius_m", inscribed_radius_m_);
	declare_parameter<double>("cost_scaling_factor", cost_scaling_factor_);

	get_parameter("scan_topic", scan_topic_);
	get_parameter("costmap_topic", costmap_topic_);
	get_parameter("robot_frame", robot_frame_);

	get_parameter("resolution_m", resolution_m_);
	get_parameter("cone_range_m", cone_range_m_);
	get_parameter("update_rate_hz", update_rate_hz_);

	get_parameter("obstacle_range_m", obstacle_range_m_);
	get_parameter("min_valid_range_m", min_valid_range_m_);
	get_parameter("cone_fov_deg", cone_fov_deg_);

	get_parameter("inflation_radius_m", inflation_radius_m_);
	get_parameter("inscribed_radius_m", inscribed_radius_m_);
	get_parameter("cost_scaling_factor", cost_scaling_factor_);

	resolution_m_ = std::max(resolution_m_, 0.01);
	cone_range_m_ = std::max(cone_range_m_, resolution_m_);
	cone_fov_deg_ = std::clamp(cone_fov_deg_, 1.0, 360.0);
}

void CostmapMaker::initializeGrid()
{
	width_m_ = cone_range_m_;
	height_m_ = 2.0 * cone_range_m_ * std::tan(0.5 * cone_fov_deg_ * M_PI / 180.0);

	width_m_ = std::max(width_m_, resolution_m_);
	height_m_ = std::max(height_m_, resolution_m_);

	width_cells_ = static_cast<int>(std::ceil(width_m_ / resolution_m_));
	height_cells_ = static_cast<int>(std::ceil(height_m_ / resolution_m_));

	if (width_cells_ < 1 || height_cells_ < 1) {
		throw std::runtime_error("Costmap dimensions are invalid.");
	}

	origin_x_ = 0.0;
	origin_y_ = -height_m_ * 0.5;

	grid_msg_.info.resolution = static_cast<float>(resolution_m_);
	grid_msg_.info.width = static_cast<uint32_t>(width_cells_);
	grid_msg_.info.height = static_cast<uint32_t>(height_cells_);
	grid_msg_.info.origin.position.x = origin_x_;
	grid_msg_.info.origin.position.y = origin_y_;
	grid_msg_.info.origin.orientation.w = 1.0;
	grid_msg_.header.frame_id = robot_frame_;

	costmap_data_.assign(static_cast<std::size_t>(width_cells_ * height_cells_), unknown_cost_);
}

void CostmapMaker::resetGrid()
{
	std::fill(costmap_data_.begin(), costmap_data_.end(), free_cost_);
	obstacle_indices_.clear();

	grid_msg_.info.origin.position.x = origin_x_;
	grid_msg_.info.origin.position.y = origin_y_;
}

bool CostmapMaker::beamInsideForwardCone(double scan_angle) const
{
	const double half_fov_rad = cone_fov_deg_ * 0.5 * M_PI / 180.0;
	return std::abs(scan_angle) <= half_fov_rad;
}

bool CostmapMaker::localToMap(double local_x, double local_y, int& mx, int& my) const
{
	mx = static_cast<int>(std::floor((local_x - origin_x_) / resolution_m_));
	my = static_cast<int>(std::floor((local_y - origin_y_) / resolution_m_));
	return mx >= 0 && mx < width_cells_ && my >= 0 && my < height_cells_;
}

void CostmapMaker::setCell(int mx, int my, int8_t value)
{
	if (mx < 0 || mx >= width_cells_ || my < 0 || my >= height_cells_) {
		return;
	}

	const int idx = toIndex(mx, my, width_cells_);
	costmap_data_[static_cast<std::size_t>(idx)] = value;
}

void CostmapMaker::markObstaclesFromScan(const sensor_msgs::msg::LaserScan& scan)
{
	const std::size_t count = scan.ranges.size();
	for (std::size_t i = 0; i < count; ++i) {
		const float raw_range = scan.ranges[i];
		if (!std::isfinite(raw_range) || raw_range < min_valid_range_m_) {
			continue;
		}

		const double scan_angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
		if (!beamInsideForwardCone(scan_angle)) {
			continue;
		}

		const bool hit_obstacle =
			std::isfinite(raw_range) && raw_range <= obstacle_range_m_ && raw_range <= scan.range_max;

		if (!hit_obstacle) {
			continue;
		}

		const double hit_x = static_cast<double>(raw_range) * std::cos(scan_angle);
		const double hit_y = static_cast<double>(raw_range) * std::sin(scan_angle);
		int hx = 0;
		int hy = 0;
		if (localToMap(hit_x, hit_y, hx, hy)) {
			setCell(hx, hy, lethal_cost_);
			obstacle_indices_.push_back(toIndex(hx, hy, width_cells_));
		}
	}
}

void CostmapMaker::inflateObstacles()
{
	if (inflation_radius_m_ <= 0.0 || obstacle_indices_.empty()) {
		return;
	}

	const int inflation_cells = static_cast<int>(std::ceil(inflation_radius_m_ / resolution_m_));
	const std::vector<int8_t> original = costmap_data_;

	for (const int obstacle_idx : obstacle_indices_) {
		const int ox = obstacle_idx % width_cells_;
		const int oy = obstacle_idx / width_cells_;

		for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
			for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
				const int nx = ox + dx;
				const int ny = oy + dy;
				if (nx < 0 || nx >= width_cells_ || ny < 0 || ny >= height_cells_) {
					continue;
				}

				const double distance = std::hypot(static_cast<double>(dx) * resolution_m_,
																					 static_cast<double>(dy) * resolution_m_);
				if (distance > inflation_radius_m_) {
					continue;
				}

				const int index = toIndex(nx, ny, width_cells_);
				if (original[static_cast<std::size_t>(index)] >= lethal_cost_) {
					continue;
				}

				const int8_t inflated = distanceToCost(distance);
				if (inflated > costmap_data_[static_cast<std::size_t>(index)]) {
					costmap_data_[static_cast<std::size_t>(index)] = inflated;
				}
			}
		}
	}
}

int8_t CostmapMaker::distanceToCost(double distance_m) const
{
	if (distance_m <= inscribed_radius_m_) {
		return lethal_cost_;
	}

	if (distance_m > inflation_radius_m_) {
		return free_cost_;
	}

	const double exponent = -cost_scaling_factor_ * (distance_m - inscribed_radius_m_);
	const double scaled = static_cast<double>(lethal_cost_ - 1) * std::exp(exponent);
	const double clamped = std::clamp(scaled, 1.0, static_cast<double>(lethal_cost_ - 1));
	return static_cast<int8_t>(std::lround(clamped));
}

void CostmapMaker::publishCostmap()
{
	grid_msg_.header.stamp = now();
	grid_msg_.header.frame_id = robot_frame_;
	grid_msg_.data = costmap_data_;
	costmap_pub_->publish(grid_msg_);
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<CostmapMaker>());
	rclcpp::shutdown();
	return 0;
}
