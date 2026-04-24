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

	_costmapPub = create_publisher<nav_msgs::msg::OccupancyGrid>(_costmapTopic, DEFAULT_QOS_DEPTH);

	_scanSub = create_subscription<sensor_msgs::msg::LaserScan>(
		_scanTopic, rclcpp::SensorDataQoS(),
		std::bind(&CostmapMaker::scanCallback, this, std::placeholders::_1));

	  _globalMapSub = create_subscription<nav_msgs::msg::OccupancyGrid>(
		  _globalMapTopic,
		  rclcpp::QoS(1).reliable().transient_local(),
		  std::bind(&CostmapMaker::globalMapCallback, this, std::placeholders::_1));

	  _poseSub = create_subscription<nav_msgs::msg::Odometry>(
		  _poseTopic, rclcpp::SensorDataQoS(),
		  std::bind(&CostmapMaker::poseCallback, this, std::placeholders::_1));

	const auto update_period = std::chrono::duration<double>(1.0 / std::max(_updateRateHz, 1.0));
	_updateTimer = create_wall_timer(
		std::chrono::duration_cast<std::chrono::nanoseconds>(update_period),
		std::bind(&CostmapMaker::updateTimerCallback, this));

	RCLCPP_INFO(
		get_logger(),
		  "CostmapMaker started: scan='%s', pose='%s', map='%s', out='%s', cone_range=%.2fm, res=%.3fm, cone=%.1fdeg",
		  _scanTopic.c_str(), _poseTopic.c_str(), _globalMapTopic.c_str(), _costmapTopic.c_str(), _coneRangeM, _resolutionM,
		  _coneFovDeg);
}

void CostmapMaker::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
	std::lock_guard<std::mutex> lock(_dataMutex);
	_latestScan = *msg;
	_hasScan = true;
}

void CostmapMaker::globalMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
	std::lock_guard<std::mutex> lock(_dataMutex);
	_globalMap = *msg;
	_hasGlobalMap = true;
}

void CostmapMaker::poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	std::lock_guard<std::mutex> lock(_dataMutex);
	_robotX = msg->pose.pose.position.x;
	_robotY = msg->pose.pose.position.y;

	const auto& q = msg->pose.pose.orientation;
	_robotYaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
	_hasPose = true;
}

void CostmapMaker::updateTimerCallback()
{
	sensor_msgs::msg::LaserScan scan;
	nav_msgs::msg::OccupancyGrid globalMap;
	const nav_msgs::msg::OccupancyGrid* globalMapPtr = nullptr;
	bool requireUnmappedFilter = false;
	double robotX = 0.0;
	double robotY = 0.0;
	double robotYaw = 0.0;
	{
		std::lock_guard<std::mutex> lock(_dataMutex);
		if (!_hasScan) {
			return;
		}
		scan = _latestScan;

		  requireUnmappedFilter = _onlyUnmappedObstacles;
		  const bool canFilterWithGlobalMap = requireUnmappedFilter && _hasGlobalMap && _hasPose;
					if (canFilterWithGlobalMap) {
									globalMap = _globalMap;
									globalMapPtr = &globalMap;
									robotX = _robotX;
									robotY = _robotY;
									robotYaw = _robotYaw;
					}
	}

	resetGrid();
	  if (requireUnmappedFilter && globalMapPtr == nullptr) {
		  RCLCPP_WARN_THROTTLE(
			  get_logger(), *get_clock(), 5000,
			  "only_unmapped_obstacles is enabled, but map/pose is unavailable; publishing unfiltered local costmap");
		  publishCostmap();
		  return;
	  }

	markObstaclesFromScan(scan, globalMapPtr, robotX, robotY, robotYaw);
	inflateObstacles();
	publishCostmap();
}

void CostmapMaker::declareAndLoadParameters()
{
	declare_parameter<std::string>("scan_topic", _scanTopic);
	declare_parameter<std::string>("costmap_topic", _costmapTopic);
	declare_parameter<std::string>("robot_frame", _robotFrame);
	declare_parameter<std::string>("global_map_topic", _globalMapTopic);
	declare_parameter<std::string>("pose_topic", _poseTopic);

	declare_parameter<double>("resolution_m", _resolutionM);
	declare_parameter<double>("cone_range_m", _coneRangeM);
	declare_parameter<double>("update_rate_hz", _updateRateHz);

	declare_parameter<double>("min_valid_range_m", _minValidRangeM);
	declare_parameter<double>("cone_fov_deg", _coneFovDeg);

	declare_parameter<double>("inflation_radius_m", _inflationRadiusM);
	declare_parameter<double>("inscribed_radius_m", _inscribedRadiusM);
	declare_parameter<double>("cost_scaling_factor", _costScalingFactor);

	declare_parameter<bool>("only_unmapped_obstacles", _onlyUnmappedObstacles);
	declare_parameter<int>("global_obstacle_threshold", _globalObstacleThreshold);
	declare_parameter<double>("global_obstacle_neighborhood_radius_m", _globalObstacleNeighborhoodRadiusM);

	get_parameter("scan_topic", _scanTopic);
	get_parameter("costmap_topic", _costmapTopic);
	get_parameter("robot_frame", _robotFrame);
	get_parameter("global_map_topic", _globalMapTopic);
	get_parameter("pose_topic", _poseTopic);

	get_parameter("resolution_m", _resolutionM);
	get_parameter("cone_range_m", _coneRangeM);
	get_parameter("update_rate_hz", _updateRateHz);

	get_parameter("min_valid_range_m", _minValidRangeM);
	get_parameter("cone_fov_deg", _coneFovDeg);

	get_parameter("inflation_radius_m", _inflationRadiusM);
	get_parameter("inscribed_radius_m", _inscribedRadiusM);
	get_parameter("cost_scaling_factor", _costScalingFactor);
	
	get_parameter("only_unmapped_obstacles", _onlyUnmappedObstacles);
	get_parameter("global_obstacle_threshold", _globalObstacleThreshold);
	get_parameter("global_obstacle_neighborhood_radius_m", _globalObstacleNeighborhoodRadiusM);

	_resolutionM = std::max(_resolutionM, 0.01);
	_coneRangeM = std::max(_coneRangeM, _resolutionM);
	_coneFovDeg = std::clamp(_coneFovDeg, 1.0, 360.0);
	_globalObstacleThreshold = static_cast<int8_t>(std::clamp<int>(_globalObstacleThreshold, 1, 100));
	_globalObstacleNeighborhoodRadiusM = std::max(_globalObstacleNeighborhoodRadiusM, 0.0);
}

void CostmapMaker::initializeGrid()
{
	_widthM = _coneRangeM;
	_heightM = 2.0 * _coneRangeM * std::tan(0.5 * _coneFovDeg * M_PI / 180.0);

	_widthM = std::max(_widthM, _resolutionM);
	_heightM = std::max(_heightM, _resolutionM);

	_widthCells = static_cast<int>(std::ceil(_widthM / _resolutionM));
	_heightCells = static_cast<int>(std::ceil(_heightM / _resolutionM));

	if (_widthCells < 1 || _heightCells < 1) {
		throw std::runtime_error("Costmap dimensions are invalid.");
	}

	_originX = 0.0;
	_originY = -_heightM * 0.5;

	_gridMsg.info.resolution = static_cast<float>(_resolutionM);
	_gridMsg.info.width = static_cast<uint32_t>(_widthCells);
	_gridMsg.info.height = static_cast<uint32_t>(_heightCells);
	_gridMsg.info.origin.position.x = _originX;
	_gridMsg.info.origin.position.y = _originY;
	_gridMsg.info.origin.orientation.w = 1.0;
	_gridMsg.header.frame_id = _robotFrame;

	_costmapData.assign(static_cast<std::size_t>(_widthCells * _heightCells), _unknownCost);
}

void CostmapMaker::resetGrid()
{
	std::fill(_costmapData.begin(), _costmapData.end(), _freeCost);
	_obstacleIndices.clear();

	_gridMsg.info.origin.position.x = _originX;
	_gridMsg.info.origin.position.y = _originY;
}

bool CostmapMaker::beamInsideForwardCone(double scan_angle) const
{
	const double half_fov_rad = _coneFovDeg * 0.5 * M_PI / 180.0;
	return std::abs(scan_angle) <= half_fov_rad;
}

bool CostmapMaker::localToMap(double local_x, double local_y, int& mx, int& my) const
{
	mx = static_cast<int>(std::floor((local_x - _originX) / _resolutionM));
	my = static_cast<int>(std::floor((local_y - _originY) / _resolutionM));
	return mx >= 0 && mx < _widthCells && my >= 0 && my < _heightCells;
}

void CostmapMaker::setCell(int mx, int my, int8_t value)
{
	if (mx < 0 || mx >= _widthCells || my < 0 || my >= _heightCells) {
		return;
	}

	const int idx = toIndex(mx, my, _widthCells);
	_costmapData[static_cast<std::size_t>(idx)] = value;
}

void CostmapMaker::markObstaclesFromScan(const sensor_msgs::msg::LaserScan& scan,
																				 const nav_msgs::msg::OccupancyGrid* globalMap,
																				 double robotX,
																				 double robotY,
																				 double robotYaw)
{
	if (scan.ranges.empty()) {
		return;
	}
	const std::size_t count = scan.ranges.size();
	const double cone_half_fov_rad = _coneFovDeg * 0.5 * M_PI / 180.0;
	const std::size_t bottom_cone_index = std::floor((-cone_half_fov_rad - scan.angle_min) / scan.angle_increment);
	const std::size_t top_cone_index = std::floor((cone_half_fov_rad - scan.angle_min) / scan.angle_increment);

	for (std::size_t i = bottom_cone_index; i <= top_cone_index && i < count; ++i) {
		const float raw_range = scan.ranges[i];
		if (!std::isfinite(raw_range) || raw_range < _minValidRangeM) {
			continue;
		}
		const double scan_angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;

		const bool hit_obstacle =
			std::isfinite(raw_range) && raw_range <= _coneRangeM && raw_range <= scan.range_max;

		if (!hit_obstacle) {
			continue;
		}

		const double hit_x = static_cast<double>(raw_range) * std::cos(scan_angle);
		const double hit_y = static_cast<double>(raw_range) * std::sin(scan_angle);
		  if (!shouldKeepObstacle(hit_x, hit_y, globalMap, robotX, robotY, robotYaw)) {
			  continue;
		  }

		int hx = 0;
		int hy = 0;
		if (localToMap(hit_x, hit_y, hx, hy)) {
			setCell(hx, hy, _lethalCost);
			_obstacleIndices.push_back(toIndex(hx, hy, _widthCells));
		}
	}
}

	bool CostmapMaker::worldToGlobalMap(const nav_msgs::msg::OccupancyGrid& globalMap,
					    double worldX,
					    double worldY,
					    int& mx,
					    int& my) const
	{
	  const double resolution = globalMap.info.resolution;
	  if (resolution <= 0.0 || globalMap.info.width == 0U || globalMap.info.height == 0U) {
		  return false;
	  }

	  const double originX = globalMap.info.origin.position.x;
	  const double originY = globalMap.info.origin.position.y;
	  mx = static_cast<int>(std::floor((worldX - originX) / resolution));
	  my = static_cast<int>(std::floor((worldY - originY) / resolution));

	  return mx >= 0 && my >= 0 && mx < static_cast<int>(globalMap.info.width)
		 && my < static_cast<int>(globalMap.info.height);
	}

	bool CostmapMaker::shouldKeepObstacle(double localX,
					      double localY,
					      const nav_msgs::msg::OccupancyGrid* globalMap,
					      double robotX,
					      double robotY,
					      double robotYaw) const
	{
	  if (!_onlyUnmappedObstacles) {
		  return true;
	  }

	  if (globalMap == nullptr) {
		  return true;
	  }

	  const double worldX = robotX + std::cos(robotYaw) * localX - std::sin(robotYaw) * localY;
	  const double worldY = robotY + std::sin(robotYaw) * localX + std::cos(robotYaw) * localY;

	  int mapX = 0;
	  int mapY = 0;
	  if (!worldToGlobalMap(*globalMap, worldX, worldY, mapX, mapY)) {
		  return true;
	  }

	  const int width = static_cast<int>(globalMap->info.width);
	  const int height = static_cast<int>(globalMap->info.height);
	  const double resolution = globalMap->info.resolution;
	  const int neighborhoodCells =
		  static_cast<int>(std::ceil(_globalObstacleNeighborhoodRadiusM / resolution));

	  const int minX = std::max(0, mapX - neighborhoodCells);
	  const int maxX = std::min(width - 1, mapX + neighborhoodCells);
	  const int minY = std::max(0, mapY - neighborhoodCells);
	  const int maxY = std::min(height - 1, mapY + neighborhoodCells);

	  const double radiusSquared = _globalObstacleNeighborhoodRadiusM * _globalObstacleNeighborhoodRadiusM;
	  for (int y = minY; y <= maxY; ++y) {
		  for (int x = minX; x <= maxX; ++x) {
			  if (neighborhoodCells > 0) {
				  const double dx = static_cast<double>(x - mapX) * resolution;
				  const double dy = static_cast<double>(y - mapY) * resolution;
				  if (dx * dx + dy * dy > radiusSquared) {
					  continue;
				  }
			  }

			  const int index = y * width + x;
			  if (index < 0 || index >= static_cast<int>(globalMap->data.size())) {
				  continue;
			  }

			  const int8_t globalCost = globalMap->data[static_cast<std::size_t>(index)];
			  if (globalCost >= _globalObstacleThreshold) {
				  return false;
			  }
		  }
	  }

	  return true;
	}

void CostmapMaker::inflateObstacles()
{
	if (_inflationRadiusM <= 0.0 || _obstacleIndices.empty()) {
		return;
	}

	const int inflation_cells = static_cast<int>(std::ceil(_inflationRadiusM / _resolutionM));
	const std::vector<int8_t> original = _costmapData;

	for (const int obstacle_idx : _obstacleIndices) {
		const int ox = obstacle_idx % _widthCells;
		const int oy = obstacle_idx / _widthCells;

		for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
			for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
				const int nx = ox + dx;
				const int ny = oy + dy;
				if (nx < 0 || nx >= _widthCells || ny < 0 || ny >= _heightCells) {
					continue;
				}

				const double distance = std::hypot(static_cast<double>(dx) * _resolutionM,
																					 static_cast<double>(dy) * _resolutionM);
				if (distance > _inflationRadiusM) {
					continue;
				}

				const int index = toIndex(nx, ny, _widthCells);
				if (original[static_cast<std::size_t>(index)] >= _lethalCost) {
					continue;
				}

				const int8_t inflated = distanceToCost(distance);
				if (inflated > _costmapData[static_cast<std::size_t>(index)]) {
					_costmapData[static_cast<std::size_t>(index)] = inflated;
				}
			}
		}
	}
}

int8_t CostmapMaker::distanceToCost(double distance_m) const
{
	if (distance_m <= _inscribedRadiusM) {
		return _lethalCost;
	}

	if (distance_m > _inflationRadiusM) {
		return _freeCost;
	}

	const double exponent = -_costScalingFactor * (distance_m - _inscribedRadiusM);
	const double scaled = static_cast<double>(_lethalCost - 1) * std::exp(exponent);
	const double clamped = std::clamp(scaled, 1.0, static_cast<double>(_lethalCost - 1));
	return static_cast<int8_t>(std::lround(clamped));
}

void CostmapMaker::publishCostmap()
{
	_gridMsg.header.stamp = now();
	_gridMsg.header.frame_id = _robotFrame;
	_gridMsg.data = _costmapData;
	_costmapPub->publish(_gridMsg);
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<CostmapMaker>());
	rclcpp::shutdown();
	return 0;
}
