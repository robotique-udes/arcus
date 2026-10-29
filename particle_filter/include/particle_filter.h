#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <iostream>
#include <mutex>
#include <random>
#include <string.h>

#include "RangeLib.h"
#include "pf_utils.h"
using namespace ranges;

#define VAR_NO_EVAL_SENSOR_MODEL 0
#define VAR_CALC_RANGE_MANY_EVAL_SENSOR 1
#define VAR_REPEAT_ANGLES_EVAL_SENSOR 2
#define VAR_REPEAT_ANGLES_EVAL_SENSOR_ONE_SHOT 3
#define VAR_RADIAL_CDDT_OPTIMIZATIONS 4

namespace particle_filter
{

    typedef std::recursive_mutex RecursiveMutex;
    typedef std::lock_guard<std::recursive_mutex> RecursiveLock;

    typedef std::chrono::high_resolution_clock Clock;
    typedef std::chrono::_V2::system_clock::time_point Time;
    typedef std::chrono::duration<double> Duration;

    // RNG (Random Number Generator) wrapper
    class RNG
    {
      public:
        RNG()
        {
            // Initialize random number generator
            // Source: https://stackoverflow.com/a/13446015
            std::random_device dev;
            std::chrono::_V2::system_clock::duration time = std::chrono::_V2::system_clock::now().time_since_epoch();
            std::mt19937::result_type time_seconds = std::chrono::duration_cast<std::chrono::seconds>(time).count();
            std::mt19937::result_type time_microseconds = std::chrono::duration_cast<std::chrono::microseconds>(time).count();
            std::mt19937::result_type seed = dev() ^ (time_seconds + time_microseconds);
            gen_.seed(seed);
        }

        std::mt19937& engine()
        {
            return gen_;
        }

      private:
        std::mt19937 gen_;
    };

    inline double durationMsec(const Time& start, const Time& end)
    {
        return std::chrono::duration_cast<Duration>(end - start).count() * 1000.0;
    }
    //----------------------------------------------------------------------------*/

    class ParticleFilter : public rclcpp::Node
    {
      public:
        ParticleFilter(const rclcpp::NodeOptions& options);

        void loadParam();
        void precomputeSensorModel();
        void setupROS();

        // Initialize particles over the permissible region of state space
        void initializeGlobalDistribution();

        // Initialize particles according to a pose msg
        void initializeParticlesPose(const geometry_msgs::msg::PoseWithCovarianceStamped& msg);

        // Callback functions
        void lidar_cb(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg);
        void odom_cb(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
        void laser_odom_cb(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg,
                           const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg);
        void clickedPose_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
        void map_cb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
        void globalLocalization_cb(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                   std::shared_ptr<std_srvs::srv::Empty::Response> response);

        // Fucntions for MCL algorithm
        void update();
        void sampling(int count);
        // Moves the particles by odometry_data_; false if it was below the standstill threshold and wasn't applied
        bool motionModel();
        void sensorModel();
        void applyRoughening();
        void expectedPoseMean();
        void getMaxWeightParticle();
        void publishTfOdom();
        void visualize();

        // Utils
        std::vector<unsigned int> worldToMap(std::vector<double> position);
        std::vector<double> mapToWorld(std::vector<double> idx);
        double getYaw(const geometry_msgs::msg::Quaternion& q);

      private:
        // ROS interface
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_pub_;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr fake_scan_pub_;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr global_localization_srv_;

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        // Data containers used in MCL algorithm
        double max_range_px_;
        RangeMethod* range_method_;
        RecursiveMutex particles_mtx_;
        nav_msgs::msg::OccupancyGrid::SharedPtr loaded_map_;
        std::vector<ParticleState> particles_;
        ParticleState expected_pose_;

        bool map_initialized_;
        bool odom_initialized_;
        bool lidar_initialized_;
        rclcpp::Time last_stamp_;
        rclcpp::Time scan_stamp_;  // stamp of the scan the latest update used
        Pose2D last_pose_;        // latest odometry pose
        Pose2D last_update_pose_;  // odometry pose consumed by the previous update
        Pose2D odometry_data_;     // motion since the previous update, in the robot frame
        double linear_speed;
        double angular_speed;

        double angle_min_;
        double angle_increment_;
        std::vector<double> downsampled_laser_angles_;
        std::vector<double> downsampled_laser_ranges_;

        // Initial pose
        bool set_initial_pose_;
        double init_pose_x_;
        double init_pose_y_;
        double init_pose_z_;
        double init_orientation_x_;
        double init_orientation_y_;
        double init_orientation_z_;
        double init_orientation_w_;

        // Sampling related tools
        RNG rng_;                                         // Random number generator
        std::uniform_real_distribution<double> x_dist_;   // Distribution of x locations in map frame [0, width)
        std::uniform_real_distribution<double> y_dist_;   // Distribution of y locations in map frame [0, height)
        std::uniform_real_distribution<double> th_dist_;  // Distribution of theta in map frame (-pi, pi]
        double ess_threshold_;  // Threshold for effective sample size (ESS) to trigger resampling

        // Global localization
        double global_squash_factor_;         // squash factor while relocalizing, flatter than squash_factor_
        double global_converged_spread_m_;    // position spread under which the cloud counts as converged
        double global_converged_spread_rad_;  // heading spread under which the cloud counts as converged
        int global_converged_scans_;          // consecutive converged scans before switching to tracking
        bool relocalizing_{false};            // whether a global localization hasn't converged yet
        int converged_scans_{0};              // consecutive scans the cloud has been converged for

        // topic parameters
        std::string scan_topic_;
        std::string odometry_topic_;

        // tf parameters
        std::string map_frame_;
        std::string odom_frame_;
        std::string laser_frame_;
        double transform_tolerance_;  // seconds the published map -> odom is post-dated by

        // sensor model constants
        double z_short_;
        double z_max_;
        double z_rand_;
        double z_hit_;
        double sigma_hit_;
        double lambda_short_;

        // TUM motion model constants
        double alpha_1_;        // how rotation affects rotation noise
        double alpha_2_;        // how (inverse) translation affects rotation noise
        double alpha_3_;        // how translation affects translation noise
        double alpha_4_;        // how rotation affects translation noise
        double lambda_thresh_;  // floor on the translation in the alpha_2 term [m]
        double noise_forward_bias_;   // shift of the translation noise mean, as a fraction of its std
        double standstill_m_;       // below this translation, particles aren't moved [m]

        // options
        int viz_;
        int fine_timing_;
        int publish_odom_;

        // downsampling parameter and other parameters
        int angle_step_;
        int num_downsampled_angles_;
        int tracking_particles_;  // particle count once localized ("max_particles" parameter)
        int global_particles_;    // particle count during global localization
        int buffer_capacity_;     // size of the range_libc arrays: max(global_particles_, tracking_particles_)
        int max_viz_particles_;
        int rangelib_variant_;
        std::string which_range_method_;
        double theta_discretization_;
        double squash_factor_;
        double max_range_;

        std::string pose_selection_method_;

        // Timing related variables
        double sensor_model_calc_worst_time_;
        double motion_model_calc_worst_time_;

        // Set up array pointers
        float* angles_;
        float* obs_;
        float* outs_;
        double* weights_;
        float* samples_;
        float* viz_queries_;
        float* viz_ranges_;
    };

}  // namespace particle_filter

#endif  // PARTICLE_FILTER_H
