#ifndef PF_UTILS_H
#define PF_UTILS_H

// ROS-free helpers for the particle filter, kept separate so they can be unit tested

#include <random>
#include <vector>

namespace particle_filter
{

    typedef struct
    {
        double x;
        double y;
        double theta;
        double weight;
    } ParticleState;

    struct Pose2D
    {
        double x{0.0};
        double y{0.0};
        double theta{0.0};
    };

    // Angle wrapped to [-pi, pi]
    double wrapAngle(double angle);

    // Pose of `to` expressed in the frame of `from`, heading change wrapped to [-pi, pi]
    Pose2D relativePose(const Pose2D& from, const Pose2D& to);

    // Weighted mean of the particles; the heading is the circular mean, so it is correct across +-pi
    Pose2D weightedMeanPose(const std::vector<ParticleState>& particles);

    struct Spread
    {
        double position_m{0.0};   // weighted RMS distance of the particles from the mean
        double heading_rad{0.0};  // circular standard deviation of the headings
    };

    // How spread out the particles are around `mean` (as returned by weightedMeanPose)
    Spread weightedSpread(const std::vector<ParticleState>& particles, const Pose2D& mean);

    // Whether map cell (x, y) lies inside a width x height grid
    bool cellInBounds(unsigned int x, unsigned int y, unsigned int width, unsigned int height);

    // Draws `count` particle indices in proportion to the particles' weights, advancing `generator`
    std::vector<int> drawIndices(const std::vector<ParticleState>& particles, int count, std::mt19937& generator);

    // Draws `count` particle indices by systematic (low-variance) resampling: one random offset, then
    // evenly spaced points along the cumulative weights. Each particle gets floor(w * count) or
    // ceil(w * count) copies, in O(N)
    std::vector<int> systematicIndices(const std::vector<ParticleState>& particles, int count, std::mt19937& generator);

    // Effective sample size 1 / sum(w^2) of normalized weights; 0 if every weight is 0
    double calculateESS(const std::vector<ParticleState>& particles);

}  // namespace particle_filter

#endif  // PF_UTILS_H
