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

    // Whether map cell (x, y) lies inside a width x height grid
    bool cellInBounds(unsigned int x, unsigned int y, unsigned int width, unsigned int height);

    // Draws `count` particle indices in proportion to the particles' weights, advancing `generator`
    std::vector<int> drawIndices(const std::vector<ParticleState>& particles, int count, std::mt19937& generator);

}  // namespace particle_filter

#endif  // PF_UTILS_H
