#include "pf_utils.h"

#include <algorithm>
#include <cmath>

namespace particle_filter
{
    double wrapAngle(double angle)
    {
        return std::remainder(angle, 2.0 * M_PI);
    }

    Pose2D relativePose(const Pose2D& from, const Pose2D& to)
    {
        double dx = to.x - from.x;
        double dy = to.y - from.y;
        double c = cos(from.theta);
        double s = sin(from.theta);

        Pose2D delta;
        delta.x = c * dx + s * dy;
        delta.y = -s * dx + c * dy;
        delta.theta = wrapAngle(to.theta - from.theta);
        return delta;
    }

    Pose2D weightedMeanPose(const std::vector<ParticleState>& particles)
    {
        Pose2D mean;
        double sin_sum = 0.0;
        double cos_sum = 0.0;
        for (const ParticleState& particle : particles)
        {
            mean.x += particle.weight * particle.x;
            mean.y += particle.weight * particle.y;
            sin_sum += particle.weight * sin(particle.theta);
            cos_sum += particle.weight * cos(particle.theta);
        }
        mean.theta = atan2(sin_sum, cos_sum);
        return mean;
    }

    bool cellInBounds(unsigned int x, unsigned int y, unsigned int width, unsigned int height)
    {
        return x < width && y < height;
    }

    std::vector<int> drawIndices(const std::vector<ParticleState>& particles, int count, std::mt19937& generator)
    {
        std::vector<double> proba;
        proba.reserve(particles.size());
        for (const ParticleState& particle : particles)
        {
            proba.push_back(particle.weight);
        }
        std::discrete_distribution<int> distribution(proba.begin(), proba.end());

        std::vector<int> indices;
        indices.reserve(count);
        // `generator` is used by reference: drawing from a copy would leave it unchanged, and the
        // motion model would then reuse the same random numbers
        std::generate_n(back_inserter(indices), count, [&distribution, &generator]() { return distribution(generator); });
        return indices;
    }

}  // namespace particle_filter
