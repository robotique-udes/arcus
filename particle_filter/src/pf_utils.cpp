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

    Spread weightedSpread(const std::vector<ParticleState>& particles, const Pose2D& mean)
    {
        double squared_sum = 0.0;
        double sin_sum = 0.0;
        double cos_sum = 0.0;
        for (const ParticleState& particle : particles)
        {
            double dx = particle.x - mean.x;
            double dy = particle.y - mean.y;
            squared_sum += particle.weight * (dx * dx + dy * dy);
            sin_sum += particle.weight * sin(particle.theta);
            cos_sum += particle.weight * cos(particle.theta);
        }
        // Length of the mean heading vector: 1 when all headings agree, 0 when they cancel out
        double resultant = std::clamp(std::hypot(sin_sum, cos_sum), 1e-12, 1.0);

        Spread spread;
        spread.position_m = std::sqrt(squared_sum);
        spread.heading_rad = std::sqrt(-2.0 * std::log(resultant));
        return spread;
    }

    DiffModel decomposeMotion(const Pose2D& delta, double standstill_m)
    {
        DiffModel motion;
        bool reversing = delta.x < 0.0;

        motion.trans = !reversing ? std::hypot(delta.x, delta.y) : -std::hypot(delta.x, delta.y);
        if (std::fabs(motion.trans) < standstill_m)
        {
            motion.trans = 0.0;
            motion.rot1 = 0.0;
        }
        else 
        {
            motion.rot1 = !reversing ? std::atan2(delta.y, delta.x) : std::atan2(-delta.y, -delta.x);
            motion.rot2 = wrapAngle(delta.theta - motion.rot1);
        }

        return motion;
    }

    namespace
    {
        struct Point2D
        {
            double x;
            double y;
        };

        // Where a beam at `angle` leaves a box with corners (+-aspect_ratio, +-1) centred on the lidar
        Point2D boxHit(double angle, double aspect_ratio)
        {
            double c = std::cos(angle);
            double s = std::sin(angle);

            // Distance along the beam to the front/back wall and to the side wall; a beam parallel to a wall never
            // reaches it
            double to_end_wall = std::fabs(c) > 1e-12 ? aspect_ratio / std::fabs(c) : HUGE_VAL;
            double to_side_wall = std::fabs(s) > 1e-12 ? 1.0 / std::fabs(s) : HUGE_VAL;

            // The beam leaves through whichever wall it reaches first
            double t = std::min(to_end_wall, to_side_wall);
            return Point2D{t * c, t * s};
        }

        double distance(const Point2D& a, const Point2D& b)
        {
            return std::hypot(b.x - a.x, b.y - a.y);
        }
    }  // namespace

    std::vector<int> boxedBeamIndices(double angle_min, double angle_increment, int num_ranges, int num_beams,
                                      double aspect_ratio)
    {
        std::vector<int> indices;
        if (num_ranges <= 0 || num_beams <= 0)
            return indices;

        // The middle beam points forward and is always used
        const int middle = num_ranges / 2;
        const int last = num_ranges - 1;
        const int per_side = std::min((num_beams - 1) / 2, last - middle);
        indices.push_back(middle);
        if (per_side == 0)
            return indices;

        // Hit points of the beams from the middle to the end of the scan. Only this side is walked; the scan is
        // symmetric, so the other side is its mirror image
        std::vector<Point2D> hits;
        for (int i = middle; i <= last; i++)
        {
            hits.push_back(boxHit(angle_min + angle_increment * i, aspect_ratio));
        }

        // Length of wall this side of the scan covers, and the wall distance between two picked beams
        double wall_length = 0.0;
        for (size_t k = 1; k < hits.size(); k++)
        {
            wall_length += distance(hits[k - 1], hits[k]);
        }
        const double spacing = wall_length / per_side;

        // Walk outward from the middle, picking a beam each time another `spacing` of wall has been covered
        std::vector<int> side;
        double covered = 0.0;
        for (size_t k = 1; k < hits.size() && (int)side.size() < per_side; k++)
        {
            covered += distance(hits[k - 1], hits[k]);
            if (covered >= spacing)
            {
                side.push_back(middle + (int)k);
                covered -= spacing;  // keep the overshoot, so the gaps don't drift
            }
        }
        // Rounding can leave the outermost pick just short of the scan's edge
        if ((int)side.size() < per_side && (side.empty() || side.back() != last))
        {
            side.push_back(last);
        }

        // Each pick and its mirror image on the other side
        for (int i : side)
        {
            indices.push_back(i);
            indices.push_back(2 * middle - i);
        }

        std::sort(indices.begin(), indices.end());
        return indices;
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

    std::vector<int> systematicIndices(const std::vector<ParticleState>& particles, int count, std::mt19937& generator)
    {
        std::vector<int> indices;
        if (particles.empty() || count <= 0)
            return indices;
        indices.reserve(count);

        // Teeth of a comb `step` apart, shifted by one random offset in [0, step)
        const double step = 1.0 / count;
        double target = std::uniform_real_distribution<double>(0.0, step)(generator);
        // Right edge of the current particle's segment on [0, 1]
        double cumulative = particles[0].weight;
        size_t source = 0;
        for (int i = 0; i < count; i++)
        {
            // Move to the segment this tooth lands in; the bound guards against weights summing to slightly under 1
            while (target > cumulative && source + 1 < particles.size())
            {
                source++;
                cumulative += particles[source].weight;
            }
            indices.push_back(static_cast<int>(source));
            target += step;
        }
        return indices;
    }

    double calculateESS(const std::vector<ParticleState>& particles)
    {
        double weight_sum_sq = 0.0;
        for (const ParticleState& particle : particles)
        {
            weight_sum_sq += particle.weight * particle.weight;
        }
        return weight_sum_sq > 0.0 ? 1.0 / weight_sum_sq : 0.0;
    }

}  // namespace particle_filter
