#pragma once

#include <algorithm>
#include <cmath>

namespace tinympc_chicane
{

constexpr double kFirstCornerX = 1.50;
constexpr double kSecondCornerY = 1.00;
constexpr double kFinishX = 3.00;
constexpr double kCorridorHalfWidth = 0.18;
constexpr double kFirstSegmentDuration = 1.60;
constexpr double kSecondSegmentDuration = 1.20;
constexpr double kThirdSegmentDuration = 1.60;
// The geometric corridor legs overlap at each corner. Delay the active box
// handoff so a real PX4 inner loop may occupy that overlap without making the
// next 20 ms prediction artificially infeasible.
constexpr double kConstraintTransitionGrace = 0.30;
constexpr double kFirstCornerTime = kFirstSegmentDuration;
constexpr double kSecondCornerTime = kFirstSegmentDuration + kSecondSegmentDuration;
constexpr double kFinishTime = kFirstSegmentDuration + kSecondSegmentDuration + kThirdSegmentDuration;

struct Sample {
    double x;
    double y;
    double vx;
    double vy;
    int segment;
};

struct Bounds {
    double xMinimum;
    double xMaximum;
    double yMinimum;
    double yMaximum;
};

inline Sample sample(double time)
{
    const double boundedTime = std::max(0.0, time);

    if (boundedTime < kFirstCornerTime) {
        const double progress = boundedTime / kFirstSegmentDuration;
        return {kFirstCornerX * progress, 0.0,
                kFirstCornerX / kFirstSegmentDuration, 0.0, 0};
    }

    if (boundedTime < kSecondCornerTime) {
        const double progress = (boundedTime - kFirstCornerTime) /
                                kSecondSegmentDuration;
        return {kFirstCornerX, kSecondCornerY * progress, 0.0,
                kSecondCornerY / kSecondSegmentDuration, 1};
    }

    if (boundedTime < kFinishTime) {
        const double progress = (boundedTime - kSecondCornerTime) /
                                kThirdSegmentDuration;
        return {kFirstCornerX + (kFinishX - kFirstCornerX) * progress,
                kSecondCornerY,
                (kFinishX - kFirstCornerX) / kThirdSegmentDuration,
                0.0, 2};
    }

    return {kFinishX, kSecondCornerY, 0.0, 0.0, 2};
}

inline Bounds boundsForSegment(int segment)
{
    if (segment == 0) {
        return {-0.30, kFirstCornerX + kCorridorHalfWidth,
                -kCorridorHalfWidth, kCorridorHalfWidth};
    }

    if (segment == 1) {
        return {kFirstCornerX - kCorridorHalfWidth,
                kFirstCornerX + kCorridorHalfWidth,
                -kCorridorHalfWidth,
                kSecondCornerY + kCorridorHalfWidth};
    }

    return {kFirstCornerX - kCorridorHalfWidth, kFinishX + 0.30,
            kSecondCornerY - kCorridorHalfWidth,
            kSecondCornerY + kCorridorHalfWidth};
}

inline Bounds planningBounds(double time)
{
    return boundsForSegment(sample(time - kConstraintTransitionGrace).segment);
}

inline double distanceOutsideRectangle(double x, double y, const Bounds &bounds)
{
    const double dx = std::max({bounds.xMinimum - x, 0.0, x - bounds.xMaximum});
    const double dy = std::max({bounds.yMinimum - y, 0.0, y - bounds.yMaximum});
    return std::sqrt(dx * dx + dy * dy);
}

inline double corridorViolation(double x, double y)
{
    return std::min({distanceOutsideRectangle(x, y, boundsForSegment(0)),
                     distanceOutsideRectangle(x, y, boundsForSegment(1)),
                     distanceOutsideRectangle(x, y, boundsForSegment(2))});
}

} // namespace tinympc_chicane
