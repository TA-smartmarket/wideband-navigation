// Portable geometry helpers: angles, distances, bearings, point-to-segment.
//
// All inputs/outputs are radians and meters.  Every routine is defensive about
// NaN/inf and degenerate (zero length) inputs because UWB positions can be
// noisy and a single NaN must not poison the control loop.
#pragma once

#include "navigation/types.hpp"

namespace nav {

inline constexpr float kPi = 3.14159265358979323846f;
inline constexpr float kTwoPi = 6.28318530717958647692f;

/// Wrap an angle into [-pi, +pi].  Non-finite input yields 0.
float normalizeAngle(float angle);

/// Shortest signed rotation from `from` to `to`, in [-pi, +pi].
float shortestAngularDistance(float from, float to);

/// Angle from `current` to `target` measured counter-clockwise from +X.
/// Returns 0 when the two points coincide.
float calculateBearing(const Position2D& current, const Position2D& target);

/// Euclidean distance between two points.
float euclideanDistance(const Position2D& a, const Position2D& b);

/// Squared Euclidean distance (avoids a sqrt when only ordering matters).
float euclideanDistanceSquared(const Position2D& a, const Position2D& b);

/// Perpendicular distance from `p` to the *finite* segment [a, b].
/// Degenerate segments fall back to the point-to-point distance.
float pointToSegmentDistance(const Position2D& p, const Position2D& a, const Position2D& b);

/// Closest point on segment [a, b] to `p`.  Writes the projected point.
Position2D closestPointOnSegment(const Position2D& p, const Position2D& a, const Position2D& b);

/// Point at fraction `t` (0..1, clamped) along segment [a, b].
Position2D lerp(const Position2D& a, const Position2D& b, float t);

/// True when the value is neither NaN nor infinite.
inline bool isFinite(float value) {
    return value == value && value <= 3.0e38f && value >= -3.0e38f;
}

/// Clamp helper shared across the core.
inline float clampf(float value, float lo, float hi) {
    if (!isFinite(value)) {
        return lo;
    }
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

}  // namespace nav
