#include "navigation/geometry.hpp"

#include <cmath>

namespace nav {

float normalizeAngle(float angle) {
    if (!isFinite(angle)) {
        return 0.0f;
    }
    // fmod keeps the magnitude bounded; the two corrections then place the
    // result inside [-pi, +pi] without a loop (input is already bounded).
    float wrapped = std::fmod(angle, kTwoPi);
    if (wrapped > kPi) {
        wrapped -= kTwoPi;
    } else if (wrapped < -kPi) {
        wrapped += kTwoPi;
    }
    return wrapped;
}

float shortestAngularDistance(float from, float to) {
    return normalizeAngle(to - from);
}

float calculateBearing(const Position2D& current, const Position2D& target) {
    const float dx = target.x_m - current.x_m;
    const float dy = target.y_m - current.y_m;
    if (!isFinite(dx) || !isFinite(dy)) {
        return 0.0f;
    }
    if (dx == 0.0f && dy == 0.0f) {
        return 0.0f;
    }
    return std::atan2(dy, dx);
}

float euclideanDistanceSquared(const Position2D& a, const Position2D& b) {
    const float dx = b.x_m - a.x_m;
    const float dy = b.y_m - a.y_m;
    if (!isFinite(dx) || !isFinite(dy)) {
        return 0.0f;
    }
    return dx * dx + dy * dy;
}

float euclideanDistance(const Position2D& a, const Position2D& b) {
    return std::sqrt(euclideanDistanceSquared(a, b));
}

Position2D lerp(const Position2D& a, const Position2D& b, float t) {
    const float tc = clampf(t, 0.0f, 1.0f);
    Position2D out;
    out.x_m = a.x_m + (b.x_m - a.x_m) * tc;
    out.y_m = a.y_m + (b.y_m - a.y_m) * tc;
    return out;
}

Position2D closestPointOnSegment(const Position2D& p, const Position2D& a, const Position2D& b) {
    const float dx = b.x_m - a.x_m;
    const float dy = b.y_m - a.y_m;
    const float len_sq = dx * dx + dy * dy;
    if (!isFinite(len_sq) || len_sq <= 1.0e-12f) {
        return a;  // degenerate segment: the endpoint is the closest point
    }
    float t = ((p.x_m - a.x_m) * dx + (p.y_m - a.y_m) * dy) / len_sq;
    t = clampf(t, 0.0f, 1.0f);
    return lerp(a, b, t);
}

float pointToSegmentDistance(const Position2D& p, const Position2D& a, const Position2D& b) {
    const Position2D closest = closestPointOnSegment(p, a, b);
    return euclideanDistance(p, closest);
}

}  // namespace nav
