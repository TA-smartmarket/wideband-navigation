#include "navigation/waypoint_manager.hpp"

#include <cmath>

#include "navigation/geometry.hpp"

namespace nav {

namespace {
/// Distance margin used by the "passed the waypoint plane" test.  Chosen
/// larger than typical UWB noise (0.05-0.20 m) so noise alone cannot advance
/// the route.
constexpr float kPassedMarginM = 0.05f;
}  // namespace

void WaypointManager::reset() {
    count_ = 0;
    index_ = 0;
    destination_node_id_ = -1;
    finished_ = false;
    has_entry_ = false;
}

void WaypointManager::setEntryPoint(const Position2D& position) {
    if (!isFinite(position.x_m) || !isFinite(position.y_m)) {
        return;
    }
    entry_ = position;
    has_entry_ = true;
}

bool WaypointManager::loadRoute(const Waypoint* waypoints, int count, int destination_node_id) {
    reset();
    if (waypoints == nullptr || count <= 0) {
        return false;
    }
    const int copied = count > kMaxRouteNodes ? kMaxRouteNodes : count;
    for (int i = 0; i < copied; ++i) {
        waypoints_[i] = waypoints[i];
    }
    count_ = copied;
    index_ = 0;
    destination_node_id_ = destination_node_id;
    finished_ = false;
    return true;
}

void WaypointManager::snapToNearestWaypoint(const Position2D& position) {
    if (count_ == 0 || finished_) {
        return;
    }
    // Never skip the destination itself; only leading intermediate waypoints.
    while (index_ < count_ - 1) {
        const Waypoint& candidate = waypoints_[index_];
        const Position2D node{candidate.x_m, candidate.y_m};
        if (euclideanDistance(position, node) <= config_.waypoint_tolerance_m) {
            ++index_;
        } else {
            break;
        }
    }
}

bool WaypointManager::hasPassedWaypoint(int index, const Position2D& position) const {
    if (index + 1 >= count_) {
        return false;  // the destination is only accepted by radius
    }
    const Waypoint& current = waypoints_[index];
    const Waypoint& following = waypoints_[index + 1];
    const float dir_x = following.x_m - current.x_m;
    const float dir_y = following.y_m - current.y_m;
    const float length = std::sqrt(dir_x * dir_x + dir_y * dir_y);
    if (!isFinite(length) || length <= 1.0e-6f) {
        return false;
    }
    // Projection of (position - current) on the unit direction of the segment.
    const float along = ((position.x_m - current.x_m) * dir_x +
                         (position.y_m - current.y_m) * dir_y) / length;
    if (along <= -kPassedMarginM) {
        return false;  // still behind the waypoint
    }
    // Perpendicular distance to the infinite line through the segment.
    const float perpendicular =
        std::fabs((position.x_m - current.x_m) * (-dir_y / length) +
                  (position.y_m - current.y_m) * (dir_x / length));
    // Accept when the trolley is clearly ahead of the waypoint and roughly on
    // the corridor towards the next node.
    return along >= config_.waypoint_tolerance_m * 0.5f &&
           perpendicular <= config_.max_cross_track_error_m;
}

WaypointProgress WaypointManager::update(const Position2D& position) {
    WaypointProgress progress;
    if (count_ == 0 || finished_) {
        return progress;
    }

    // A route may contain several waypoints that are already satisfied (for
    // example a dense cluster of nodes near the start); consume them all.
    while (index_ < count_) {
        const Waypoint& waypoint = waypoints_[index_];
        const Position2D node{waypoint.x_m, waypoint.y_m};
        const float distance = euclideanDistance(position, node);
        const float tolerance = toleranceForIndex(index_);
        const bool inside = distance <= tolerance;
        const bool passed = !inside && hasPassedWaypoint(index_, position);
        if (!inside && !passed) {
            break;
        }
        progress.waypoint_reached = true;
        progress.reached_node_id = waypoint.node_id;
        ++progress.advanced_count;
        if (isDestinationIndex(index_)) {
            progress.destination_reached = true;
            finished_ = true;
            break;  // index_ stays on the destination for telemetry
        }
        ++index_;
    }
    return progress;
}

const Waypoint* WaypointManager::current() const {
    if (count_ == 0 || index_ < 0 || index_ >= count_) {
        return nullptr;
    }
    return &waypoints_[index_];
}

const Waypoint* WaypointManager::next() const {
    const int next_index = index_ + 1;
    if (count_ == 0 || next_index >= count_) {
        return nullptr;
    }
    return &waypoints_[next_index];
}

int WaypointManager::activeSegmentIndex() const {
    if (count_ < 2 || finished_) {
        return -1;
    }
    // The segment being traversed is the leg leading *into* the current
    // waypoint.  Using `index_` here would measure the trolley against the leg
    // that starts at a waypoint it has not reached yet, which makes a correctly
    // tracking trolley look metres off-route.
    return index_ > 0 ? index_ - 1 : 0;
}

float WaypointManager::distanceToCurrent(const Position2D& position) const {
    const Waypoint* waypoint = current();
    if (waypoint == nullptr) {
        return 0.0f;
    }
    return euclideanDistance(position, Position2D{waypoint->x_m, waypoint->y_m});
}

float WaypointManager::remainingDistance(const Position2D& position) const {
    const Waypoint* waypoint = current();
    if (waypoint == nullptr) {
        return 0.0f;
    }
    float total = distanceToCurrent(position);
    for (int i = index_; i + 1 < count_; ++i) {
        const float dx = waypoints_[i + 1].x_m - waypoints_[i].x_m;
        const float dy = waypoints_[i + 1].y_m - waypoints_[i].y_m;
        total += std::sqrt(dx * dx + dy * dy);
    }
    return total;
}

float WaypointManager::crossTrackError(const Position2D& position) const {
    if (count_ == 0) {
        return 0.0f;
    }
    float best = -1.0f;

    // While the trolley is still on the entry leg (start position -> first
    // waypoint) the reference includes that leg, not only the first graph
    // segment: the route the trolley was told to follow starts where it is.
    if (has_entry_ && index_ == 0) {
        const Position2D first{waypoints_[0].x_m, waypoints_[0].y_m};
        if (euclideanDistance(position, first) > config_.waypoint_tolerance_m) {
            best = pointToSegmentDistance(position, entry_, first);
        }
    }

    // Every remaining leg, including the one leading into the current waypoint.
    // Taking the minimum is correct for a tracked polyline: the trolley is "on
    // route" when it is close to *any* leg it still has to travel.
    const int first_segment = index_ > 0 ? index_ - 1 : 0;
    for (int i = first_segment; i + 1 < count_; ++i) {
        const float distance = nav::distanceToSegment(position, waypoints_, count_, i);
        if (best < 0.0f || distance < best) {
            best = distance;
        }
    }

    if (best < 0.0f) {
        // Single node route: fall back to the distance to that node.
        return distanceToCurrent(position);
    }
    return best;
}

float WaypointManager::distanceToDestination(const Position2D& position) const {
    if (count_ == 0) {
        return 0.0f;
    }
    const Waypoint& destination = waypoints_[count_ - 1];
    return euclideanDistance(position, Position2D{destination.x_m, destination.y_m});
}

Position2D WaypointManager::targetPoint(const Position2D& position) const {
    const Waypoint* waypoint = current();
    if (waypoint == nullptr) {
        return position;
    }
    Position2D target{waypoint->x_m, waypoint->y_m};
    if (!config_.enable_lookahead || config_.lookahead_distance_m <= 0.0f) {
        return target;
    }
    if (finished_ || index_ + 1 >= count_) {
        return target;
    }

    // Pure-pursuit style lookahead: walk along the remaining polyline and stop
    // at the point `lookahead_distance_m` ahead of the projection of the
    // current position onto the route.
    const Position2D current_node{waypoint->x_m, waypoint->y_m};
    const Position2D next_node{waypoints_[index_ + 1].x_m, waypoints_[index_ + 1].y_m};
    const Position2D projected = closestPointOnSegment(position, current_node, next_node);
    const float remaining_on_segment = euclideanDistance(projected, next_node);
    if (remaining_on_segment >= config_.lookahead_distance_m) {
        const float segment_length = euclideanDistance(current_node, next_node);
        if (segment_length <= 1.0e-6f) {
            return target;
        }
        const float travelled = euclideanDistance(current_node, projected);
        const float t = (travelled + config_.lookahead_distance_m) / segment_length;
        return lerp(current_node, next_node, t);
    }
    return next_node;
}

}  // namespace nav
