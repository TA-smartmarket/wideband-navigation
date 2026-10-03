// Waypoint tracking: owns the ordered waypoint list produced from the Dijkstra
// route and decides when a waypoint (or the final destination) is reached.
//
// Anti-oscillation: the index only ever increases and a waypoint is accepted
// either when the trolley is inside the acceptance radius or when it has
// provably driven past the waypoint plane.  A single noisy UWB sample therefore
// cannot bounce the follower between two waypoints.
#pragma once

#include "navigation/dijkstra.hpp"
#include "navigation/navigation_config.hpp"
#include "navigation/types.hpp"

namespace nav {

struct WaypointProgress {
    bool waypoint_reached{false};
    int reached_node_id{-1};
    bool destination_reached{false};
    int advanced_count{0};
    /// True when the route was re-snapped because the trolley was already
    /// inside the acceptance radius of one or more leading waypoints.
    bool skipped_leading_waypoints{false};
};

class WaypointManager {
public:
    WaypointManager() = default;
    explicit WaypointManager(const PathConfig& config) : config_(config) {}

    void setConfig(const PathConfig& config) { config_ = config; }
    const PathConfig& config() const { return config_; }

    /// Reset to "no route".
    void reset();

    /// Load a route.  `count` must be >= 1 and the last waypoint is treated as
    /// the destination.  Returns false for an empty route.
    bool loadRoute(const Waypoint* waypoints, int count, int destination_node_id);

    /// Record the position at which the route was planned.
    ///
    /// The entry leg (entry point -> first waypoint) is part of the route the
    /// trolley is expected to travel, so cross-track error must be measured
    /// against it.  Without this the deviation monitor would report a large
    /// error for a trolley that is simply still driving towards the first
    /// waypoint from where it started, and would replan in a loop.
    void setEntryPoint(const Position2D& position);

    /// Advance the index past leading waypoints the trolley is already at
    /// (so the trolley does not drive back to the node it started from).
    void snapToNearestWaypoint(const Position2D& position);

    /// Process a new position and report arrival events.
    WaypointProgress update(const Position2D& position);

    bool hasRoute() const { return count_ > 0; }
    bool finished() const { return finished_; }
    int currentIndex() const { return index_; }
    int waypointCount() const { return count_; }
    int destinationNodeId() const { return destination_node_id_; }
    const Waypoint* current() const;
    const Waypoint* next() const;
    const Waypoint* waypoints() const { return waypoints_; }

    /// Segment index of the polyline the trolley is currently following
    /// (segment from waypoint[i] to waypoint[i+1]); -1 when finished.
    int activeSegmentIndex() const;

    float distanceToCurrent(const Position2D& position) const;

    /// Remaining path length: position -> current waypoint -> ... -> destination.
    float remainingDistance(const Position2D& position) const;

    /// Perpendicular distance to the remaining polyline.
    float crossTrackError(const Position2D& position) const;

    /// Distance from the trolley to the final destination coordinate.
    float distanceToDestination(const Position2D& position) const;

    /// Point the follower should steer towards (waypoint centre, or a point
    /// `lookahead_distance_m` ahead along the route when lookahead is enabled).
    Position2D targetPoint(const Position2D& position) const;

private:
    bool isDestinationIndex(int index) const { return index >= count_ - 1; }
    float toleranceForIndex(int index) const {
        return isDestinationIndex(index) ? config_.destination_tolerance_m
                                         : config_.waypoint_tolerance_m;
    }
    bool hasPassedWaypoint(int index, const Position2D& position) const;

    Waypoint waypoints_[kMaxRouteNodes]{};
    int count_{0};
    int index_{0};
    int destination_node_id_{-1};
    bool finished_{false};
    Position2D entry_{};
    bool has_entry_{false};
    PathConfig config_{};
};

}  // namespace nav
