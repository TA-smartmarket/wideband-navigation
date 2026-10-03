// Dijkstra shortest path, implemented from scratch on a binary heap.
//
// Complexity: O((V + E) log V) using a binary min-heap with decrease-key via
// lazy insertion (a node may appear several times in the heap; the stale entry
// is skipped when popped because the recorded distance is already better).
//
// No third-party shortest-path library is used.
#pragma once

#include "navigation/graph.hpp"
#include "navigation/types.hpp"

namespace nav {

/// Dijkstra shortest path between two node ids.
///
/// Handles: start == destination (single node route, distance 0), unknown
/// start/destination, empty graph and disconnected pairs.
PathResult dijkstra(const Graph& graph, int start_node_id, int destination_node_id);

/// Convenience overload returning the route as waypoints with coordinates.
int pathToWaypoints(const Graph& graph,
                    const PathResult& path,
                    Waypoint* out_waypoints,
                    int max_waypoints);

/// Total length of a polyline through `count` waypoints (meters).
float polylineLength(const Waypoint* waypoints, int count);

/// Distance from `position` to the polyline segment at index `segment_index`.
float distanceToSegment(const Position2D& position, const Waypoint* waypoints, int count,
                        int segment_index);

/// Minimum distance from `position` to any segment of the polyline.
float crossTrackError(const Position2D& position, const Waypoint* waypoints, int count);

}  // namespace nav
