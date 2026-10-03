#include "navigation/dijkstra.hpp"

#include <cmath>

namespace nav {

namespace {

constexpr float kInfinity = 3.0e38f;

/// Minimal binary min-heap keyed by (distance, node_index).
/// Capacity is bounded by the node table plus the number of relaxed edges.
struct HeapEntry {
    float distance;
    int node_index;
};

class MinHeap {
public:
    explicit MinHeap(HeapEntry* storage, int capacity) : storage_(storage), capacity_(capacity) {}

    bool empty() const { return size_ == 0; }

    void push(float distance, int node_index) {
        if (size_ >= capacity_) {
            return;  // bounded: cannot overflow, worst case drops a stale entry
        }
        int i = size_++;
        storage_[i] = {distance, node_index};
        while (i > 0) {
            const int parent = (i - 1) / 2;
            if (storage_[parent].distance <= storage_[i].distance) {
                break;
            }
            const HeapEntry tmp = storage_[parent];
            storage_[parent] = storage_[i];
            storage_[i] = tmp;
            i = parent;
        }
    }

    HeapEntry pop() {
        const HeapEntry top = storage_[0];
        --size_;
        if (size_ > 0) {
            storage_[0] = storage_[size_];
            int i = 0;
            for (;;) {
                const int left = 2 * i + 1;
                const int right = left + 1;
                int smallest = i;
                if (left < size_ && storage_[left].distance < storage_[smallest].distance) {
                    smallest = left;
                }
                if (right < size_ && storage_[right].distance < storage_[smallest].distance) {
                    smallest = right;
                }
                if (smallest == i) {
                    break;
                }
                const HeapEntry tmp = storage_[smallest];
                storage_[smallest] = storage_[i];
                storage_[i] = tmp;
                i = smallest;
            }
        }
        return top;
    }

private:
    HeapEntry* storage_;
    int capacity_;
    int size_{0};
};

}  // namespace

PathResult dijkstra(const Graph& graph, int start_node_id, int destination_node_id) {
    PathResult result;
    result.clear();

    if (graph.nodeCount() == 0) {
        result.error = PathError::EMPTY_GRAPH;
        return result;
    }
    const int start_index = graph.indexOfNode(start_node_id);
    if (start_index < 0) {
        result.error = PathError::INVALID_START;
        return result;
    }
    const int dest_index = graph.indexOfNode(destination_node_id);
    if (dest_index < 0) {
        result.error = PathError::INVALID_DESTINATION;
        return result;
    }

    // A zero length route is a valid answer, not an error: the trolley is
    // already at the destination node.
    if (start_index == dest_index) {
        result.success = true;
        result.error = PathError::NONE;
        result.node_ids[0] = start_node_id;
        result.node_count = 1;
        result.total_distance_m = 0.0f;
        return result;
    }

    const int node_count = graph.nodeCount();
    float dist[kMaxGraphNodes];
    int previous[kMaxGraphNodes];
    bool settled[kMaxGraphNodes];
    for (int i = 0; i < node_count; ++i) {
        dist[i] = kInfinity;
        previous[i] = -1;
        settled[i] = false;
    }

    // Lazy-deletion heap: at most one entry per relaxation, i.e. one per
    // directed edge plus the initial push.
    //
    // The scratch buffer is a function-local static rather than a stack array
    // because ~7 KB does not fit comfortably in an ESP32 task stack.  Path
    // planning is owned exclusively by the navigation task (never called from
    // the telemetry or position tasks), so this is single-threaded in practice;
    // dijkstra() must not be called concurrently.
    static HeapEntry heap_storage[kMaxGraphEdges * 2 + kMaxGraphNodes + 8];
    MinHeap heap(heap_storage, static_cast<int>(sizeof(heap_storage) / sizeof(heap_storage[0])));

    dist[start_index] = 0.0f;
    heap.push(0.0f, start_index);

    while (!heap.empty()) {
        const HeapEntry entry = heap.pop();
        const int u = entry.node_index;
        if (settled[u] || entry.distance > dist[u]) {
            continue;  // stale heap entry
        }
        settled[u] = true;
        ++result.expanded_nodes;

        if (u == dest_index) {
            break;  // Dijkstra settles the destination with its final cost
        }

        for (int adj = graph.firstAdjacencyOf(u); adj >= 0; adj = graph.nextAdjacencyOf(u, adj)) {
            const int v = graph.adjacencyTargetNodeIndex(adj);
            if (v < 0 || settled[v]) {
                continue;
            }
            const float weight = graph.adjacencyWeight(adj);
            if (!isFinite(weight) || weight < 0.0f) {
                continue;  // malformed edge: never relax through it
            }
            const float candidate = dist[u] + weight;
            // Deterministic tie-break: on equal total cost prefer the path whose
            // predecessor has the smaller node id.  A uniform grid admits several
            // equal-length optimal routes, and without this rule the chosen route
            // would depend on binary-heap pop order (i.e. on insertion order).
            const int candidate_predecessor_id = graph.nodeIdAtIndex(u);
            const int current_predecessor_id =
                previous[v] >= 0 ? graph.nodeIdAtIndex(previous[v]) : -1;
            const bool better = candidate < dist[v];
            const bool equal_but_preferred = candidate == dist[v] && previous[v] >= 0 &&
                                             candidate_predecessor_id < current_predecessor_id;
            if (better || equal_but_preferred) {
                dist[v] = candidate;
                previous[v] = u;
                heap.push(candidate, v);
            }
        }
    }

    if (dist[dest_index] >= kInfinity) {
        result.error = PathError::NO_ROUTE;
        return result;
    }

    // Reconstruct backwards, then reverse into the result array.
    int reverse_ids[kMaxRouteNodes];
    int count = 0;
    int cursor = dest_index;
    while (cursor >= 0 && count < kMaxRouteNodes) {
        reverse_ids[count++] = graph.nodeIdAtIndex(cursor);
        if (cursor == start_index) {
            break;
        }
        cursor = previous[cursor];
    }

    result.success = true;
    result.error = PathError::NONE;
    result.node_count = count;
    result.total_distance_m = dist[dest_index];
    for (int i = 0; i < count; ++i) {
        result.node_ids[i] = reverse_ids[count - 1 - i];
    }
    return result;
}

int pathToWaypoints(const Graph& graph,
                    const PathResult& path,
                    Waypoint* out_waypoints,
                    int max_waypoints) {
    if (out_waypoints == nullptr || max_waypoints <= 0 || !path.success) {
        return 0;
    }
    int written = 0;
    for (int i = 0; i < path.node_count && written < max_waypoints; ++i) {
        const GraphNode* node = graph.findNode(path.node_ids[i]);
        if (node == nullptr) {
            continue;
        }
        out_waypoints[written].node_id = node->id;
        out_waypoints[written].x_m = node->x_m;
        out_waypoints[written].y_m = node->y_m;
        ++written;
    }
    return written;
}

float polylineLength(const Waypoint* waypoints, int count) {
    if (waypoints == nullptr || count < 2) {
        return 0.0f;
    }
    float total = 0.0f;
    for (int i = 1; i < count; ++i) {
        const float dx = waypoints[i].x_m - waypoints[i - 1].x_m;
        const float dy = waypoints[i].y_m - waypoints[i - 1].y_m;
        total += std::sqrt(dx * dx + dy * dy);
    }
    return total;
}

float distanceToSegment(const Position2D& position, const Waypoint* waypoints, int count,
                        int segment_index) {
    if (waypoints == nullptr || segment_index < 0 || segment_index + 1 >= count) {
        return 0.0f;
    }
    Position2D a{waypoints[segment_index].x_m, waypoints[segment_index].y_m};
    Position2D b{waypoints[segment_index + 1].x_m, waypoints[segment_index + 1].y_m};
    return pointToSegmentDistance(position, a, b);
}

float crossTrackError(const Position2D& position, const Waypoint* waypoints, int count) {
    if (waypoints == nullptr || count < 2) {
        return 0.0f;
    }
    float best = -1.0f;
    for (int i = 0; i + 1 < count; ++i) {
        const float d = distanceToSegment(position, waypoints, count, i);
        if (best < 0.0f || d < best) {
            best = d;
        }
    }
    return best < 0.0f ? 0.0f : best;
}

}  // namespace nav
