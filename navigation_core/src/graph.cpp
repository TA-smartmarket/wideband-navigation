#include "navigation/graph.hpp"

#include <cmath>

namespace nav {

namespace {
constexpr float kMinEdgeWeight = 1.0e-4f;  // below this a self-loop is implied
}

const char* toString(NodeType type) {
    switch (type) {
        case NodeType::INTERSECTION: return "intersection";
        case NodeType::AISLE: return "aisle";
        case NodeType::DESTINATION: return "destination";
        case NodeType::PARKING: return "parking";
        case NodeType::ENTRY: return "entry";
        case NodeType::EXIT: return "exit";
        case NodeType::CHECKOUT: return "checkout";
        default: return "unknown";
    }
}

NodeType nodeTypeFromString(const char* text) {
    if (text == nullptr) return NodeType::UNKNOWN;
    if (std::strcmp(text, "intersection") == 0) return NodeType::INTERSECTION;
    if (std::strcmp(text, "aisle") == 0) return NodeType::AISLE;
    if (std::strcmp(text, "destination") == 0) return NodeType::DESTINATION;
    if (std::strcmp(text, "parking") == 0) return NodeType::PARKING;
    if (std::strcmp(text, "entry") == 0) return NodeType::ENTRY;
    if (std::strcmp(text, "exit") == 0) return NodeType::EXIT;
    if (std::strcmp(text, "checkout") == 0) return NodeType::CHECKOUT;
    return NodeType::UNKNOWN;
}

const char* toString(GraphIssue issue) {
    switch (issue) {
        case GraphIssue::NONE: return "ok";
        case GraphIssue::EMPTY_GRAPH: return "graph is empty";
        case GraphIssue::DUPLICATE_NODE_ID: return "duplicate node id";
        case GraphIssue::INVALID_NODE_ID: return "invalid node id";
        case GraphIssue::INVALID_COORDINATE: return "node coordinate is not finite";
        case GraphIssue::EDGE_UNKNOWN_NODE: return "edge references a nonexistent node";
        case GraphIssue::NEGATIVE_EDGE_WEIGHT: return "negative edge weight";
        case GraphIssue::SELF_LOOP_NOT_ALLOWED: return "self loop is not allowed";
        case GraphIssue::EDGE_TABLE_FULL: return "edge table is full";
        case GraphIssue::NODE_TABLE_FULL: return "node table is full";
        case GraphIssue::DUPLICATE_EDGE: return "duplicate edge";
        default: return "unknown graph issue";
    }
}

void Graph::clear() {
    node_count_ = 0;
    edge_count_ = 0;
    directed_edge_count_ = 0;
    adjacency_count_ = 0;
    for (int i = 0; i < kMaxGraphNodes; ++i) {
        adjacency_head_[i] = -1;
    }
}

void Graph::clearEdges() {
    edge_count_ = 0;
    directed_edge_count_ = 0;
    adjacency_count_ = 0;
    for (int i = 0; i < kMaxGraphNodes; ++i) {
        adjacency_head_[i] = -1;
    }
}

int Graph::indexOfNode(int id) const {
    for (int i = 0; i < node_count_; ++i) {
        if (nodes_[i].id == id) {
            return i;
        }
    }
    return -1;
}

const GraphNode* Graph::findNode(int id) const {
    const int index = indexOfNode(id);
    return index < 0 ? nullptr : &nodes_[index];
}

int Graph::addNode(int id, float x_m, float y_m, NodeType type) {
    if (id < 0) {
        return -1;
    }
    if (node_count_ >= kMaxGraphNodes) {
        return -1;
    }
    if (indexOfNode(id) >= 0) {
        return -1;  // duplicate id
    }
    if (!isFinite(x_m) || !isFinite(y_m)) {
        return -1;
    }
    GraphNode& node = nodes_[node_count_];
    node.id = id;
    node.x_m = x_m;
    node.y_m = y_m;
    node.type = type;
    adjacency_head_[node_count_] = -1;
    return node_count_++;
}

int Graph::addEdge(int from, int to, float weight_m, bool bidirectional) {
    const int from_index = indexOfNode(from);
    const int to_index = indexOfNode(to);
    if (from_index < 0 || to_index < 0) {
        return -1;
    }
    if (from_index == to_index) {
        return -1;  // self loops are rejected by design
    }
    if (edge_count_ >= kMaxGraphEdges) {
        return -1;
    }
    // Reject a duplicate in the same direction to keep validation deterministic.
    for (int i = 0; i < edge_count_; ++i) {
        if (edges_[i].from == from && edges_[i].to == to) {
            return -1;
        }
    }

    float weight = weight_m;
    if (weight < 0.0f) {
        // Automatic Euclidean weight when the caller omits `weight_m`.
        const GraphNode& a = nodes_[from_index];
        const GraphNode& b = nodes_[to_index];
        const float dx = b.x_m - a.x_m;
        const float dy = b.y_m - a.y_m;
        weight = std::sqrt(dx * dx + dy * dy);
    }

    GraphEdge& edge = edges_[edge_count_];
    edge.from = from;
    edge.to = to;
    edge.weight_m = weight;
    edge.bidirectional = bidirectional;

    const int edge_index = edge_count_++;
    // Forward adjacency entry.
    if (adjacency_count_ < kMaxGraphEdges * 2) {
        Adjacency& fwd = adjacency_[adjacency_count_];
        fwd.to_node_index = to_index;
        fwd.weight_m = weight;
        fwd.edge_index = edge_index;
        fwd.next_adjacency = adjacency_head_[from_index];
        adjacency_head_[from_index] = adjacency_count_;
        ++adjacency_count_;
        ++directed_edge_count_;
    }
    if (bidirectional && adjacency_count_ < kMaxGraphEdges * 2) {
        Adjacency& rev = adjacency_[adjacency_count_];
        rev.to_node_index = from_index;
        rev.weight_m = weight;
        rev.edge_index = edge_index;
        rev.next_adjacency = adjacency_head_[to_index];
        adjacency_head_[to_index] = adjacency_count_;
        ++adjacency_count_;
        ++directed_edge_count_;
    }
    return edge_index;
}

int Graph::firstAdjacencyOf(int node_index) const {
    if (node_index < 0 || node_index >= node_count_) {
        return -1;
    }
    return adjacency_head_[node_index];
}

int Graph::nextAdjacencyOf(int node_index, int adjacency_index) const {
    (void)node_index;
    if (adjacency_index < 0 || adjacency_index >= adjacency_count_) {
        return -1;
    }
    return adjacency_[adjacency_index].next_adjacency;
}

int Graph::adjacencyTargetNodeIndex(int adjacency_index) const {
    if (adjacency_index < 0 || adjacency_index >= adjacency_count_) {
        return -1;
    }
    return adjacency_[adjacency_index].to_node_index;
}

float Graph::adjacencyWeight(int adjacency_index) const {
    if (adjacency_index < 0 || adjacency_index >= adjacency_count_) {
        return 0.0f;
    }
    return adjacency_[adjacency_index].weight_m;
}

int Graph::nodeIdAtIndex(int node_index) const {
    if (node_index < 0 || node_index >= node_count_) {
        return -1;
    }
    return nodes_[node_index].id;
}

NearestNodeResult Graph::findNearestNode(const Position2D& position, float max_distance_m) const {
    NearestNodeResult result;
    if (!isFinite(position.x_m) || !isFinite(position.y_m) || node_count_ == 0) {
        return result;
    }
    float best_sq = -1.0f;
    for (int i = 0; i < node_count_; ++i) {
        const float dx = nodes_[i].x_m - position.x_m;
        const float dy = nodes_[i].y_m - position.y_m;
        const float dist_sq = dx * dx + dy * dy;
        if (best_sq < 0.0f || dist_sq < best_sq) {
            best_sq = dist_sq;
            result.node_id = nodes_[i].id;
            result.distance_m = std::sqrt(dist_sq);
        }
    }
    if (result.node_id >= 0) {
        if (max_distance_m < 0.0f || result.distance_m <= max_distance_m) {
            result.found = true;
        } else {
            result.found = false;  // too far to snap safely
        }
    }
    return result;
}

GraphValidation Graph::validate() const {
    GraphValidation validation;
    if (node_count_ == 0) {
        validation.issue = GraphIssue::EMPTY_GRAPH;
        return validation;
    }
    for (int i = 0; i < node_count_; ++i) {
        if (nodes_[i].id < 0) {
            validation.issue = GraphIssue::INVALID_NODE_ID;
            validation.offending_node_id = nodes_[i].id;
            return validation;
        }
        if (!isFinite(nodes_[i].x_m) || !isFinite(nodes_[i].y_m)) {
            validation.issue = GraphIssue::INVALID_COORDINATE;
            validation.offending_node_id = nodes_[i].id;
            return validation;
        }
        for (int j = i + 1; j < node_count_; ++j) {
            if (nodes_[i].id == nodes_[j].id) {
                validation.issue = GraphIssue::DUPLICATE_NODE_ID;
                validation.offending_node_id = nodes_[i].id;
                return validation;
            }
        }
    }
    for (int i = 0; i < edge_count_; ++i) {
        const GraphEdge& edge = edges_[i];
        if (indexOfNode(edge.from) < 0 || indexOfNode(edge.to) < 0) {
            validation.issue = GraphIssue::EDGE_UNKNOWN_NODE;
            validation.offending_edge_index = i;
            return validation;
        }
        if (edge.from == edge.to) {
            validation.issue = GraphIssue::SELF_LOOP_NOT_ALLOWED;
            validation.offending_edge_index = i;
            return validation;
        }
        if (edge.weight_m < 0.0f) {
            validation.issue = GraphIssue::NEGATIVE_EDGE_WEIGHT;
            validation.offending_edge_index = i;
            return validation;
        }
    }
    validation.valid = true;
    return validation;
}

bool Graph::isReachable(int start_node_id, int destination_node_id) const {
    const int start_index = indexOfNode(start_node_id);
    const int dest_index = indexOfNode(destination_node_id);
    if (start_index < 0 || dest_index < 0) {
        return false;
    }
    if (start_index == dest_index) {
        return true;
    }
    bool visited[kMaxGraphNodes] = {};
    int stack[kMaxGraphNodes];
    int top = 0;
    stack[top++] = start_index;
    visited[start_index] = true;
    while (top > 0) {
        const int current = stack[--top];
        for (int adj = adjacency_head_[current]; adj >= 0; adj = adjacency_[adj].next_adjacency) {
            const int next = adjacency_[adj].to_node_index;
            if (next == dest_index) {
                return true;
            }
            if (!visited[next]) {
                visited[next] = true;
                if (top < kMaxGraphNodes) {
                    stack[top++] = next;
                }
            }
        }
    }
    return false;
}

void Graph::bounds(float& min_x, float& min_y, float& max_x, float& max_y) const {
    min_x = 0.0f;
    min_y = 0.0f;
    max_x = 0.0f;
    max_y = 0.0f;
    if (node_count_ == 0) {
        return;
    }
    min_x = max_x = nodes_[0].x_m;
    min_y = max_y = nodes_[0].y_m;
    for (int i = 1; i < node_count_; ++i) {
        if (nodes_[i].x_m < min_x) min_x = nodes_[i].x_m;
        if (nodes_[i].x_m > max_x) max_x = nodes_[i].x_m;
        if (nodes_[i].y_m < min_y) min_y = nodes_[i].y_m;
        if (nodes_[i].y_m > max_y) max_y = nodes_[i].y_m;
    }
}

float Graph::euclideanWeight(int edge_index) const {
    if (edge_index < 0 || edge_index >= edge_count_) {
        return 0.0f;
    }
    const GraphEdge& edge = edges_[edge_index];
    const int a = indexOfNode(edge.from);
    const int b = indexOfNode(edge.to);
    if (a < 0 || b < 0) {
        return 0.0f;
    }
    const float dx = nodes_[b].x_m - nodes_[a].x_m;
    const float dy = nodes_[b].y_m - nodes_[a].y_m;
    return std::sqrt(dx * dx + dy * dy);
}

}  // namespace nav
