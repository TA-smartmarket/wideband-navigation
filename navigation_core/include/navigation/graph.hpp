// Graph model of the Smart Market: nodes with metric coordinates and weighted
// edges.  Storage is a fixed-size adjacency list so the same code runs on the
// ESP32-S3 without heap churn; `capacity` is a compile-time bound.
#pragma once

#include "navigation/geometry.hpp"
#include "navigation/types.hpp"

namespace nav {

enum class NodeType {
    INTERSECTION = 0,
    AISLE,
    DESTINATION,
    PARKING,
    ENTRY,
    EXIT,
    CHECKOUT,
    UNKNOWN,
};

const char* toString(NodeType type);
NodeType nodeTypeFromString(const char* text);

struct GraphNode {
    int id{-1};
    float x_m{0.0f};
    float y_m{0.0f};
    NodeType type{NodeType::INTERSECTION};
};

struct GraphEdge {
    int from{-1};
    int to{-1};
    float weight_m{0.0f};
    bool bidirectional{true};
};

enum class GraphIssue {
    NONE = 0,
    EMPTY_GRAPH,
    DUPLICATE_NODE_ID,
    INVALID_NODE_ID,
    INVALID_COORDINATE,
    EDGE_UNKNOWN_NODE,
    NEGATIVE_EDGE_WEIGHT,
    SELF_LOOP_NOT_ALLOWED,
    EDGE_TABLE_FULL,
    NODE_TABLE_FULL,
    DUPLICATE_EDGE,
};

const char* toString(GraphIssue issue);

struct GraphValidation {
    bool valid{false};
    GraphIssue issue{GraphIssue::NONE};
    int offending_node_id{-1};
    int offending_edge_index{-1};

    const char* message() const { return toString(issue); }
};

/// Fixed-capacity undirected/directed weighted graph.
class Graph {
public:
    Graph() = default;

    void clear();

    /// Add a node.  Returns the node index, or -1 when the table is full or the
    /// id/coordinates are invalid.
    int addNode(int id, float x_m, float y_m, NodeType type = NodeType::INTERSECTION);

    /// Add an edge.  When `weight_m < 0` the Euclidean distance between the
    /// endpoints is used (automatic weight).  Returns the edge index or -1.
    int addEdge(int from, int to, float weight_m = -1.0f, bool bidirectional = true);

    /// Remove every edge (used when reloading a map at runtime).
    void clearEdges();

    int nodeCount() const { return node_count_; }
    int edgeCount() const { return edge_count_; }
    int directedEdgeCount() const { return directed_edge_count_; }

    const GraphNode& nodeAt(int index) const { return nodes_[index]; }
    const GraphEdge& edgeAt(int index) const { return edges_[index]; }

    /// Index of the node with the given id, or -1.
    int indexOfNode(int id) const;

    /// Node pointer by id, or nullptr.
    const GraphNode* findNode(int id) const;

    /// Nearest node to `position`.  `max_distance_m < 0` disables the limit.
    NearestNodeResult findNearestNode(const Position2D& position, float max_distance_m = -1.0f) const;

    /// Adjacency iteration: returns the first adjacency entry index or -1.
    int firstAdjacencyOf(int node_index) const;
    /// Next adjacency entry index for the same node, or -1.
    int nextAdjacencyOf(int node_index, int adjacency_index) const;
    /// Target node index of an adjacency entry (-1 when out of range).
    int adjacencyTargetNodeIndex(int adjacency_index) const;
    /// Edge weight of an adjacency entry (0 when out of range).
    float adjacencyWeight(int adjacency_index) const;
    /// Index of the node in the node table (identity on the stored index).
    int nodeIdAtIndex(int node_index) const;

    /// Check structural invariants (see docs/navigation_algorithm.md).
    GraphValidation validate() const;

    /// True when every node is reachable from `start_node_id` (used to detect a
    /// disconnected destination before planning).
    bool isReachable(int start_node_id, int destination_node_id) const;

    /// Bounding box of the graph, useful for validation and visualisation.
    void bounds(float& min_x, float& min_y, float& max_x, float& max_y) const;

    /// Auto-computed Euclidean weight of an edge as currently stored.
    float euclideanWeight(int edge_index) const;

private:
    struct Adjacency {
        int to_node_index{-1};
        float weight_m{0.0f};
        int edge_index{-1};
        int next_adjacency{-1};
    };

    GraphNode nodes_[kMaxGraphNodes]{};
    int node_count_{0};

    GraphEdge edges_[kMaxGraphEdges]{};
    int edge_count_{0};
    int directed_edge_count_{0};

    Adjacency adjacency_[kMaxGraphEdges * 2]{};
    int adjacency_count_{0};
    int adjacency_head_[kMaxGraphNodes]{};
};

}  // namespace nav
