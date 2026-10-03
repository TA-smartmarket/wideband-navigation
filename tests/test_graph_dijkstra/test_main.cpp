// Unit tests: graph construction/validation and Dijkstra shortest path.
//
// The graph in `buildSpecificationGraph()` is the worked example from the
// project specification:
//   A --1-- B --1-- C
//   |               |
//   5               1
//   |               |
//   D ------1-------E
#include <unity.h>

#include <cmath>

#include "navigation/dijkstra.hpp"
#include "navigation/geometry.hpp"
#include "navigation/graph.hpp"

using namespace nav;

namespace {

Graph buildSpecificationGraph() {
    Graph graph;
    graph.addNode(1, 0.0f, 0.0f, NodeType::INTERSECTION);  // A
    graph.addNode(2, 1.0f, 0.0f, NodeType::INTERSECTION);  // B
    graph.addNode(3, 2.0f, 0.0f, NodeType::INTERSECTION);  // C
    graph.addNode(4, 0.0f, 5.0f, NodeType::INTERSECTION);  // D
    graph.addNode(5, 2.0f, 1.0f, NodeType::INTERSECTION);  // E
    graph.addEdge(1, 2, 1.0f);
    graph.addEdge(2, 3, 1.0f);
    graph.addEdge(1, 4, 5.0f);
    graph.addEdge(3, 5, 1.0f);
    graph.addEdge(4, 5, 1.0f);
    return graph;
}

}  // namespace

void setUp() {}
void tearDown() {}

// ---------------------------------------------------------------------------
// Graph
// ---------------------------------------------------------------------------

static void test_graph_node_and_edge_creation() {
    Graph graph;
    TEST_ASSERT_EQUAL_INT(0, graph.addNode(1, 1.0f, 2.0f, NodeType::AISLE));
    TEST_ASSERT_EQUAL_INT(1, graph.addNode(2, 3.0f, 2.0f, NodeType::DESTINATION));
    TEST_ASSERT_EQUAL_INT(-1, graph.addNode(1, 5.0f, 5.0f));  // duplicate id
    TEST_ASSERT_EQUAL_INT(-1, graph.addNode(-3, 5.0f, 5.0f));  // invalid id
    TEST_ASSERT_EQUAL_INT(2, graph.nodeCount());
    TEST_ASSERT_EQUAL_FLOAT(1.0f, graph.findNode(1)->x_m);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(NodeType::DESTINATION),
                          static_cast<int>(graph.findNode(2)->type));

    const int edge = graph.addEdge(1, 2);
    TEST_ASSERT_TRUE(edge >= 0);
    TEST_ASSERT_EQUAL_INT(1, graph.edgeCount());
    TEST_ASSERT_EQUAL_FLOAT(2.0f, graph.edgeAt(edge).weight_m);  // auto Euclidean
    TEST_ASSERT_EQUAL_INT(-1, graph.addEdge(1, 1));              // self loop rejected
    TEST_ASSERT_EQUAL_INT(-1, graph.addEdge(1, 99));             // unknown node rejected
}

static void test_graph_validation_detects_problems() {
    Graph empty;
    TEST_ASSERT_FALSE(empty.validate().valid);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(GraphIssue::EMPTY_GRAPH),
                          static_cast<int>(empty.validate().issue));

    const Graph good = buildSpecificationGraph();
    TEST_ASSERT_TRUE(good.validate().valid);
    TEST_ASSERT_EQUAL_INT(5, good.nodeCount());
    TEST_ASSERT_EQUAL_INT(5, good.edgeCount());
}

static void test_graph_bounds_and_clear() {
    const Graph graph = buildSpecificationGraph();
    float min_x = 0.0f, min_y = 0.0f, max_x = 0.0f, max_y = 0.0f;
    graph.bounds(min_x, min_y, max_x, max_y);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, min_x);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, min_y);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, max_x);
    TEST_ASSERT_EQUAL_FLOAT(5.0f, max_y);
}

// ---------------------------------------------------------------------------
// Dijkstra
// ---------------------------------------------------------------------------

static void test_shortest_path_is_mathematically_correct() {
    const Graph graph = buildSpecificationGraph();
    // A -> B -> C costs 1 + 1 = 2 m; the detour A -> D -> E -> C costs 7 m.
    const PathResult result = dijkstra(graph, 1, 3);
    TEST_ASSERT_TRUE(result.success);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, result.total_distance_m);
    TEST_ASSERT_EQUAL_INT(3, result.node_count);
    TEST_ASSERT_EQUAL_INT(1, result.node_ids[0]);
    TEST_ASSERT_EQUAL_INT(2, result.node_ids[1]);
    TEST_ASSERT_EQUAL_INT(3, result.node_ids[2]);
}

static void test_shortest_path_prefers_cheaper_detour() {
    const Graph graph = buildSpecificationGraph();
    // 1 -> 4 -> 5 costs 5 + 1 = 6 m, 1 -> 2 -> 3 -> 5 costs 1 + 1 + 1 = 3 m.
    const PathResult result = dijkstra(graph, 1, 5);
    TEST_ASSERT_TRUE(result.success);
    TEST_ASSERT_EQUAL_FLOAT(3.0f, result.total_distance_m);
    TEST_ASSERT_EQUAL_INT(4, result.node_count);
    TEST_ASSERT_EQUAL_INT(1, result.node_ids[0]);
    TEST_ASSERT_EQUAL_INT(2, result.node_ids[1]);
    TEST_ASSERT_EQUAL_INT(3, result.node_ids[2]);
    TEST_ASSERT_EQUAL_INT(5, result.node_ids[3]);
}

static void test_start_equals_destination() {
    const Graph graph = buildSpecificationGraph();
    const PathResult result = dijkstra(graph, 3, 3);
    TEST_ASSERT_TRUE(result.success);
    TEST_ASSERT_EQUAL_INT(1, result.node_count);
    TEST_ASSERT_EQUAL_INT(3, result.node_ids[0]);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, result.total_distance_m);
}

static void test_unreachable_destination_reports_no_route() {
    Graph graph = buildSpecificationGraph();
    graph.addNode(9, 50.0f, 50.0f);  // isolated node
    const PathResult result = dijkstra(graph, 1, 9);
    TEST_ASSERT_FALSE(result.success);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(PathError::NO_ROUTE), static_cast<int>(result.error));
}

static void test_invalid_start_and_destination() {
    const Graph graph = buildSpecificationGraph();
    TEST_ASSERT_EQUAL_INT(static_cast<int>(PathError::INVALID_START),
                          static_cast<int>(dijkstra(graph, 42, 3).error));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(PathError::INVALID_DESTINATION),
                          static_cast<int>(dijkstra(graph, 1, 42).error));
}

static void test_empty_graph() {
    Graph graph;
    TEST_ASSERT_EQUAL_INT(static_cast<int>(PathError::EMPTY_GRAPH),
                          static_cast<int>(dijkstra(graph, 1, 2).error));
}

static void test_direct_edge_beats_longer_detour() {
    // Second, independent graph: a diamond where the direct edge is longer than
    // the two-hop detour, exercising multiple candidate routes.
    Graph graph;
    graph.addNode(1, 0.0f, 0.0f);
    graph.addNode(2, 1.0f, 0.0f);
    graph.addNode(3, 2.0f, 0.0f);
    graph.addNode(4, 1.0f, 1.0f);
    graph.addEdge(1, 3, 5.0f);  // expensive direct link
    graph.addEdge(1, 4, 1.0f);
    graph.addEdge(4, 3, 1.0f);
    graph.addEdge(1, 2, 1.0f);
    graph.addEdge(2, 3, 1.0f);
    const PathResult result = dijkstra(graph, 1, 3);
    TEST_ASSERT_TRUE(result.success);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, result.total_distance_m);
    TEST_ASSERT_EQUAL_INT(3, result.node_count);
}

static void test_directed_edges_are_respected() {
    Graph graph;
    graph.addNode(1, 0.0f, 0.0f);
    graph.addNode(2, 1.0f, 0.0f);
    graph.addEdge(1, 2, 1.0f, /*bidirectional=*/false);
    TEST_ASSERT_TRUE(dijkstra(graph, 1, 2).success);
    TEST_ASSERT_FALSE(dijkstra(graph, 2, 1).success);
}

static void test_path_to_waypoints_carries_coordinates() {
    const Graph graph = buildSpecificationGraph();
    const PathResult result = dijkstra(graph, 1, 3);
    Waypoint waypoints[kMaxRouteNodes];
    const int count = pathToWaypoints(graph, result, waypoints, kMaxRouteNodes);
    TEST_ASSERT_EQUAL_INT(3, count);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, waypoints[0].x_m);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, waypoints[1].x_m);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, waypoints[2].x_m);
}

static void test_nearest_node_and_snap_limit() {
    const Graph graph = buildSpecificationGraph();
    const NearestNodeResult near = graph.findNearestNode(Position2D{0.2f, 0.1f}, 1.5f);
    TEST_ASSERT_TRUE(near.found);
    TEST_ASSERT_EQUAL_INT(1, near.node_id);

    const NearestNodeResult far = graph.findNearestNode(Position2D{20.0f, 20.0f}, 1.5f);
    TEST_ASSERT_FALSE(far.found);          // too far to snap safely
    // Closest candidate from (20,20): node 4 at (0,5) => sqrt(400+225) = 25.0 m,
    // node 5 at (2,1) => sqrt(324+361) = 26.2 m.
    TEST_ASSERT_EQUAL_INT(4, far.node_id);

    const NearestNodeResult unlimited = graph.findNearestNode(Position2D{20.0f, 20.0f}, -1.0f);
    TEST_ASSERT_TRUE(unlimited.found);
}

static void test_reachability_query() {
    Graph graph = buildSpecificationGraph();
    TEST_ASSERT_TRUE(graph.isReachable(1, 5));
    graph.addNode(9, 50.0f, 50.0f);
    TEST_ASSERT_FALSE(graph.isReachable(1, 9));
}

static void test_polyline_length_and_cross_track() {
    Waypoint waypoints[3];
    waypoints[0] = Waypoint{1, 0.0f, 0.0f};
    waypoints[1] = Waypoint{2, 2.0f, 0.0f};
    waypoints[2] = Waypoint{3, 2.0f, 2.0f};
    TEST_ASSERT_EQUAL_FLOAT(4.0f, polylineLength(waypoints, 3));
    TEST_ASSERT_EQUAL_FLOAT(0.5f, distanceToSegment(Position2D{1.0f, 0.5f}, waypoints, 3, 0));
    TEST_ASSERT_EQUAL_FLOAT(0.5f, crossTrackError(Position2D{1.0f, 0.5f}, waypoints, 3));
    // A point beyond the segment end uses the endpoint distance.
    TEST_ASSERT_EQUAL_FLOAT(1.0f, distanceToSegment(Position2D{3.0f, 0.0f}, waypoints, 3, 0));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_graph_node_and_edge_creation);
    RUN_TEST(test_graph_validation_detects_problems);
    RUN_TEST(test_graph_bounds_and_clear);
    RUN_TEST(test_shortest_path_is_mathematically_correct);
    RUN_TEST(test_shortest_path_prefers_cheaper_detour);
    RUN_TEST(test_start_equals_destination);
    RUN_TEST(test_unreachable_destination_reports_no_route);
    RUN_TEST(test_invalid_start_and_destination);
    RUN_TEST(test_empty_graph);
    RUN_TEST(test_direct_edge_beats_longer_detour);
    RUN_TEST(test_directed_edges_are_respected);
    RUN_TEST(test_path_to_waypoints_carries_coordinates);
    RUN_TEST(test_nearest_node_and_snap_limit);
    RUN_TEST(test_reachability_query);
    RUN_TEST(test_polyline_length_and_cross_track);
    return UNITY_END();
}
