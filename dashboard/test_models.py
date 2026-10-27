import math
import unittest

from dashboard.models import (ValidationError, build_visibility_edges, fit_nodes_to_room, plan_route,
                              segment_hits_obstacle, segment_hits_rect, validate_graph,
                              validate_motion, validate_scene)


class DashboardModelTests(unittest.TestCase):
    def test_segment_rectangle_collision(self):
        self.assertTrue(segment_hits_rect((0, 0), (2, 0), (0.8, -0.2, 1.2, 0.2)))
        self.assertFalse(segment_hits_rect((0, 0), (0, 0.5), (0.8, -0.2, 1.2, 0.2)))

    def test_route_avoids_blocked_edge(self):
        graph = {"nodes": [{"id": 1, "x_m": 0, "y_m": 0}, {"id": 2, "x_m": 1, "y_m": 0}, {"id": 3, "x_m": 0, "y_m": 1}],
                 "edges": [{"from": 1, "to": 2}, {"from": 1, "to": 3}, {"from": 3, "to": 2}]}
        scene = {"scene": {"obstacles": [{"id": "rack", "x": .5, "y": 0, "sx": .2, "sy": .2}]}}
        self.assertEqual(plan_route(graph, scene, 1, 2).nodes, [1, 3, 2])

    def test_motion_limit_is_enforced(self):
        with self.assertRaises(ValidationError):
            validate_motion(1, 0, .45, 2.2)

    def test_product_mapping_must_reference_graph_node(self):
        map_document = {"width_m": 12, "height_m": 8}
        graph = {"nodes": [{"id": 1}]}
        scene = {"scene": {"room": {}, "obstacles": [{"id": "rack", "x": 2, "y": 2,
                 "sx": 1, "sy": 1, "products": [{"id": "p1", "name": "Milk", "node_id": 99}]}]}}
        with self.assertRaises(ValidationError):
            validate_scene(scene, map_document, graph)

    def test_rotated_obstacle_collision_uses_oriented_rectangle(self):
        obstacle = {"x": 1, "y": 1, "sx": 1.4, "sy": .2, "rot": math.pi / 4}
        self.assertTrue(segment_hits_obstacle((.5, .5), (1.5, 1.5), obstacle))
        self.assertFalse(segment_hits_obstacle((.4, 1.6), (.6, 1.8), obstacle))

    def test_rotated_obstacle_must_fit_room(self):
        scene = {"scene": {"room": {"height": 2.7}, "obstacles": [
            {"id": "rack", "x": .5, "y": .5, "sx": 1, "sy": 1, "sz": 1, "z": .5, "rot": math.pi / 4}
        ]}}
        with self.assertRaises(ValidationError):
            validate_scene(scene, {"width_m": 4, "height_m": 4})

    def test_rack_always_rests_on_floor(self):
        scene = {"scene": {"room": {"height": 2.7}, "obstacles": [
            {"id": "rack", "x": 2, "y": 2, "sx": 1, "sy": 1, "sz": 1.6, "z": 99}
        ]}}
        validated = validate_scene(scene, {"width_m": 4, "height_m": 4})
        self.assertEqual(validated["scene"]["obstacles"][0]["z"], .8)

    def test_graph_node_names_are_unique(self):
        graph = {"nodes": [
            {"id": 1, "name": "Kasir", "x_m": 1, "y_m": 1},
            {"id": 2, "name": "kasir", "x_m": 2, "y_m": 1},
        ], "edges": []}
        with self.assertRaises(ValidationError):
            validate_graph(graph, {"width_m": 4, "height_m": 4})

    def test_graph_node_cannot_be_inside_rotated_obstacle(self):
        graph = {"nodes": [{"id": 1, "name": "A", "x_m": 1, "y_m": 1}], "edges": []}
        scene = {"scene": {"obstacles": [{"x": 1, "y": 1, "sx": 1, "sy": .2, "rot": math.pi / 4}]}}
        with self.assertRaises(ValidationError):
            validate_graph(graph, {"width_m": 4, "height_m": 4}, scene)

    def test_visibility_graph_omits_edges_through_obstacles(self):
        nodes = [
            {"id": 1, "x_m": 0, "y_m": 0},
            {"id": 2, "x_m": 2, "y_m": 0},
            {"id": 3, "x_m": 0, "y_m": 2},
        ]
        scene = {"scene": {"obstacles": [{"x": 1, "y": 0, "sx": .2, "sy": .2}]}}
        edges = build_visibility_edges(nodes, scene)
        pairs = {(edge["from"], edge["to"]) for edge in edges}
        self.assertNotIn((1, 2), pairs)
        self.assertIn((1, 3), pairs)

    def test_visibility_grid_prefers_cardinal_edges(self):
        nodes = [
            {"id": 1, "x_m": 0, "y_m": 0}, {"id": 2, "x_m": 2, "y_m": 0},
            {"id": 3, "x_m": 0, "y_m": 2}, {"id": 4, "x_m": 2, "y_m": 2},
        ]
        edges = build_visibility_edges(nodes, {"scene": {"obstacles": []}})
        pairs = {(edge["from"], edge["to"]) for edge in edges}
        self.assertEqual(pairs, {(1, 2), (1, 3), (2, 4), (3, 4)})

    def test_moving_node_preserves_logical_grid_neighbours(self):
        nodes = [
            {"id": 1, "x_m": 0, "y_m": 0}, {"id": 2, "x_m": 3, "y_m": 1},
            {"id": 3, "x_m": 0, "y_m": 2}, {"id": 4, "x_m": 2, "y_m": 2},
        ]
        edges = build_visibility_edges(nodes, {"scene": {"obstacles": []}})
        pairs = {(edge["from"], edge["to"]) for edge in edges}
        self.assertEqual(pairs, {(1, 2), (1, 3), (2, 4), (3, 4)})

    def test_node_grid_follows_room_and_moves_whole_row_around_rack(self):
        nodes = [{"id": identifier, "name": f"N{identifier}"} for identifier in range(1, 13)]
        scene = {"scene": {"obstacles": [
            {"x": 3.3, "y": 4.8, "sx": 1.2, "sy": .6, "rot": 0}
        ]}}
        fitted = fit_nodes_to_room(nodes, {"width_m": 9, "height_m": 6}, scene)
        self.assertEqual(len({round(node["x_m"], 6) for node in fitted}), 4)
        self.assertEqual(len({round(node["y_m"], 6) for node in fitted}), 3)
        self.assertTrue(all(0 <= node["x_m"] <= 9 and 0 <= node["y_m"] <= 6 for node in fitted))
        self.assertEqual(len({round(node["y_m"], 6) for node in fitted[8:12]}), 1)


if __name__ == "__main__":
    unittest.main()
