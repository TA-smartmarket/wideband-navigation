import unittest

from dashboard.models import ValidationError, plan_route, segment_hits_rect, validate_motion


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


if __name__ == "__main__":
    unittest.main()
