import unittest
from unittest.mock import patch

from fastapi import HTTPException

from dashboard import main


class MissionLockTests(unittest.TestCase):
    def setUp(self):
        self.previous_active = main.station.mission_active
        self.previous_mode = main.station.mode
        self.previous_telemetry = main.station.telemetry.copy()

    def tearDown(self):
        main.station.mission_active = self.previous_active
        main.station.mode = self.previous_mode
        main.station.telemetry = self.previous_telemetry

    def test_scene_edits_are_rejected_during_mission(self):
        main.station.mission_active = True
        with self.assertRaises(HTTPException) as raised:
            main.update_scene(main.station.scene)
        self.assertEqual(raised.exception.status_code, 409)
        with self.assertRaises(HTTPException) as raised:
            main.control({"type": "odometry.set_pose", "x_m": 1, "y_m": 1,
                          "heading_rad": 0})
        self.assertEqual(raised.exception.status_code, 409)

    def test_simulation_stop_unlocks_editor(self):
        main.station.mode = "simulation"
        self.assertTrue(main.simulation_action("start")["mission_active"])
        self.assertFalse(main.simulation_action("stop")["mission_active"])
        main.require_idle_mission()

    def test_arrival_unlocks_and_wheel_pose_survives_navigation_packet(self):
        with patch.object(main.station, "_append_telemetry"):
            main.station.receive_packet({"type": "wheel_telemetry",
                                         "pose_reset_count": 3,
                                         "odometry_pose": {"x_m": 1, "y_m": 2,
                                                           "valid": True}})
            main.station.mission_active = True
            main.station.receive_packet({"type": "telemetry",
                                         "navigation": {"state": "ARRIVED"}})
        self.assertFalse(main.station.mission_active)
        self.assertEqual(main.station.telemetry["pose_reset_count"], 3)


if __name__ == "__main__":
    unittest.main()
