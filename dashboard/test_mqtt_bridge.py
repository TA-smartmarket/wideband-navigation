import json
import time
import unittest

from dashboard.mqtt_bridge import DEVICE_STALE_SECONDS, MqttBridge


class Message:
    def __init__(self, topic: str, payload: str):
        self.topic = topic
        self.payload = payload.encode()


class MqttBridgeTests(unittest.TestCase):
    def setUp(self):
        self.packets = []
        self.events = []
        self.bridge = MqttBridge(self.packets.append,
                                 lambda code, message: self.events.append((code, message)))
        self.bridge._broker_connected = True

    def message(self, suffix: str, payload: str) -> None:
        self.bridge._on_message(None, None,
                                Message(f"smart_trolley/TROLLEY_01/{suffix}", payload))

    def test_online_without_fresh_telemetry_does_not_arm_controls(self):
        self.message("status", "online")
        self.assertFalse(self.bridge.connected)
        self.message("telemetry", json.dumps({"type": "telemetry", "timestamp_ms": 1}))
        self.assertTrue(self.bridge.connected)

    def test_stale_telemetry_and_offline_status_disarm_controls(self):
        self.message("status", "online")
        self.message("wheel_telemetry", json.dumps({"type": "wheel_telemetry"}))
        self.bridge._last_rx_monotonic = time.monotonic() - DEVICE_STALE_SECONDS - 0.1
        self.assertFalse(self.bridge.connected)
        self.message("status", "offline")
        self.assertFalse(self.bridge.snapshot()["device_online"])

    def test_all_json_device_topics_reach_dashboard(self):
        for suffix, kind in (("telemetry", "telemetry"),
                             ("wheel_telemetry", "wheel_telemetry"),
                             ("position_verification", "position_verification"),
                             ("response", "ack")):
            self.message(suffix, json.dumps({"type": kind}))
        self.assertEqual([packet["type"] for packet in self.packets],
                         ["telemetry", "wheel_telemetry", "position_verification", "ack"])

    def test_invalid_json_is_reported_not_forwarded(self):
        self.message("telemetry", "not-json")
        self.assertFalse(self.packets)
        self.assertEqual(self.events[-1][0], "MQTT_PACKET_ERROR")


if __name__ == "__main__":
    unittest.main()
