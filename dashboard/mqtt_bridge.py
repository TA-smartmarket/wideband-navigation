"""Threaded MQTT transport for the Smart Trolley dashboard protocol.

The dashboard connects to the broker on the same laptop.  ``connected`` only
becomes true after the ESP32 publishes ``online`` and fresh telemetry arrives;
being connected to the broker alone is not enough to arm motor controls.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import json
import threading
import time
from typing import Any, Callable

try:
    import paho.mqtt.client as mqtt
except ImportError:  # pragma: no cover - reported clearly by connect()
    mqtt = None


PROTOCOL_VERSION = 1
DEVICE_STALE_SECONDS = 2.0


def _reason_value(reason_code: Any) -> int:
    """Normalise Paho v2 ReasonCode without relying on its display string."""
    value = getattr(reason_code, "value", reason_code)
    try:
        return int(value)
    except (TypeError, ValueError):
        return 1


@dataclass
class MqttBridge:
    on_message: Callable[[dict[str, Any]], None]
    on_event: Callable[[str, str], None]
    trolley_id: str = "TROLLEY_01"
    base_topic: str = "smart_trolley"
    _client: Any = None
    _lock: threading.RLock = field(default_factory=threading.RLock)
    _broker_connected: bool = False
    _device_online: bool = False
    _last_rx_monotonic: float | None = None
    broker_host: str = "127.0.0.1"
    broker_port: int = 1883
    rx_lines: int = 0
    tx_lines: int = 0

    @property
    def topic_root(self) -> str:
        return f"{self.base_topic}/{self.trolley_id}"

    @property
    def connected(self) -> bool:
        with self._lock:
            fresh = (self._last_rx_monotonic is not None and
                     time.monotonic() - self._last_rx_monotonic <= DEVICE_STALE_SECONDS)
            return self._broker_connected and self._device_online and fresh

    def connect(self, host: str = "127.0.0.1", port: int = 1883) -> None:
        if mqtt is None:
            raise RuntimeError("paho-mqtt is not installed; install dashboard/requirements.txt")
        if not host or not 1 <= int(port) <= 65535:
            raise ValueError("invalid MQTT broker address")
        self.disconnect(send_stop=False)
        self.broker_host, self.broker_port = host, int(port)
        client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=f"smart-trolley-dashboard-{int(time.time())}",
            protocol=mqtt.MQTTv311,
            reconnect_on_failure=True,
        )
        client.on_connect = self._on_connect
        client.on_disconnect = self._on_disconnect
        client.on_message = self._on_message
        with self._lock:
            self._client = client
            self._broker_connected = False
            self._device_online = False
            self._last_rx_monotonic = None
        try:
            client.connect(host, int(port), keepalive=15)
            client.loop_start()
        except Exception:
            with self._lock:
                self._client = None
            try:
                client.disconnect()
            except Exception:
                pass
            raise
        self.on_event("MQTT_CONNECTING", f"broker {host}:{port}")

    def disconnect(self, send_stop: bool = True) -> None:
        client = self._client
        if client is None:
            return
        if send_stop and self._broker_connected:
            self._publish({"type": "motor.manual", "linear_mps": 0.0,
                           "angular_radps": 0.0, "heartbeat": False}, require_device=False)
            time.sleep(0.05)
        with self._lock:
            self._client = None
            self._broker_connected = False
            self._device_online = False
            self._last_rx_monotonic = None
        try:
            client.disconnect()
            client.loop_stop()
        except Exception:
            pass

    def send(self, payload: dict[str, Any]) -> None:
        if not self.connected:
            raise RuntimeError("ESP32 MQTT is offline or telemetry is stale")
        self._publish(payload, require_device=True)

    def _publish(self, payload: dict[str, Any], require_device: bool) -> None:
        client = self._client
        if client is None or not self._broker_connected or (require_device and not self.connected):
            raise RuntimeError("MQTT transport is not connected")
        message = dict(payload)
        message.setdefault("protocol_version", PROTOCOL_VERSION)
        encoded = json.dumps(message, separators=(",", ":"), allow_nan=False)
        result = client.publish(f"{self.topic_root}/command", encoded, qos=0, retain=False)
        if result.rc != mqtt.MQTT_ERR_SUCCESS:
            raise RuntimeError(f"MQTT publish failed ({result.rc})")
        self.tx_lines += 1

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            age_ms = (None if self._last_rx_monotonic is None else
                      round((time.monotonic() - self._last_rx_monotonic) * 1000))
            return {
                "connected": self.connected,
                "transport": "mqtt",
                "endpoint": f"MQTT {self.broker_host}:{self.broker_port}",
                "port": f"MQTT {self.broker_host}:{self.broker_port}",
                "baud_rate": None,
                "broker_connected": self._broker_connected,
                "device_online": self._device_online,
                "last_rx_age_ms": age_ms,
                "rx_lines": self.rx_lines,
                "tx_lines": self.tx_lines,
            }

    def _on_connect(self, client: Any, _userdata: Any, _flags: Any,
                    reason_code: Any, _properties: Any) -> None:
        if _reason_value(reason_code) != 0:
            self.on_event("MQTT_ERROR", f"broker rejected connection: {reason_code}")
            return
        with self._lock:
            self._broker_connected = True
        client.subscribe(f"{self.topic_root}/#", qos=0)
        self.on_event("MQTT_BROKER_CONNECTED", f"{self.broker_host}:{self.broker_port}")

    def _on_disconnect(self, _client: Any, _userdata: Any, _flags: Any,
                       reason_code: Any, _properties: Any) -> None:
        with self._lock:
            self._broker_connected = False
            self._device_online = False
        if _reason_value(reason_code) != 0:
            self.on_event("MQTT_DISCONNECTED", f"broker connection lost: {reason_code}")

    def _on_message(self, _client: Any, _userdata: Any, message: Any) -> None:
        suffix = str(message.topic).removeprefix(self.topic_root + "/")
        text = bytes(message.payload).decode("utf-8", errors="replace").strip()
        self.rx_lines += 1
        if suffix == "status":
            with self._lock:
                self._device_online = text == "online"
                if not self._device_online:
                    self._last_rx_monotonic = None
            self.on_event("ESP32_MQTT_STATUS", text or "unknown")
            return
        if suffix == "event":
            self.on_event("ESP32_EVENT", text)
            return
        if suffix not in {"telemetry", "wheel_telemetry", "position_verification", "response"}:
            return
        try:
            packet = json.loads(text)
            if not isinstance(packet, dict):
                raise ValueError("MQTT packet is not a JSON object")
        except (ValueError, json.JSONDecodeError) as exc:
            self.on_event("MQTT_PACKET_ERROR", str(exc))
            return
        with self._lock:
            self._last_rx_monotonic = time.monotonic()
        self.on_message(packet)
