"""Remote position sources: feed the navigation core from the positioning server.

The default simulator drives navigation from its own UWB sensor model
(`UwbSimulator`).  These sources replace that model with the real positioning
subsystem (wideband-positioning), reached over HTTP or MQTT, so an integration
test can run on the host without any ESP32 hardware:

  * :class:`HttpPositionSource` polls ``GET /api/v1/navigation/position`` at the
    configured rate (pull transport).
  * :class:`MqttPositionSource` subscribes to ``<base>/navigation/position`` and
    surfaces the latest pushed sample (push transport).

Both return the same ``UwbSample`` shape the engine already consumes, so the
navigation core, Dijkstra, follower and PID are exercised identically.
"""

from __future__ import annotations

import json
import math
import threading
import time
import urllib.request
from dataclasses import dataclass
from typing import Callable, Optional

try:
    import paho.mqtt.client as mqtt  # type: ignore
    _HAS_MQTT = True
except Exception:  # pragma: no cover - optional dependency
    _HAS_MQTT = False

from .uwb_simulator import UwbSample


@dataclass
class RemotePositionConfig:
    """Connection settings for a remote position source."""

    server_url: str = "http://127.0.0.1:8080"
    mqtt_host: str = "127.0.0.1"
    mqtt_port: int = 1883
    mqtt_topic: str = "uwb/home/navigation/position"
    poll_interval_s: float = 0.05


def fetch_map_from_server(server_url: str, timeout_s: float = 3.0) -> dict:
    """Fetch ``GET /api/v1/navigation/map`` and return its JSON dict.

    Raises :class:`RuntimeError` on transport/HTTP errors so a bad URL is a
    clear failure rather than a silently defaulted map.
    """
    url = server_url.rstrip("/") + "/api/v1/navigation/map"
    with urllib.request.urlopen(url, timeout=timeout_s) as response:
        payload = response.read().decode("utf-8")
    document = json.loads(payload)
    if "width_m" not in document or "height_m" not in document:
        raise RuntimeError(f"map response missing dimensions: {url}")
    return document


def fetch_scene_from_server(server_url: str, timeout_s: float = 3.0) -> dict:
    """Fetch ``GET /api/v1/navigation/scene`` and return its JSON dict.

    The scene document is the static obstacle/room contract (rot already in
    radians at the boundary), consumed once per navigation session.  Raises
    :class:`RuntimeError` on transport/HTTP errors.
    """
    url = server_url.rstrip("/") + "/api/v1/navigation/scene"
    with urllib.request.urlopen(url, timeout=timeout_s) as response:
        payload = response.read().decode("utf-8")
    document = json.loads(payload)
    if "scene" not in document:
        raise RuntimeError(f"scene response missing scene block: {url}")
    return document


class HttpPositionSource:
    """Poll the positioning server's navigation position endpoint (pull)."""

    def __init__(self, server_url: str, poll_interval_s: float = 0.05):
        self._url = server_url.rstrip("/") + "/api/v1/navigation/position"
        self._interval = poll_interval_s
        self._last_poll = 0.0

    def poll(self, now_ms: int) -> Optional[UwbSample]:
        if time.monotonic() - self._last_poll < self._interval:
            return None
        self._last_poll = time.monotonic()
        try:
            with urllib.request.urlopen(self._url, timeout=1.0) as response:
                payload = response.read().decode("utf-8")
        except Exception:
            # A transient drop-out yields no sample; the core treats that as a
            # stale position and enters POSITION_LOST, which is the safe path.
            return None
        return _sample_from_json(payload)


class MqttPositionSource:
    """Consume the positioning server's pushed position (push, lowest latency)."""

    def __init__(self, host: str, port: int, topic: str):
        if not _HAS_MQTT:
            raise RuntimeError("paho-mqtt is required for --position-source mqtt")
        self._topic = topic
        self._lock = threading.Lock()
        self._latest: Optional[UwbSample] = None
        self._client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self._client.on_message = self._on_message
        self._client.connect(host, port, keepalive=30)
        self._client.subscribe(topic, qos=0)
        self._client.loop_start()

    def _on_message(self, client, userdata, message) -> None:
        try:
            payload = message.payload.decode("utf-8")
        except Exception:
            return
        sample = _sample_from_json(payload)
        if sample is not None:
            with self._lock:
                self._latest = sample

    def poll(self, now_ms: int) -> Optional[UwbSample]:
        with self._lock:
            latest = self._latest
            self._latest = None
        return latest

    def close(self) -> None:
        try:
            self._client.loop_stop()
            self._client.disconnect()
        except Exception:
            pass


def _sample_from_json(payload: str) -> Optional[UwbSample]:
    """Parse one navigation position-contract document into a UwbSample."""
    try:
        document = json.loads(payload)
    except Exception:
        return None
    position = document.get("position", {})
    try:
        x_m = float(position.get("x_m"))
        y_m = float(position.get("y_m"))
        quality = float(document.get("quality", 0.0))
        valid = bool(document.get("valid", False))
        timestamp_ms = int(document.get("timestamp_ms", 0))
    except (TypeError, ValueError):
        return None
    if not (math.isfinite(x_m) and math.isfinite(y_m) and math.isfinite(quality)):
        return None
    return UwbSample(
        x_m=x_m,
        y_m=y_m,
        quality=quality,
        valid=valid,
        timestamp_ms=timestamp_ms,
        fresh=True,
        fault="",
    )


__all__ = [
    "RemotePositionConfig",
    "HttpPositionSource",
    "MqttPositionSource",
    "fetch_map_from_server",
    "fetch_scene_from_server",
]
