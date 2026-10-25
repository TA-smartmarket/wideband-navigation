"""Threaded USB-serial bridge for newline-delimited dashboard protocol v1."""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import json
import queue
import threading
import time
from typing import Any, Callable

try:  # Keep simulation mode usable without pyserial installed.
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - exercised on a laptop without pyserial
    serial = None
    list_ports = None


PROTOCOL_VERSION = 1


def available_ports() -> list[dict[str, str]]:
    if list_ports is None:
        return []
    return [{"device": port.device, "description": port.description} for port in list_ports.comports()]


@dataclass
class SerialBridge:
    on_message: Callable[[dict[str, Any]], None]
    on_event: Callable[[str, str], None]
    _port: Any = None
    _thread: threading.Thread | None = None
    _stop: threading.Event = field(default_factory=threading.Event)
    _outgoing: queue.Queue[bytes] = field(default_factory=queue.Queue)
    connected_port: str | None = None
    baud_rate: int = 115200
    last_rx_monotonic: float | None = None
    rx_lines: int = 0
    tx_lines: int = 0
    recent_diagnostics: deque[str] = field(default_factory=lambda: deque(maxlen=40))

    @property
    def connected(self) -> bool:
        return self._port is not None and bool(getattr(self._port, "is_open", False))

    def connect(self, port: str, baud_rate: int) -> None:
        if serial is None:
            raise RuntimeError("pyserial is not installed; install dashboard/requirements.txt")
        self.disconnect()
        self._port = serial.Serial(port, baudrate=baud_rate, timeout=0.10, write_timeout=0.5)
        self.connected_port, self.baud_rate = port, baud_rate
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="esp32-serial", daemon=True)
        self._thread.start()
        self.on_event("SERIAL_CONNECTED", f"{port} @ {baud_rate}")
        self.send({"type": "hello", "protocol_version": PROTOCOL_VERSION})

    def disconnect(self) -> None:
        self._stop.set()
        thread, self._thread = self._thread, None
        if thread and thread is not threading.current_thread():
            thread.join(timeout=0.5)
        if self._port:
            try:
                self._port.close()
            except Exception:
                pass
        was_connected = self.connected_port is not None
        self._port, self.connected_port = None, None
        if was_connected:
            self.on_event("SERIAL_DISCONNECTED", "serial link closed")

    def send(self, payload: dict[str, Any]) -> None:
        if not self.connected:
            raise RuntimeError("ESP32 is not connected")
        message = dict(payload)
        message.setdefault("protocol_version", PROTOCOL_VERSION)
        self._outgoing.put((json.dumps(message, separators=(",", ":"), allow_nan=False) + "\n").encode())

    def snapshot(self) -> dict[str, Any]:
        return {
            "connected": self.connected,
            "port": self.connected_port,
            "baud_rate": self.baud_rate,
            "last_rx_age_ms": None if self.last_rx_monotonic is None else round((time.monotonic() - self.last_rx_monotonic) * 1000),
            "rx_lines": self.rx_lines,
            "tx_lines": self.tx_lines,
        }

    def _run(self) -> None:
        while not self._stop.is_set() and self._port:
            try:
                try:
                    raw = self._outgoing.get_nowait()
                    self._port.write(raw)
                    self.tx_lines += 1
                except queue.Empty:
                    pass
                raw_line = self._port.readline()
                if not raw_line:
                    continue
                line = raw_line.decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                self.rx_lines += 1
                self.last_rx_monotonic = time.monotonic()
                if not line.startswith("{"):
                    self.recent_diagnostics.append(line)
                    self.on_event("ESP32_LOG", line)
                    continue
                message = json.loads(line)
                if not isinstance(message, dict):
                    raise ValueError("JSON packet is not an object")
                self.on_message(message)
            except (OSError, ValueError, json.JSONDecodeError) as exc:
                self.recent_diagnostics.append(str(exc))
                self.on_event("SERIAL_ERROR", str(exc))
                if isinstance(exc, OSError):
                    self.disconnect()
                    return
