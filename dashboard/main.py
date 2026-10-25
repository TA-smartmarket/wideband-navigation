"""FastAPI entry point for the local Smart Trolley control station.

Run from the repository root: ``python dashboard/main.py``.
"""
from __future__ import annotations

import asyncio
from collections import deque
from contextlib import asynccontextmanager
from datetime import datetime
import json
import logging
from pathlib import Path
import threading
import time
from typing import Any

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
import uvicorn

try:  # Supports both `python dashboard/main.py` and `python -m dashboard.main`.
    from .models import ValidationError, plan_route, validate_map, validate_motion, validate_scene
    from .serial_bridge import PROTOCOL_VERSION, SerialBridge, available_ports
except ImportError:  # pragma: no cover - direct-script invocation
    from models import ValidationError, plan_route, validate_map, validate_motion, validate_scene
    from serial_bridge import PROTOCOL_VERSION, SerialBridge, available_ports

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config"
RUNTIME_SCENE = CONFIG / "dashboard_scene.json"
LOGS = ROOT / "logs"
LOGS.mkdir(exist_ok=True)
logging.basicConfig(filename=LOGS / "dashboard.log", level=logging.INFO,
                    format="%(asctime)s %(levelname)s %(message)s")


def read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def atomic_json_write(path: Path, document: dict[str, Any]) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(document, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


class Station:
    def __init__(self) -> None:
        self.lock = threading.RLock()
        self.mode = "simulation"
        self.map = validate_map(read_json(CONFIG / "map.json"))
        self.graph = read_json(CONFIG / "graph.json")
        self.scene = read_json(RUNTIME_SCENE if RUNTIME_SCENE.exists() else CONFIG / "scene.example.json")
        validate_scene(self.scene, self.map)
        self.telemetry: dict[str, Any] = {"type": "telemetry", "mode": "IDLE", "source": "simulation", "timestamp_ms": 0}
        self.events: deque[dict[str, Any]] = deque(maxlen=80)
        self._log_run = None
        self.bridge = SerialBridge(self.receive_packet, self.event)
        self.event("DASHBOARD_READY", "simulation mode ready; existing Pygame simulator is unchanged")

    def event(self, code: str, message: str) -> None:
        entry = {"timestamp": datetime.now().isoformat(timespec="seconds"), "code": code, "message": message}
        with self.lock:
            self.events.appendleft(entry)
        logging.info("[%s] %s", code, message)

    def receive_packet(self, packet: dict[str, Any]) -> None:
        packet_type = packet.get("type")
        if packet_type == "telemetry":
            with self.lock:
                self.telemetry = packet
            self._append_telemetry(packet)
        elif packet_type in {"event", "ack", "error", "config"}:
            self.event(str(packet_type).upper(), str(packet.get("message", packet_type)))
        else:
            self.event("SERIAL_PACKET", f"unrecognised packet type: {packet_type!r}")

    def _append_telemetry(self, packet: dict[str, Any]) -> None:
        if self._log_run is None:
            self._log_run = LOGS / f"hardware_{datetime.now():%Y%m%d_%H%M%S}.jsonl"
        with self._log_run.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(packet, allow_nan=False) + "\n")

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            return {"mode": self.mode, "map": self.map, "graph": self.graph, "scene": self.scene,
                    "telemetry": self.telemetry, "events": list(self.events), "connection": self.bridge.snapshot(),
                    "protocol_version": PROTOCOL_VERSION}


station = Station()


@asynccontextmanager
async def lifespan(_: FastAPI):
    yield
    station.bridge.disconnect()


app = FastAPI(title="Smart Trolley Control Station", version="1.0.0", lifespan=lifespan)
app.mount("/static", StaticFiles(directory=Path(__file__).parent / "static"), name="static")


@app.get("/")
def index() -> FileResponse:
    return FileResponse(Path(__file__).parent / "static" / "index.html")


@app.get("/api/state")
def state() -> dict[str, Any]:
    return station.snapshot()


@app.get("/api/ports")
def ports() -> list[dict[str, str]]:
    return available_ports()


@app.post("/api/mode/{mode}")
def set_mode(mode: str) -> dict[str, Any]:
    if mode not in {"simulation", "hardware"}:
        raise HTTPException(422, "mode must be simulation or hardware")
    station.mode = mode
    station.event("MODE_CHANGED", mode.upper())
    return station.snapshot()


@app.post("/api/connect")
def connect(payload: dict[str, Any]) -> dict[str, Any]:
    port = str(payload.get("port", "")).strip()
    baud_rate = int(payload.get("baud_rate", 115200))
    if not port or baud_rate < 9600 or baud_rate > 2_000_000:
        raise HTTPException(422, "valid serial port and baud rate are required")
    try:
        station.bridge.connect(port, baud_rate)
    except (RuntimeError, OSError) as exc:
        raise HTTPException(503, str(exc)) from exc
    return station.bridge.snapshot()


@app.post("/api/disconnect")
def disconnect() -> dict[str, Any]:
    station.bridge.disconnect()
    return station.bridge.snapshot()


@app.post("/api/control")
def control(payload: dict[str, Any]) -> dict[str, Any]:
    kind = str(payload.get("type", ""))
    try:
        if kind == "motor.manual":
            linear, angular = validate_motion(payload.get("linear_mps"), payload.get("angular_radps"), 0.45, 2.2)
            command = {"type": kind, "linear_mps": linear, "angular_radps": angular, "heartbeat": bool(payload.get("heartbeat", False))}
        elif kind in {"estop", "estop.reset", "config.get", "config.save", "config.defaults", "pid.test.stop", "navigation.stop"}:
            command = {"type": kind}
        elif kind == "navigation.destination":
            node_id = int(payload.get("node_id"))
            if node_id not in {int(n["id"]) for n in station.graph["nodes"]}:
                raise ValidationError("unknown graph node")
            command = {"type": kind, "node_id": node_id}
        elif kind == "pid.set":
            motor = str(payload.get("motor"))
            if motor not in {"left", "right"}:
                raise ValidationError("motor must be left or right")
            values = {name: float(payload[name]) for name in ("kp", "ki", "kd")}
            if not all(abs(value) < 1_000_000 for value in values.values()):
                raise ValidationError("PID values must be finite and reasonable")
            command = {"type": kind, "motor": motor, **values}
        else:
            raise ValidationError("unsupported control type")
        station.bridge.send(command)
        station.event("COMMAND_SENT", kind)
        return {"ok": True, "command": command}
    except (ValidationError, RuntimeError, ValueError) as exc:
        raise HTTPException(422, str(exc)) from exc


@app.post("/api/map")
def update_map(document: dict[str, Any]) -> dict[str, Any]:
    try:
        station.map = validate_map(document)
        validate_scene(station.scene, station.map)
        atomic_json_write(CONFIG / "map.json", station.map)
        station.event("MAP_SAVED", "map metadata updated")
        return station.map
    except ValidationError as exc:
        raise HTTPException(422, str(exc)) from exc


@app.post("/api/scene")
def update_scene(document: dict[str, Any]) -> dict[str, Any]:
    try:
        station.scene = validate_scene(document, station.map)
        atomic_json_write(RUNTIME_SCENE, station.scene)
        station.event("SCENE_SAVED", f"{len(station.scene['scene']['obstacles'])} obstacle(s)")
        return station.scene
    except ValidationError as exc:
        raise HTTPException(422, str(exc)) from exc


@app.post("/api/route")
def route(payload: dict[str, Any]) -> dict[str, Any]:
    try:
        planned = plan_route(station.graph, station.scene, int(payload["start"]), int(payload["destination"]))
        station.event("PATH_GENERATED", " -> ".join(map(str, planned.nodes)))
        return {"nodes": planned.nodes, "distance_m": planned.distance_m}
    except (KeyError, ValueError, ValidationError) as exc:
        raise HTTPException(422, str(exc)) from exc


@app.websocket("/ws")
async def websocket(websocket: WebSocket) -> None:
    await websocket.accept()
    try:
        while True:
            await websocket.send_json(station.snapshot())
            await asyncio.sleep(0.10)
    except (WebSocketDisconnect, RuntimeError):
        return


if __name__ == "__main__":
    uvicorn.run(app, host="127.0.0.1", port=8000, log_level="info")
