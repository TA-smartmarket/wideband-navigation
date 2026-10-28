"""FastAPI entry point for the local Smart Trolley control station.

Run from the repository root: ``python dashboard/main.py``.
"""
from __future__ import annotations

import asyncio
import copy
from collections import deque
from contextlib import asynccontextmanager
from datetime import datetime
import json
import logging
from pathlib import Path
import sys
import threading
import time
from typing import Any

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
import uvicorn

ROOT = Path(__file__).resolve().parents[1]
if __package__ in (None, "") and str(ROOT) not in sys.path:
    # Direct script execution puts dashboard/ (not the repository root) on
    # sys.path.  Route previews load the shared C++ binding through the sibling
    # simulator package, so make the documented `python dashboard/main.py`
    # launch mode resolve repository packages exactly like `python -m` does.
    sys.path.insert(0, str(ROOT))

try:  # Supports both `python dashboard/main.py` and `python -m dashboard.main`.
    from .models import (ValidationError, build_visibility_edges, fit_nodes_to_room, plan_route,
                         validate_graph, validate_map, validate_motion, validate_pid_gains,
                         validate_scene)
    from .serial_bridge import PROTOCOL_VERSION, SerialBridge, available_ports
except ImportError:  # pragma: no cover - direct-script invocation
    from models import (ValidationError, build_visibility_edges, fit_nodes_to_room, plan_route,
                        validate_graph, validate_map, validate_motion, validate_pid_gains,
                        validate_scene)
    from serial_bridge import PROTOCOL_VERSION, SerialBridge, available_ports

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
        validate_scene(self.scene, self.map, self.graph)
        validate_graph(self.graph, self.map, self.scene)
        self.telemetry: dict[str, Any] = {"type": "telemetry", "mode": "IDLE", "source": "simulation", "timestamp_ms": 0}
        self.device_config: dict[str, Any] | None = None
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
        elif packet_type == "wheel_telemetry":
            with self.lock:
                self.telemetry.update(packet)
                self.telemetry["type"] = "telemetry"
            self._append_telemetry(packet)
        elif packet_type == "config":
            with self.lock:
                self.device_config = packet
            self.event("CONFIG", f"loaded device configuration revision {packet.get('revision', '?')}")
        elif packet_type in {"event", "ack", "error"}:
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
                    "device_config": self.device_config, "protocol_version": PROTOCOL_VERSION}


station = Station()


@asynccontextmanager
async def lifespan(_: FastAPI):
    yield
    station.bridge.disconnect()


app = FastAPI(title="Smart Trolley Control Station", version="1.0.0", lifespan=lifespan)
app.mount("/static", StaticFiles(directory=Path(__file__).parent / "static"), name="static")
THREE_BUILD = Path(__file__).parent / "node_modules" / "three" / "build"
THREE_EXAMPLES = Path(__file__).parent / "node_modules" / "three" / "examples" / "jsm"
if THREE_BUILD.exists() and THREE_EXAMPLES.exists():
    app.mount("/vendor/three", StaticFiles(directory=THREE_BUILD), name="three-build")
    app.mount("/vendor/three-examples", StaticFiles(directory=THREE_EXAMPLES), name="three-examples")


@app.get("/")
def index() -> FileResponse:
    if not THREE_BUILD.exists():
        raise HTTPException(503, "dashboard assets missing; run npm install --prefix dashboard")
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
        elif kind in {"estop", "estop.reset", "config.get", "config.save", "config.defaults", "pid.test.stop", "navigation.stop", "encoder.zero"}:
            command = {"type": kind}
        elif kind == "odometry.set_pose":
            values = {name: float(payload[name]) for name in ("x_m", "y_m", "heading_rad")}
            if not all(abs(value) <= 100.0 for value in values.values()) or abs(values["heading_rad"]) > 6.2831854:
                raise ValidationError("initial pose is invalid or out of range")
            command = {"type": kind, **values}
        elif kind == "position.source":
            source = str(payload.get("source", ""))
            if source not in {"encoder_odometry", "external_mock_uwb", "real_uwb"}:
                raise ValidationError("invalid position source")
            command = {"type": kind, "source": source}
        elif kind == "navigation.destination":
            node_id = int(payload.get("node_id"))
            if node_id not in {int(n["id"]) for n in station.graph["nodes"]}:
                raise ValidationError("unknown graph node")
            command = {"type": kind, "node_id": node_id}
        elif kind == "pid.set":
            motor = str(payload.get("motor"))
            if motor not in {"left", "right", "heading"}:
                raise ValidationError("PID target must be left, right or heading")
            kp, ki, kd = validate_pid_gains(payload.get("kp"), payload.get("ki"), payload.get("kd"))
            command = {"type": kind, "motor": motor, "kp": kp, "ki": ki, "kd": kd}
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
        validate_scene(station.scene, station.map, station.graph)
        atomic_json_write(CONFIG / "map.json", station.map)
        station.event("MAP_SAVED", "map metadata updated")
        return station.map
    except ValidationError as exc:
        raise HTTPException(422, str(exc)) from exc


@app.post("/api/graph")
def update_graph(document: dict[str, Any]) -> dict[str, Any]:
    """Save user-defined nodes and derive a sparse visibility graph."""
    try:
        candidate = copy.deepcopy(document)
        candidate["edges"] = []
        validate_graph(candidate, station.map, station.scene)
        candidate["edges"] = build_visibility_edges(candidate["nodes"], station.scene)
        validate_graph(candidate, station.map, station.scene)
        validate_scene(station.scene, station.map, candidate)
        atomic_json_write(CONFIG / "graph.json", candidate)
        with station.lock:
            station.graph = candidate
        station.event("GRAPH_SAVED", f"{len(candidate['nodes'])} node(s), {len(candidate['edges'])} visible edge(s)")
        return station.snapshot()
    except (KeyError, TypeError, ValueError, ValidationError) as exc:
        raise HTTPException(422, str(exc)) from exc


@app.post("/api/graph/auto-fit")
def auto_fit_graph() -> dict[str, Any]:
    """Reset all nodes to a tidy room-sized grid and rebuild visibility."""
    try:
        candidate = copy.deepcopy(station.graph)
        candidate["nodes"] = fit_nodes_to_room(candidate.get("nodes", []), station.map, station.scene)
        candidate["edges"] = build_visibility_edges(candidate["nodes"], station.scene)
        validate_graph(candidate, station.map, station.scene)
        validate_scene(station.scene, station.map, candidate)
        atomic_json_write(CONFIG / "graph.json", candidate)
        with station.lock:
            station.graph = candidate
        station.event("GRAPH_AUTO_FIT", f"{len(candidate['nodes'])} node(s) fitted to room")
        return station.snapshot()
    except (KeyError, TypeError, ValueError, ValidationError) as exc:
        raise HTTPException(422, str(exc)) from exc


@app.post("/api/room")
def update_room(payload: dict[str, Any]) -> dict[str, Any]:
    """Resize the dashboard room after every dependent object validates."""
    try:
        candidate_map = copy.deepcopy(station.map)
        candidate_map["width_m"] = payload.get("width_m")
        candidate_map["height_m"] = payload.get("depth_m")
        validate_map(candidate_map)

        candidate_scene = copy.deepcopy(station.scene)
        room = candidate_scene.setdefault("scene", {}).setdefault("room", {})
        room["height"] = payload.get("room_height_m")
        validate_scene(candidate_scene, candidate_map)

        candidate_graph = copy.deepcopy(station.graph)
        graph_map = candidate_graph.setdefault("map", {})
        graph_map["width_m"] = candidate_map["width_m"]
        graph_map["height_m"] = candidate_map["height_m"]
        candidate_graph["nodes"] = fit_nodes_to_room(
            candidate_graph.get("nodes", []), candidate_map, candidate_scene)
        candidate_graph["edges"] = build_visibility_edges(candidate_graph["nodes"], candidate_scene)
        validate_graph(candidate_graph, candidate_map, candidate_scene)
        validate_scene(candidate_scene, candidate_map, candidate_graph)

        atomic_json_write(CONFIG / "map.json", candidate_map)
        atomic_json_write(CONFIG / "graph.json", candidate_graph)
        atomic_json_write(RUNTIME_SCENE, candidate_scene)
        with station.lock:
            station.map = candidate_map
            station.graph = candidate_graph
            station.scene = candidate_scene
        station.event("ROOM_RESIZED", f"{candidate_map['width_m']} x {candidate_map['height_m']} x {room['height']} m")
        return station.snapshot()
    except (KeyError, TypeError, ValueError, ValidationError) as exc:
        raise HTTPException(422, str(exc)) from exc


@app.post("/api/scene")
def update_scene(document: dict[str, Any]) -> dict[str, Any]:
    try:
        candidate_scene = validate_scene(copy.deepcopy(document), station.map, station.graph)
        candidate_graph = copy.deepcopy(station.graph)
        candidate_graph["edges"] = build_visibility_edges(candidate_graph["nodes"], candidate_scene)
        validate_graph(candidate_graph, station.map, candidate_scene)
        atomic_json_write(RUNTIME_SCENE, candidate_scene)
        atomic_json_write(CONFIG / "graph.json", candidate_graph)
        with station.lock:
            station.scene = candidate_scene
            station.graph = candidate_graph
        station.event("SCENE_SAVED", f"{len(candidate_scene['scene']['obstacles'])} obstacle(s); graph visibility rebuilt")
        return station.snapshot()
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
