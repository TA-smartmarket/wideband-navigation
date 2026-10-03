"""ctypes binding to the shared C++ navigation core.

The desktop simulator must not reimplement Dijkstra, the waypoint manager, the
PID or the state machine: it drives the exact same C++ code that the ESP32-S3
firmware runs, compiled into a host shared library.  This module is the only
place that knows about ctypes.

Build the library with ``scripts/build_native_core.py`` (or let this module
build it on demand).
"""

from __future__ import annotations

import ctypes
import json
import os
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path

# --- ABI constants (must match navigation_core/include/navigation/c_api.h) ----
NAV_MAX_ROUTE_NODES = 96
NAV_MAX_ID_LEN = 24
NAV_MAX_ERROR_LEN = 160

REPO_ROOT = Path(__file__).resolve().parents[2]
CORE_INCLUDE = REPO_ROOT / "navigation_core" / "include"
CORE_SRC = REPO_ROOT / "navigation_core" / "src"


class NavState:
    """Mirror of nav::NavState (values are part of the ABI)."""

    BOOT = 0
    IDLE = 1
    WAITING_FOR_POSITION = 2
    READY = 3
    PLANNING = 4
    NAVIGATING = 5
    REPLANNING = 6
    POSITION_LOST = 7
    ARRIVED = 8
    ERROR = 9
    EMERGENCY_STOP = 10

    NAMES = {
        BOOT: "BOOT",
        IDLE: "IDLE",
        WAITING_FOR_POSITION: "WAITING_FOR_POSITION",
        READY: "READY",
        PLANNING: "PLANNING",
        NAVIGATING: "NAVIGATING",
        REPLANNING: "REPLANNING",
        POSITION_LOST: "POSITION_LOST",
        ARRIVED: "ARRIVED",
        ERROR: "ERROR",
        EMERGENCY_STOP: "EMERGENCY_STOP",
    }

    @classmethod
    def name(cls, value: int) -> str:
        return cls.NAMES.get(int(value), f"UNKNOWN({value})")


class MeasurementStatus:
    """Mirror of nav::MeasurementStatus."""

    OK = 0
    NO_DATA = 1
    BAD_SCHEMA_VERSION = 2
    WRONG_FRAME_ID = 3
    WRONG_TROLLEY_ID = 4
    NON_FINITE_COORDINATE = 5
    INVALID_FLAG = 6
    LOW_QUALITY = 7
    OUT_OF_MAP_BOUNDS = 8
    STALE_TIMESTAMP = 9

    NAMES = {
        OK: "ok",
        NO_DATA: "no data",
        BAD_SCHEMA_VERSION: "unsupported schema_version",
        WRONG_FRAME_ID: "frame_id is not smart_market_map",
        WRONG_TROLLEY_ID: "trolley_id mismatch",
        NON_FINITE_COORDINATE: "coordinate is NaN or infinite",
        INVALID_FLAG: "valid == false",
        LOW_QUALITY: "quality below minimum",
        OUT_OF_MAP_BOUNDS: "position outside map limits",
        STALE_TIMESTAMP: "timestamp invalid or stale",
    }

    @classmethod
    def name(cls, value: int) -> str:
        return cls.NAMES.get(int(value), f"UNKNOWN({value})")


class NavStatus(ctypes.Structure):
    """Mirror of the ``nav_status`` struct in c_api.h.

    Field order and types must match exactly; ``_check`` in
    :func:`_validate_abi` guards against silent drift.
    """

    _fields_ = [
        ("state", ctypes.c_int),
        ("error", ctypes.c_int),
        ("x_m", ctypes.c_double),
        ("y_m", ctypes.c_double),
        ("heading_rad", ctypes.c_double),
        ("heading_valid", ctypes.c_int),
        ("position_quality", ctypes.c_double),
        ("position_valid", ctypes.c_int),
        ("position_fresh", ctypes.c_int),
        ("start_node", ctypes.c_int),
        ("destination_node", ctypes.c_int),
        ("active_waypoint_node", ctypes.c_int),
        ("waypoint_index", ctypes.c_int),
        ("waypoint_count", ctypes.c_int),
        ("distance_to_waypoint_m", ctypes.c_double),
        ("distance_to_destination_m", ctypes.c_double),
        ("planned_distance_m", ctypes.c_double),
        ("remaining_distance_m", ctypes.c_double),
        ("travelled_distance_m", ctypes.c_double),
        ("target_bearing_rad", ctypes.c_double),
        ("heading_error_rad", ctypes.c_double),
        ("cross_track_error_m", ctypes.c_double),
        ("linear_velocity_mps", ctypes.c_double),
        ("angular_velocity_radps", ctypes.c_double),
        ("left_motor", ctypes.c_double),
        ("right_motor", ctypes.c_double),
        ("rotate_in_place", ctypes.c_int),
        ("replan_count", ctypes.c_uint32),
        ("position_loss_events", ctypes.c_uint32),
        ("invalid_sample_count", ctypes.c_uint32),
        ("plan_time_us", ctypes.c_uint32),
        ("route_length", ctypes.c_int),
        ("route", ctypes.c_int * NAV_MAX_ROUTE_NODES),
        ("position_age_ms", ctypes.c_uint64),
    ]


@dataclass
class NavSnapshot:
    """Python view of one navigation status snapshot."""

    state: int
    error: int
    x_m: float
    y_m: float
    heading_rad: float
    heading_valid: bool
    position_quality: float
    position_valid: bool
    position_fresh: bool
    start_node: int
    destination_node: int
    active_waypoint_node: int
    waypoint_index: int
    waypoint_count: int
    distance_to_waypoint_m: float
    distance_to_destination_m: float
    planned_distance_m: float
    remaining_distance_m: float
    travelled_distance_m: float
    target_bearing_rad: float
    heading_error_rad: float
    cross_track_error_m: float
    linear_velocity_mps: float
    angular_velocity_radps: float
    left_motor: float
    right_motor: float
    rotate_in_place: bool
    replan_count: int
    position_loss_events: int
    invalid_sample_count: int
    plan_time_us: int
    route: list[int] = field(default_factory=list)
    position_age_ms: int = 0

    @property
    def state_name(self) -> str:
        return NavState.name(self.state)

    @property
    def motion_allowed(self) -> bool:
        return self.state == NavState.NAVIGATING


def _validate_abi(lib: ctypes.CDLL) -> None:
    """Fail loudly when the loaded library does not match this Python binding."""
    lib.nav_state_name.restype = ctypes.c_char_p
    lib.nav_state_name.argtypes = [ctypes.c_int]
    for value, name in NavState.NAMES.items():
        got = lib.nav_state_name(value)
        got_text = got.decode("ascii") if got else ""
        if got_text != name:
            raise RuntimeError(
                f"navigation core ABI mismatch: nav_state_name({value}) returned "
                f"{got_text!r}, expected {name!r}. Rebuild the native core "
                f"(python scripts/build_native_core.py --force)."
            )


def _candidate_library_paths() -> list[Path]:
    names = ["navcore.dll", "libnavcore.so", "libnavcore.dylib"]
    roots = [REPO_ROOT / "build" / "native", REPO_ROOT / "simulator" / "build"]
    return [root / name for root in roots for name in names]


def build_native_core(force: bool = False, verbose: bool = False) -> Path:
    """Compile the navigation core into a shared library.

    Uses the system C++ compiler (g++/clang++).  Returns the library path.
    """
    script = REPO_ROOT / "scripts" / "build_native_core.py"
    if not script.is_file():
        raise RuntimeError(f"missing build helper: {script}")
    cmd = [sys.executable, str(script)]
    if force:
        cmd.append("--force")
    if verbose:
        cmd.append("--verbose")
    result = subprocess.run(cmd, capture_output=True, text=True, cwd=str(REPO_ROOT))
    if result.returncode != 0:
        raise RuntimeError(
            "failed to build the native navigation core:\n"
            f"{result.stdout}\n{result.stderr}"
        )
    if verbose:
        print(result.stdout)
    return Path(result.stdout.strip().splitlines()[-1])


def load_library(auto_build: bool = True, verbose: bool = False) -> ctypes.CDLL:
    """Load the shared navigation core, building it when necessary."""
    last_error: Exception | None = None
    for path in _candidate_library_paths():
        if path.is_file():
            try:
                lib = ctypes.CDLL(str(path))
                _validate_abi(lib)
                return lib
            except OSError as exc:  # pragma: no cover - platform specific
                last_error = exc
    if not auto_build:
        raise FileNotFoundError(
            f"native navigation core not found (looked in {_candidate_library_paths()})"
        ) from last_error
    path = build_native_core(verbose=verbose)
    lib = ctypes.CDLL(str(path))
    _validate_abi(lib)
    return lib


class NavigationCore:
    """Thin, typed wrapper around one native navigation session."""

    def __init__(self, graph_json: str, config_json: str | None = None, library: ctypes.CDLL | None = None):
        self._handle = None  # set before any operation that may raise
        self._lib = library if library is not None else load_library()
        self._declare_signatures()
        graph_text = graph_json.encode("utf-8")
        config_text = config_json.encode("utf-8") if config_json else None
        self._handle = self._lib.nav_create(graph_text, config_text)
        if not self._handle:
            message = self._lib.nav_last_error()
            raise RuntimeError(
                "nav_create failed: " + (message.decode("utf-8", "replace") if message else "unknown")
            )

    def _declare_signatures(self) -> None:
        lib = self._lib
        lib.nav_create.restype = ctypes.c_void_p
        lib.nav_create.argtypes = [ctypes.c_char_p, ctypes.c_char_p]
        lib.nav_destroy.restype = None
        lib.nav_destroy.argtypes = [ctypes.c_void_p]
        lib.nav_last_error.restype = ctypes.c_char_p
        lib.nav_last_error.argtypes = []

        lib.nav_request_destination.restype = ctypes.c_int
        lib.nav_request_destination.argtypes = [ctypes.c_void_p, ctypes.c_int]
        lib.nav_start.restype = ctypes.c_int
        lib.nav_start.argtypes = [ctypes.c_void_p]
        lib.nav_cancel.restype = None
        lib.nav_cancel.argtypes = [ctypes.c_void_p]
        lib.nav_stop.restype = None
        lib.nav_stop.argtypes = [ctypes.c_void_p]
        lib.nav_emergency_stop.restype = None
        lib.nav_emergency_stop.argtypes = [ctypes.c_void_p]
        lib.nav_clear_emergency_stop.restype = None
        lib.nav_clear_emergency_stop.argtypes = [ctypes.c_void_p]
        lib.nav_request_replan.restype = None
        lib.nav_request_replan.argtypes = [ctypes.c_void_p]
        lib.nav_reset.restype = None
        lib.nav_reset.argtypes = [ctypes.c_void_p]

        lib.nav_submit_position.restype = ctypes.c_int
        lib.nav_submit_position.argtypes = [
            ctypes.c_void_p,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_uint64,
            ctypes.c_int,
        ]
        lib.nav_update.restype = None
        lib.nav_update.argtypes = [ctypes.c_void_p, ctypes.c_uint64]

        lib.nav_get_status.restype = ctypes.c_int
        lib.nav_get_status.argtypes = [ctypes.c_void_p, ctypes.POINTER(NavStatus)]
        lib.nav_get_telemetry_json.restype = ctypes.c_int
        lib.nav_get_telemetry_json.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_uint64,
            ctypes.c_char_p,
            ctypes.c_int,
        ]
        lib.nav_get_status_text.restype = ctypes.c_int
        lib.nav_get_status_text.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_int,
        ]

        lib.nav_plan_route.restype = ctypes.c_int
        lib.nav_plan_route.argtypes = [
            ctypes.c_char_p,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.POINTER(ctypes.c_int),
            ctypes.c_int,
            ctypes.POINTER(ctypes.c_double),
        ]
        lib.nav_find_nearest_node.restype = ctypes.c_int
        lib.nav_find_nearest_node.argtypes = [
            ctypes.c_char_p,
            ctypes.c_double,
            ctypes.c_double,
            ctypes.c_double,
        ]
        lib.nav_graph_node_count.restype = ctypes.c_int
        lib.nav_graph_node_count.argtypes = [ctypes.c_char_p]
        lib.nav_graph_edge_count.restype = ctypes.c_int
        lib.nav_graph_edge_count.argtypes = [ctypes.c_char_p]

    # -- lifecycle ---------------------------------------------------------
    def close(self) -> None:
        if self._handle:
            self._lib.nav_destroy(self._handle)
            self._handle = None

    def __del__(self):
        self.close()

    def __enter__(self) -> "NavigationCore":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # -- commands ----------------------------------------------------------
    def request_destination(self, node_id: int) -> bool:
        return bool(self._lib.nav_request_destination(self._handle, int(node_id)))

    def start(self) -> bool:
        return bool(self._lib.nav_start(self._handle))

    def cancel(self) -> None:
        self._lib.nav_cancel(self._handle)

    def stop(self) -> None:
        self._lib.nav_stop(self._handle)

    def emergency_stop(self) -> None:
        self._lib.nav_emergency_stop(self._handle)

    def clear_emergency_stop(self) -> None:
        self._lib.nav_clear_emergency_stop(self._handle)

    def request_replan(self) -> None:
        self._lib.nav_request_replan(self._handle)

    def reset(self) -> None:
        self._lib.nav_reset(self._handle)

    # -- inputs ------------------------------------------------------------
    def submit_position(self, x_m: float, y_m: float, quality: float, timestamp_ms: int,
                        valid: bool = True) -> int:
        """Feed one UWB sample; returns a MeasurementStatus code."""
        return int(
            self._lib.nav_submit_position(
                self._handle, float(x_m), float(y_m), float(quality), int(timestamp_ms),
                1 if valid else 0,
            )
        )

    def update(self, now_ms: int) -> None:
        """Run one control cycle of the C++ state machine."""
        self._lib.nav_update(self._handle, int(now_ms))

    # -- observation -------------------------------------------------------
    def status(self) -> NavSnapshot:
        raw = NavStatus()
        if not self._lib.nav_get_status(self._handle, ctypes.byref(raw)):
            raise RuntimeError("nav_get_status failed")
        return NavSnapshot(
            state=raw.state,
            error=raw.error,
            x_m=raw.x_m,
            y_m=raw.y_m,
            heading_rad=raw.heading_rad,
            heading_valid=bool(raw.heading_valid),
            position_quality=raw.position_quality,
            position_valid=bool(raw.position_valid),
            position_fresh=bool(raw.position_fresh),
            start_node=raw.start_node,
            destination_node=raw.destination_node,
            active_waypoint_node=raw.active_waypoint_node,
            waypoint_index=raw.waypoint_index,
            waypoint_count=raw.waypoint_count,
            distance_to_waypoint_m=raw.distance_to_waypoint_m,
            distance_to_destination_m=raw.distance_to_destination_m,
            planned_distance_m=raw.planned_distance_m,
            remaining_distance_m=raw.remaining_distance_m,
            travelled_distance_m=raw.travelled_distance_m,
            target_bearing_rad=raw.target_bearing_rad,
            heading_error_rad=raw.heading_error_rad,
            cross_track_error_m=raw.cross_track_error_m,
            linear_velocity_mps=raw.linear_velocity_mps,
            angular_velocity_radps=raw.angular_velocity_radps,
            left_motor=raw.left_motor,
            right_motor=raw.right_motor,
            rotate_in_place=bool(raw.rotate_in_place),
            replan_count=raw.replan_count,
            position_loss_events=raw.position_loss_events,
            invalid_sample_count=raw.invalid_sample_count,
            plan_time_us=raw.plan_time_us,
            route=[int(raw.route[i]) for i in range(max(0, min(raw.route_length, NAV_MAX_ROUTE_NODES)))],
            position_age_ms=raw.position_age_ms,
        )

    def telemetry_json(self, trolley_id: str, timestamp_ms: int, buffer_size: int = 2048) -> str:
        buffer = ctypes.create_string_buffer(buffer_size)
        written = self._lib.nav_get_telemetry_json(
            self._handle, trolley_id.encode("utf-8"), int(timestamp_ms), buffer, buffer_size
        )
        if written <= 0:
            return "{}"
        return buffer.value.decode("utf-8", "replace")

    def status_text(self, trolley_id: str, buffer_size: int = 2048) -> str:
        buffer = ctypes.create_string_buffer(buffer_size)
        written = self._lib.nav_get_status_text(
            self._handle, trolley_id.encode("utf-8"), buffer, buffer_size
        )
        if written <= 0:
            return ""
        return buffer.value.decode("utf-8", "replace")

    # -- standalone helpers ------------------------------------------------
    def plan_route(self, graph_json: str, start_node: int, destination_node: int):
        """Plan a route with the C++ Dijkstra.  Returns (nodes, distance) or (None, 0)."""
        nodes = (ctypes.c_int * NAV_MAX_ROUTE_NODES)()
        distance = ctypes.c_double(0.0)
        count = self._lib.nav_plan_route(
            graph_json.encode("utf-8"), int(start_node), int(destination_node), nodes,
            NAV_MAX_ROUTE_NODES, ctypes.byref(distance),
        )
        if count <= 0:
            return None, 0.0
        return [int(nodes[i]) for i in range(count)], float(distance.value)

    def find_nearest_node(self, graph_json: str, x_m: float, y_m: float,
                          max_distance_m: float) -> int:
        return int(
            self._lib.nav_find_nearest_node(
                graph_json.encode("utf-8"), float(x_m), float(y_m), float(max_distance_m)
            )
        )

    def graph_counts(self, graph_json: str):
        encoded = graph_json.encode("utf-8")
        return (
            int(self._lib.nav_graph_node_count(encoded)),
            int(self._lib.nav_graph_edge_count(encoded)),
        )


class StepperAxis:
    """One stepper axis (NEMA 17 + A4988 class), backed by the C++ core.

    Position is the number of STEP pulses emitted, so this gives genuine
    step-based odometry rather than a velocity integral.
    """

    def __init__(self, steps_per_revolution: int = 200, microsteps: int = 16,
                 wheel_radius_m: float = 0.05, max_step_rate_hz: float = 20000.0,
                 max_step_accel_hz_per_s: float = 20000.0, invert_direction: bool = False,
                 library: ctypes.CDLL | None = None):
        self._handle = None  # set before anything that may raise
        self._lib = library if library is not None else load_library()
        self._steps_per_revolution = int(steps_per_revolution)
        self._microsteps = int(microsteps)
        self._wheel_radius_m = float(wheel_radius_m)
        self._declare_signatures()
        self._handle = self._lib.nav_stepper_create(
            int(steps_per_revolution), int(microsteps), float(wheel_radius_m),
            float(max_step_rate_hz), float(max_step_accel_hz_per_s),
            1 if invert_direction else 0,
        )
        if not self._handle:
            message = self._lib.nav_last_error()
            raise RuntimeError("nav_stepper_create failed: " +
                               (message.decode("utf-8", "replace") if message else "unknown"))

    def _declare_signatures(self) -> None:
        lib = self._lib
        lib.nav_stepper_create.restype = ctypes.c_void_p
        lib.nav_stepper_create.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_double,
                                           ctypes.c_double, ctypes.c_double, ctypes.c_int]
        lib.nav_stepper_destroy.restype = None
        lib.nav_stepper_destroy.argtypes = [ctypes.c_void_p]
        lib.nav_stepper_reset.restype = None
        lib.nav_stepper_reset.argtypes = [ctypes.c_void_p]
        lib.nav_stepper_zero_position.restype = None
        lib.nav_stepper_zero_position.argtypes = [ctypes.c_void_p]
        lib.nav_stepper_set_velocity.restype = None
        lib.nav_stepper_set_velocity.argtypes = [ctypes.c_void_p, ctypes.c_double]
        lib.nav_stepper_set_rpm.restype = None
        lib.nav_stepper_set_rpm.argtypes = [ctypes.c_void_p, ctypes.c_double]
        lib.nav_stepper_move_to.restype = None
        lib.nav_stepper_move_to.argtypes = [ctypes.c_void_p, ctypes.c_longlong]
        lib.nav_stepper_move_revolutions.restype = None
        lib.nav_stepper_move_revolutions.argtypes = [ctypes.c_void_p, ctypes.c_double]
        lib.nav_stepper_advance.restype = ctypes.c_int
        lib.nav_stepper_advance.argtypes = [ctypes.c_void_p, ctypes.c_double]
        for name, restype in (("nav_stepper_position_steps", ctypes.c_longlong),
                              ("nav_stepper_target_steps", ctypes.c_longlong),
                              ("nav_stepper_emitted_steps", ctypes.c_ulonglong)):
            getattr(lib, name).restype = restype
            getattr(lib, name).argtypes = [ctypes.c_void_p]
        for name in ("nav_stepper_distance_m", "nav_stepper_revolutions",
                     "nav_stepper_current_rate_hz", "nav_stepper_current_rpm"):
            getattr(lib, name).restype = ctypes.c_double
            getattr(lib, name).argtypes = [ctypes.c_void_p]
        for name in ("nav_stepper_direction", "nav_stepper_position_reached"):
            getattr(lib, name).restype = ctypes.c_int
            getattr(lib, name).argtypes = [ctypes.c_void_p]
        lib.nav_stepper_steps_per_meter.restype = ctypes.c_double
        lib.nav_stepper_steps_per_meter.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_double]

    def close(self) -> None:
        if self._handle:
            self._lib.nav_stepper_destroy(self._handle)
            self._handle = None

    def __del__(self):
        self.close()

    # -- commands ---------------------------------------------------------
    def reset(self) -> None:
        self._lib.nav_stepper_reset(self._handle)

    def zero_position(self) -> None:
        self._lib.nav_stepper_zero_position(self._handle)

    def set_velocity(self, velocity_mps: float) -> None:
        self._lib.nav_stepper_set_velocity(self._handle, float(velocity_mps))

    def set_rpm(self, rpm: float) -> None:
        self._lib.nav_stepper_set_rpm(self._handle, float(rpm))

    def move_to(self, target_steps: int) -> None:
        self._lib.nav_stepper_move_to(self._handle, int(target_steps))

    def move_revolutions(self, revolutions: float) -> None:
        self._lib.nav_stepper_move_revolutions(self._handle, float(revolutions))

    def advance(self, dt_s: float) -> int:
        """Advance the profile; returns the STEP pulses to emit this cycle."""
        return int(self._lib.nav_stepper_advance(self._handle, float(dt_s)))

    # -- observation ------------------------------------------------------
    @property
    def position_steps(self) -> int:
        return int(self._lib.nav_stepper_position_steps(self._handle))

    @property
    def target_steps(self) -> int:
        return int(self._lib.nav_stepper_target_steps(self._handle))

    @property
    def distance_m(self) -> float:
        return float(self._lib.nav_stepper_distance_m(self._handle))

    @property
    def revolutions(self) -> float:
        return float(self._lib.nav_stepper_revolutions(self._handle))

    @property
    def current_rate_hz(self) -> float:
        return float(self._lib.nav_stepper_current_rate_hz(self._handle))

    @property
    def current_rpm(self) -> float:
        return float(self._lib.nav_stepper_current_rpm(self._handle))

    @property
    def direction(self) -> int:
        return int(self._lib.nav_stepper_direction(self._handle))

    @property
    def position_reached(self) -> bool:
        return bool(self._lib.nav_stepper_position_reached(self._handle))

    @property
    def emitted_steps(self) -> int:
        return int(self._lib.nav_stepper_emitted_steps(self._handle))

    @property
    def steps_per_meter(self) -> float:
        """STEP pulses per meter of ground travel for this axis."""
        return float(self._lib.nav_stepper_steps_per_meter(
            self._steps_per_revolution, self._microsteps, self._wheel_radius_m))


def load_graph_json(path: str | Path) -> str:
    return Path(path).read_text(encoding="utf-8")


def load_config_json(path: str | Path | None) -> str | None:
    if path is None:
        return None
    return Path(path).read_text(encoding="utf-8")


def graph_summary(graph_json: str) -> dict:
    """Parse a graph JSON document for rendering (nodes/edges with coordinates).

    Rendering metadata is read in Python because it is pure presentation data;
    every *navigation* decision still comes from the C++ core.
    """
    document = json.loads(graph_json)
    nodes = {
        int(node["id"]): (float(node["x_m"]), float(node["y_m"]), str(node.get("type", "intersection")))
        for node in document.get("nodes", [])
    }
    edges = [
        (int(edge["from"]), int(edge["to"])) for edge in document.get("edges", [])
    ]
    return {
        "map": document.get("map", {}),
        "nodes": nodes,
        "edges": edges,
        "raw": document,
    }


def core_library_path() -> Path | None:
    for path in _candidate_library_paths():
        if path.is_file():
            return path
    return None


def core_is_built() -> bool:
    return core_library_path() is not None


__all__ = [
    "StepperAxis",
    "NavState",
    "MeasurementStatus",
    "NavStatus",
    "NavSnapshot",
    "NavigationCore",
    "load_library",
    "build_native_core",
    "load_graph_json",
    "load_config_json",
    "graph_summary",
    "core_is_built",
    "core_library_path",
    "REPO_ROOT",
]
