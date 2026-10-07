"""Simulator configuration (config/simulator.json).

Kept separate from the navigation configuration: this file describes the
*experiment environment* (window, physics, sensor model, logging), never the
navigation algorithm parameters.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG_PATH = REPO_ROOT / "config" / "simulator.json"


@dataclass
class WindowConfig:
    width_px: int = 1400
    height_px: int = 850
    margin_px: int = 40
    panel_width_px: int = 380
    caption: str = "Smart Trolley - UWB + Dijkstra Navigation Simulator"


@dataclass
class SimulationConfig:
    dt_s: float = 0.02
    speed_multiplier: float = 1.0
    random_seed: int = 12345
    max_duration_s: float = 180.0


@dataclass
class UwbDefaults:
    update_rate_hz: float = 10.0
    noise_std_m: float = 0.05
    dropout_probability: float = 0.0
    invalid_probability: float = 0.0
    quality_base: float = 0.97
    quality_noise: float = 0.01
    quality_noise_scale: float = 1.5
    frozen_position: bool = False

    #: First-order smoothing applied by the *positioning subsystem* before the
    #: sample is published, standing in for the EKF that the UWB team runs.
    #: The navigation contract states that navigation consumes the final
    #: *estimated* position, so the simulated sensor must emit an estimate, not
    #: raw white noise: feeding raw noise would make the trolley chase jitter
    #: that a real EKF would have removed.
    #:
    #: The gain adapts to the noise level, exactly as a Kalman gain falls as the
    #: measurement covariance grows.  This matters in both directions:
    #:   * with a clean sensor, heavy smoothing only adds *lag* (the estimate
    #:     trails the trolley through a turn), so the filter stays nearly open;
    #:   * with a noisy sensor, smoothing removes the jitter the controller
    #:     would otherwise chase.
    #: Measured over the scenario suite: adaptive smoothing cuts the sample error
    #: by ~35 % at sigma = 0.20 m while leaving a sigma = 0.02 m sensor untouched.
    filter_alpha: float = 1.0
    #: Smoothing floor reached at ``filter_alpha_reference_sigma`` and beyond.
    filter_alpha_min: float = 0.45
    #: Noise sigma at which ``filter_alpha_min`` is reached.
    filter_alpha_reference_sigma: float = 0.20


@dataclass
class PhysicsConfig:
    """Plant parameters.  These describe the *physical* trolley, not the
    controller: the controller lives in the C++ navigation core."""

    wheel_radius_m: float = 0.05
    wheel_base_m: float = 0.32
    max_wheel_speed_mps: float = 0.60
    motor_time_constant_s: float = 0.12
    motor_deadband_command: float = 0.03
    left_gain: float = 1.0
    right_gain: float = 0.97
    # Disturbance applied to the wheels, disabled by default so scenarios are
    # bit-for-bit reproducible unless explicitly enabled.
    wheel_noise_std_mps: float = 0.0

    #: Drive the simulated wheels through the C++ stepper model (STEP-rate
    #: conversion + acceleration profile) instead of the simple first-order lag.
    #: Off by default so the validated baseline is unchanged; enable it when
    #: studying the NEMA 17 / A4988 behaviour, at which point
    #: `wheel_odometry_distance_m` becomes a genuine pulse-count measurement.
    use_stepper_model: bool = False


@dataclass
class LoggingConfig:
    enabled: bool = True
    directory: str = "logs"
    results_directory: str = "results"
    telemetry_rate_hz: float = 20.0
    export_csv: bool = True
    export_summary: bool = True


@dataclass
class Benchmark:
    """Acceptance band for one experimental metric.

    `direction` is "max" when a smaller value is better (errors, counts) and
    "min" when a larger value is better (efficiency).  The bands only classify a
    run: exceeding one never aborts the simulation, it is reported.
    """

    pass_value: float
    warn_value: float
    direction: str = "max"

    def classify(self, value: float) -> str:
        if self.direction == "min":
            if value >= self.pass_value:
                return "PASS"
            return "WARNING" if value >= self.warn_value else "FAIL"
        if value <= self.pass_value:
            return "PASS"
        return "WARNING" if value <= self.warn_value else "FAIL"


#: Default acceptance bands.  Engineering targets, not hard limits; override in
#: config/simulator.json -> "benchmarks".
#: Default acceptance bands.  These are deliberately set to distinguish healthy
#: navigation from loss of control, not to encode the best case: the per-scenario
#: engineering targets (for example mean XTE <= 0.10 m on a clean, low-noise
#: route) are tighter than a band that must also accept a 0.20 m-noise run or a
#: deliberate deviation drill.
DEFAULT_BENCHMARKS: dict[str, Benchmark] = {
    # The destination acceptance radius is 0.15 m; measurement noise adds to the
    # final ground-truth distance, so a small margin above it is healthy.
    "destination_error_m": Benchmark(0.20, 0.30, "max"),
    # Waypoint tolerance is 0.20 m; a tracking controller that keeps the mean
    # error at or below it is doing its job.
    "mean_cross_track_error_m": Benchmark(0.25, 0.40, "max"),
    # A replan is triggered at 0.60 m of deviation, so excursions up to roughly
    # that value are expected on fault drills.
    "max_cross_track_error_m": Benchmark(0.80, 1.50, "max"),
    "path_efficiency_percent": Benchmark(90.0, 80.0, "min"),
    "replan_count": Benchmark(0.0, 2.0, "max"),
    "position_loss_events": Benchmark(0.0, 1.0, "max"),
    # The initial heading is unknown (UWB gives no yaw), so acquisition dominates
    # the mean; a persistent value above this would indicate a steering problem.
    "mean_absolute_heading_error_deg": Benchmark(30.0, 45.0, "max"),
    "uwb_rmse_m": Benchmark(0.25, 0.40, "max"),
}


@dataclass
class RenderConfig:
    show_graph_labels: bool = True
    show_trajectory: bool = True
    show_debug_overlay: bool = True
    trajectory_max_points: int = 4000
    #: Show every node's id while choosing a destination, so the operator can see
    #: which node a click will select.
    show_pick_hints: bool = True


@dataclass
class SimulatorConfig:
    window: WindowConfig = field(default_factory=WindowConfig)
    simulation: SimulationConfig = field(default_factory=SimulationConfig)
    uwb: UwbDefaults = field(default_factory=UwbDefaults)
    physics: PhysicsConfig = field(default_factory=PhysicsConfig)
    logging: LoggingConfig = field(default_factory=LoggingConfig)
    render: RenderConfig = field(default_factory=RenderConfig)
    #: How far a map click may be from a graph node for it to be accepted as a
    #: destination.  Defaults to the navigation snap distance, so a click far
    #: from the aisle network is refused instead of silently targeting a node
    #: metres away.
    max_pick_distance_m: float = 1.5
    benchmarks: dict = field(default_factory=lambda: dict(DEFAULT_BENCHMARKS))
    #: STEP generator limits mirrored from navigation.json -> stepper.common.
    stepper_max_step_rate_hz: float = 20000.0
    stepper_max_step_accel_hz_per_s: float = 20000.0

    default_scenario: str = "scenario_01_basic_navigation"
    map_path: str = "config/map.json"
    graph_path: str = "config/graph.json"
    navigation_config_path: str = "config/navigation.json"
    scene_path: str | None = None
    obstacle_clearance_m: float = 0.20

    def resolve(self, relative: str) -> Path:
        """Resolve a repository-relative path from the config file."""
        candidate = Path(relative)
        return candidate if candidate.is_absolute() else REPO_ROOT / candidate


def _build(dataclass_type, data: dict | None):
    """Instantiate a dataclass from a JSON object, ignoring unknown keys."""
    if not isinstance(data, dict):
        return dataclass_type()
    known = {f.name for f in dataclass_type.__dataclass_fields__.values()}
    return dataclass_type(**{k: v for k, v in data.items() if k in known})


def _build_benchmarks(data) -> dict:
    """Merge configured acceptance bands over the defaults."""
    benchmarks = dict(DEFAULT_BENCHMARKS)
    if not isinstance(data, dict):
        return benchmarks
    for name, entry in data.items():
        if not isinstance(entry, dict):
            continue
        try:
            pass_value = float(entry["pass"])
            warn_value = float(entry.get("warn", entry["pass"]))
        except (KeyError, TypeError, ValueError):
            continue
        direction = str(entry.get("direction", benchmarks[name].direction
                                   if name in benchmarks else "max"))
        benchmarks[name] = Benchmark(pass_value, warn_value, direction)
    return benchmarks


def load_simulator_config(path: str | Path | None = None) -> SimulatorConfig:
    """Load config/simulator.json, falling back to the built-in defaults."""
    config_path = Path(path) if path is not None else DEFAULT_CONFIG_PATH
    if not config_path.is_file():
        return SimulatorConfig()
    with config_path.open(encoding="utf-8") as handle:
        document = json.load(handle)
    return SimulatorConfig(
        window=_build(WindowConfig, document.get("window")),
        simulation=_build(SimulationConfig, document.get("simulation")),
        uwb=_build(UwbDefaults, document.get("uwb")),
        physics=_build(PhysicsConfig, document.get("physics")),
        logging=_build(LoggingConfig, document.get("logging")),
        render=_build(RenderConfig, document.get("render")),
        benchmarks=_build_benchmarks(document.get("benchmarks")),
        default_scenario=str(document.get("default_scenario", SimulatorConfig().default_scenario)),
        map_path=str(document.get("map_path", SimulatorConfig().map_path)),
        graph_path=str(document.get("graph_path", SimulatorConfig().graph_path)),
        navigation_config_path=str(
            document.get("navigation_config_path", SimulatorConfig().navigation_config_path)
        ),
        scene_path=(str(document["scene_path"]) if document.get("scene_path") else None),
        obstacle_clearance_m=float(
            document.get("obstacle_clearance_m", SimulatorConfig().obstacle_clearance_m)
        ),
    )
