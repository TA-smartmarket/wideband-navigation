"""Smart Trolley desktop simulator package.

The simulator owns only what is environment specific:

* trolley physics (differential drive + first-order motor response),
* the UWB measurement model (noise, dropout, invalid samples, jumps),
* rendering and experiment logging.

Every navigation decision (Dijkstra, waypoint tracking, PID, state machine) is
made by the C++ navigation core through :mod:`smart_trolley_sim.native_core`, so
the simulator and the ESP32-S3 firmware run the same algorithms.
"""

from .config import (
    LoggingConfig,
    PhysicsConfig,
    RenderConfig,
    SimulatorConfig,
    SimulationConfig,
    UwbDefaults,
    WindowConfig,
    load_simulator_config,
)
from .map_loader import MarketMap, MapError, Node, Edge, load_market_map
from .trolley import TrolleyPlant, TrolleyState
from .uwb_simulator import UwbSimulator, UwbSample
from .native_core import NavigationCore, NavState, MeasurementStatus, NavSnapshot
from .scenario import Scenario, TimedEvent, builtin_scenarios, find_scenario, scenario_names
from .simulation import SimulationEngine, SimulationState
from .telemetry import ExperimentLogger, Metrics, MetricsAccumulator, TelemetryRecord

#: The UWB configuration type lives in config.py; expose it under the name used
#: by the simulator documentation.
UwbConfig = UwbDefaults

__all__ = [
    "SimulatorConfig",
    "SimulationConfig",
    "WindowConfig",
    "UwbDefaults",
    "UwbConfig",
    "PhysicsConfig",
    "LoggingConfig",
    "RenderConfig",
    "load_simulator_config",
    "MarketMap",
    "MapError",
    "Node",
    "Edge",
    "load_market_map",
    "TrolleyState",
    "TrolleyPlant",
    "UwbSimulator",
    "UwbSample",
    "NavigationCore",
    "NavState",
    "NavSnapshot",
    "MeasurementStatus",
    "Scenario",
    "TimedEvent",
    "builtin_scenarios",
    "find_scenario",
    "scenario_names",
    "SimulationEngine",
    "SimulationState",
    "ExperimentLogger",
    "Metrics",
    "MetricsAccumulator",
    "TelemetryRecord",
]

__version__ = "1.0.0"
