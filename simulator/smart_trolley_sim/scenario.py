"""Predefined, reproducible simulation scenarios.

Each scenario is a small declarative description (start pose, destination,
UWB fault schedule, timed events).  The engine interprets it, so adding a
scenario never requires touching the simulation loop.

All scenarios are deterministic for a fixed ``random_seed``: they only use the
seeded RNG owned by the UWB simulator.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

from .config import REPO_ROOT


@dataclass
class TimedEvent:
    """An action applied when the simulation reaches ``at_s`` seconds."""

    at_s: float
    action: str
    value: float | int | str | None = None
    #: Optional second argument (e.g. dropout burst length).
    value2: float | None = None
    fired: bool = False


@dataclass
class Scenario:
    name: str
    description: str
    start_x_m: float = 0.8
    start_y_m: float = 0.9
    start_theta_rad: float = 0.0
    destination_node: int = 11
    #: Destination switched to at ``destination_change_at_s`` (scenario 10).
    destination_change_node: int | None = None
    destination_change_at_s: float | None = None

    noise_std_m: float = 0.05
    noise_enabled: bool = True
    dropout_probability: float = 0.0
    dropout_enabled: bool = False
    invalid_probability: float = 0.0
    invalid_enabled: bool = False
    low_quality_enabled: bool = False
    frozen_enabled: bool = False
    quality_base: float = 0.97

    #: Per-scenario overrides of navigation parameters (path/motion/position).
    navigation_overrides: dict = field(default_factory=dict)
    events: list[TimedEvent] = field(default_factory=list)

    #: Stop the run when the destination is reached or the timeout expires.
    timeout_s: float = 180.0
    expect_success: bool = True
    #: True when the scenario deliberately ends outside ARRIVED (fault drills).
    expect_stop_or_arrival: bool = False
    #: Optional description of what the scenario demonstrates (printed/logged).
    expectation: str = ""

    def resolve_events(self) -> list[TimedEvent]:
        """Return fresh (unfired) event copies so a scenario can be replayed."""
        return [
            TimedEvent(at_s=e.at_s, action=e.action, value=e.value, value2=e.value2, fired=False)
            for e in self.events
        ]


def _scenarios_path() -> Path:
    return REPO_ROOT / "test_data" / "scenarios.json"


def builtin_scenarios() -> list[Scenario]:
    """The twelve scenarios required by the specification."""
    return [
        Scenario(
            name="scenario_01_basic_navigation",
            description="Straightforward navigation with a clean UWB signal.",
            start_x_m=0.8, start_y_m=0.9, destination_node=11,
            noise_std_m=0.02,
            expectation="Trolley follows 1 -> 2 -> 3 -> 7 -> 11 and stops at node 11.",
        ),
        Scenario(
            name="scenario_02_long_route",
            description="Longest route across the market (entry to the far corner).",
            start_x_m=1.5, start_y_m=2.0, destination_node=12,
            noise_std_m=0.02,
            expectation="Full diagonal traversal with several waypoints.",
        ),
        Scenario(
            name="scenario_03_multiple_turns",
            description="Many heading changes: exercises corner slowdown and turning.",
            start_x_m=1.6, start_y_m=6.0, destination_node=4,
            noise_std_m=0.02,
            expectation="Corner handling keeps the trolley inside the acceptance radius.",
        ),
        Scenario(
            name="scenario_04_moderate_uwb_noise",
            description="Moderate Gaussian UWB noise (sigma = 0.10 m).",
            start_x_m=0.8, start_y_m=0.9, destination_node=11,
            noise_std_m=0.10,
            expectation="Route completed with slightly higher cross-track error.",
        ),
        Scenario(
            name="scenario_05_high_uwb_noise",
            description="High Gaussian UWB noise (sigma = 0.20 m).",
            start_x_m=0.8, start_y_m=0.9, destination_node=11,
            noise_std_m=0.20,
            expectation="Navigation remains safe; deviation confirmation prevents replan storms.",
        ),
        Scenario(
            name="scenario_06_position_dropout",
            description="UWB samples stop arriving for 2 s mid-route.",
            start_x_m=0.8, start_y_m=0.9, destination_node=11,
            noise_std_m=0.05,
            events=[
                TimedEvent(at_s=6.0, action="dropout", value=2.0),
            ],
            expectation="POSITION_LOST stops the motors, then navigation resumes/replans.",
            expect_stop_or_arrival=True,
        ),
        Scenario(
            name="scenario_07_low_quality_position",
            description="Quality drops below the configured threshold mid-route.",
            start_x_m=0.8, start_y_m=0.9, destination_node=11,
            noise_std_m=0.05,
            events=[
                TimedEvent(at_s=6.0, action="low_quality_on"),
                TimedEvent(at_s=10.0, action="low_quality_off"),
            ],
            expectation="Low quality samples are rejected exactly like invalid ones.",
            expect_stop_or_arrival=True,
        ),
        Scenario(
            name="scenario_08_position_jump",
            description="Measured position jumps ~1.5 m away from the true pose.",
            start_x_m=0.8, start_y_m=0.9, destination_node=11,
            noise_std_m=0.05,
            events=[
                TimedEvent(at_s=6.0, action="position_jump", value=1.5),
                TimedEvent(at_s=8.0, action="clear_jump"),
            ],
            expectation="Deviation is confirmed after 500 ms and the route is replanned.",
            expect_stop_or_arrival=True,
        ),
        Scenario(
            name="scenario_09_route_deviation",
            description="Trolley is physically pushed off the corridor (plant disturbance).",
            start_x_m=0.8, start_y_m=0.9, destination_node=11,
            noise_std_m=0.05,
            events=[
                TimedEvent(at_s=6.0, action="teleport", value=1.2, value2=1.5),
            ],
            expectation="Cross-track error exceeds the replan threshold and Dijkstra runs again.",
            expect_stop_or_arrival=True,
        ),
        Scenario(
            name="scenario_10_destination_change",
            description="Destination is changed while the trolley is en route.",
            start_x_m=0.8, start_y_m=0.9, destination_node=11,
            destination_change_node=4, destination_change_at_s=6.0,
            noise_std_m=0.05,
            expectation="The new destination triggers a replan and the trolley diverts.",
            expect_stop_or_arrival=True,
        ),
        Scenario(
            name="scenario_11_unreachable_destination",
            description="Destination cannot be reached (isolated node added at runtime).",
            start_x_m=0.8, start_y_m=0.9, destination_node=99,
            noise_std_m=0.05,
            events=[
                TimedEvent(at_s=0.1, action="request_unreachable", value=99),
            ],
            expectation="Dijkstra reports no route, the FSM enters ERROR and motors stay stopped.",
            expect_success=False, expect_stop_or_arrival=True,
        ),
        Scenario(
            name="scenario_12_emergency_stop",
            description="Software emergency stop while navigating, then recovery.",
            start_x_m=0.8, start_y_m=0.9, destination_node=11,
            noise_std_m=0.05,
            events=[
                TimedEvent(at_s=6.0, action="emergency_stop"),
                TimedEvent(at_s=8.0, action="clear_emergency_stop"),
                TimedEvent(at_s=8.2, action="start_navigation"),
            ],
            expectation="Motors are cut instantly; after clearing, navigation resumes safely.",
            expect_stop_or_arrival=True,
        ),
        Scenario(
            name="scenario_13_invalid_samples",
            description="Positioning flags samples as invalid for 4 s mid-route.",
            start_x_m=0.8, start_y_m=0.9, destination_node=11,
            noise_std_m=0.05,
            events=[
                TimedEvent(at_s=6.0, action="invalid_on", value=0.5),
                TimedEvent(at_s=10.0, action="invalid_off"),
            ],
            expectation="Invalid samples are rejected; the position ages out and navigation recovers.",
            expect_stop_or_arrival=True,
        ),
        Scenario(
            name="scenario_14_frozen_position",
            description="UWB output freezes for 3 s while the trolley keeps moving.",
            start_x_m=0.8, start_y_m=0.9, destination_node=11,
            noise_std_m=0.05,
            events=[
                TimedEvent(at_s=6.0, action="frozen_on"),
                TimedEvent(at_s=9.0, action="frozen_off"),
            ],
            expectation="A frozen position is treated as stale data: navigation stops or replans safely.",
            expect_stop_or_arrival=True,
        ),
    ]


def scenario_names() -> list[str]:
    return [scenario.name for scenario in builtin_scenarios()]


def find_scenario(name: str) -> Scenario | None:
    for scenario in builtin_scenarios():
        if scenario.name == name:
            return scenario
    return None


def scenario_by_index(index: int) -> Scenario | None:
    """Scenario for the 1-9 keyboard shortcuts (1-based)."""
    scenarios = builtin_scenarios()
    if 1 <= index <= len(scenarios):
        return scenarios[index - 1]
    return None


def export_scenarios_json(path: Path | None = None) -> Path:
    """Write test_data/scenarios.json so the scenarios are inspectable."""
    target = path or _scenarios_path()
    target.parent.mkdir(parents=True, exist_ok=True)
    payload = []
    for scenario in builtin_scenarios():
        payload.append(
            {
                "name": scenario.name,
                "description": scenario.description,
                "start": {"x_m": scenario.start_x_m, "y_m": scenario.start_y_m,
                          "theta_rad": scenario.start_theta_rad},
                "destination_node": scenario.destination_node,
                "destination_change": {
                    "node": scenario.destination_change_node,
                    "at_s": scenario.destination_change_at_s,
                },
                "uwb": {
                    "noise_std_m": scenario.noise_std_m,
                    "noise_enabled": scenario.noise_enabled,
                    "dropout_probability": scenario.dropout_probability,
                    "dropout_enabled": scenario.dropout_enabled,
                    "invalid_probability": scenario.invalid_probability,
                    "invalid_enabled": scenario.invalid_enabled,
                    "low_quality_enabled": scenario.low_quality_enabled,
                    "frozen_enabled": scenario.frozen_enabled,
                    "quality_base": scenario.quality_base,
                },
                "events": [
                    {"at_s": e.at_s, "action": e.action, "value": e.value, "value2": e.value2}
                    for e in scenario.events
                ],
                "timeout_s": scenario.timeout_s,
                "expect_success": scenario.expect_success,
                "expect_stop_or_arrival": scenario.expect_stop_or_arrival,
                "expectation": scenario.expectation,
            }
        )
    with target.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2)
        handle.write("\n")
    return target


def load_scenarios_json(path: Path | None = None) -> list[Scenario]:
    """Load scenarios from JSON (used by the tests and by scripts)."""
    source = path or _scenarios_path()
    if not source.is_file():
        return builtin_scenarios()
    with source.open(encoding="utf-8") as handle:
        document = json.load(handle)
    scenarios = []
    for entry in document:
        start = entry.get("start", {})
        uwb = entry.get("uwb", {})
        change = entry.get("destination_change", {}) or {}
        scenarios.append(
            Scenario(
                name=str(entry["name"]),
                description=str(entry.get("description", "")),
                start_x_m=float(start.get("x_m", 0.8)),
                start_y_m=float(start.get("y_m", 0.9)),
                start_theta_rad=float(start.get("theta_rad", 0.0)),
                destination_node=int(entry.get("destination_node", 11)),
                destination_change_node=change.get("node"),
                destination_change_at_s=change.get("at_s"),
                noise_std_m=float(uwb.get("noise_std_m", 0.05)),
                noise_enabled=bool(uwb.get("noise_enabled", True)),
                dropout_probability=float(uwb.get("dropout_probability", 0.0)),
                dropout_enabled=bool(uwb.get("dropout_enabled", False)),
                invalid_probability=float(uwb.get("invalid_probability", 0.0)),
                invalid_enabled=bool(uwb.get("invalid_enabled", False)),
                low_quality_enabled=bool(uwb.get("low_quality_enabled", False)),
                frozen_enabled=bool(uwb.get("frozen_enabled", False)),
                quality_base=float(uwb.get("quality_base", 0.97)),
                events=[
                    TimedEvent(
                        at_s=float(event.get("at_s", 0.0)),
                        action=str(event.get("action", "")),
                        value=event.get("value"),
                        value2=event.get("value2"),
                    )
                    for event in entry.get("events", [])
                ],
                timeout_s=float(entry.get("timeout_s", 180.0)),
                expect_success=bool(entry.get("expect_success", True)),
                expect_stop_or_arrival=bool(entry.get("expect_stop_or_arrival", False)),
                expectation=str(entry.get("expectation", "")),
            )
        )
    return scenarios
