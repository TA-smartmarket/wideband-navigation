"""Simulation engine: wires the plant, the UWB model and the C++ navigation core.

The engine contains no navigation logic.  Each step it:

1. generates a UWB sample from the *true* pose (never handing ground truth to
   navigation),
2. submits it to the C++ navigation core,
3. runs one control cycle of the core,
4. applies the resulting motor commands to the simulated plant,
5. logs telemetry and updates metrics.

Because the core is the same code the firmware runs, the simulator is a
faithful behavioural model rather than a reimplementation.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from pathlib import Path

from .config import SimulatorConfig, load_simulator_config
from .map_loader import MarketMap, load_market_map
from .native_core import MeasurementStatus, NavState, NavigationCore
from .scenario import Scenario, TimedEvent
from .telemetry import ExperimentLogger, Metrics, MetricsAccumulator
from .trolley import TrolleyPlant
from .uwb_simulator import UwbSimulator

#: Node id used by the "unreachable destination" scenario.  The engine appends
#: an isolated node to the graph so Dijkstra genuinely has no route.
UNREACHABLE_NODE_ID = 99


@dataclass
class SimulationState:
    """Observable state of one running simulation (consumed by the renderer)."""

    running: bool = False
    paused: bool = False
    finished: bool = False
    sim_time_s: float = 0.0
    steps: int = 0
    true_pose: tuple[float, float, float] = (0.0, 0.0, 0.0)
    measured: tuple[float, float] | None = None
    uwb_fault: str = ""
    active_faults: list[str] = field(default_factory=list)
    last_measurement_status: int = MeasurementStatus.NO_DATA
    #: Live sample accounting (see Metrics for the final values).
    samples_received: int = 0
    samples_valid: int = 0
    samples_invalid: int = 0
    current_scenario: str = ""
    notes: list[str] = field(default_factory=list)


class SimulationEngine:
    """Headless simulation loop (the renderer drives it, or scripts run it)."""

    def __init__(self,
                 scenario: Scenario,
                 config: SimulatorConfig | None = None,
                 market_map: MarketMap | None = None,
                 run_index: int = 1):
        self.config = config or load_simulator_config()
        self.scenario = scenario
        self.run_index = run_index

        self.market_map = market_map or load_market_map(
            self.config.map_path, self.config.graph_path
        )

        # Deterministic per scenario: the seed is derived from the configured
        # seed so that scenario N always reproduces exactly.
        self.seed = self.config.simulation.random_seed
        self.rng = random.Random(self.seed)

        self.graph_json = self._prepare_graph()
        self.navigation_config_json = self._prepare_navigation_config()

        scene_json = None
        if self.config.scene_path:
            scene_path = self.config.resolve(self.config.scene_path)
            scene_json = scene_path.read_text(encoding="utf-8")
        self.core = NavigationCore(
            self.graph_json, self.navigation_config_json,
            scene_json=scene_json,
            obstacle_clearance_m=self.config.obstacle_clearance_m,
        )

        # Optional stepper drivetrain: the plant then drives its wheels through the
        # same STEP-rate conversion the A4988 firmware uses.
        self.stepper_pair = None
        if self.config.physics.use_stepper_model:
            from .stepper_drive import create_stepper_pair

            physics = self.config.physics
            self.stepper_pair = create_stepper_pair(
                wheel_radius_m=physics.wheel_radius_m,
                max_step_rate_hz=self.config.stepper_max_step_rate_hz,
                max_step_accel_hz_per_s=self.config.stepper_max_step_accel_hz_per_s,
            )
        self.plant = TrolleyPlant(self.config.physics, rng=self.rng)
        if self.stepper_pair is not None:
            self.plant.set_stepper(self.stepper_pair)
        self.uwb = UwbSimulator(self.config.uwb, seed=self.seed)
        self.logger = ExperimentLogger(self.config.logging, scenario.name, run_index)
        self.metrics_accumulator = MetricsAccumulator()
        self.metrics_accumulator.sample_counter_provider = lambda: (
            self._samples_submitted,
            self._samples_submitted - self._samples_rejected,
            self._samples_rejected,
            self.uwb.statistics.dropped,
        )
        # Wheel odometry is a property of the plant, so the engine supplies it.
        # Wheel odometry: with the stepper model it is a real STEP-pulse count,
        # otherwise the plant's own integrated wheel travel.
        if self.stepper_pair is not None:
            self.metrics_accumulator.odometry_distance_provider = (
                self.stepper_pair.odometry_distance_m
            )
        else:
            self.metrics_accumulator.odometry_distance_provider = (
                lambda: self.plant.state.distance_travelled_m
            )

        self.state = SimulationState(current_scenario=scenario.name)
        self.events: list[TimedEvent] = scenario.resolve_events()
        self._last_sample = None
        self._telemetry_accumulator_s = 0.0
        self._telemetry_period_s = 1.0 / max(self.config.logging.telemetry_rate_hz, 0.1)
        self._finish_reason = ""
        # After the destination is reached the controller keeps levelling the
        # heading, which makes the trolley arc around the node.  The run is only
        # declared finished once it has actually come to rest (or the grace
        # period expires), so the logged trajectory and the destination error
        # describe a settled trolley rather than one still slewing.
        self._settling = False
        self._settle_started_s = 0.0
        self._settle_grace_s = 3.0
        self._settle_speed_eps = 0.02
        self._samples_submitted = 0
        self._samples_rejected = 0
        #: Simulation clock offset.  The position contract requires a non-zero
        #: timestamp, so the clock must not start at exactly 0 - otherwise the
        #: very first (perfectly good) sample would be rejected as stale and
        #: inflate the invalid-sample counter.
        self._clock_offset_ms = 1000
        #: Where the trolley actually started.  Kept separate from the scenario so
        #: an overridden start still yields a correct ideal-path distance.
        self.effective_start_m = (scenario.start_x_m, scenario.start_y_m)

        self._configure_from_scenario()
        self.effective_start_m = (self.plant.state.x_m, self.plant.state.y_m)

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------
    def _prepare_graph(self) -> str:
        """Return the graph JSON, adding the isolated node when required.

        Scenario 11 needs a destination that is genuinely unreachable; injecting
        a node with no edges keeps that a property of the graph rather than a
        special case inside the navigation core.
        """
        needs_isolated = self.scenario.destination_node == UNREACHABLE_NODE_ID or any(
            event.action == "request_unreachable" for event in self.scenario.events
        )
        if not needs_isolated:
            return self.market_map.graph_json

        import json

        document = json.loads(self.market_map.graph_json)
        nodes = document.setdefault("nodes", [])
        if not any(int(node["id"]) == UNREACHABLE_NODE_ID for node in nodes):
            nodes.append(
                {
                    "id": UNREACHABLE_NODE_ID,
                    "x_m": self.market_map.width_m - 0.5,
                    "y_m": self.market_map.height_m - 0.5,
                    "type": "destination",
                    "notes": "Isolated node: exists in the map but has no edges (unreachable).",
                }
            )
        return json.dumps(document)

    def _prepare_navigation_config(self) -> str:
        """Apply per-scenario navigation overrides on top of navigation.json."""
        base_path = self.config.resolve(self.config.navigation_config_path)
        import json

        document = json.loads(base_path.read_text(encoding="utf-8")) if base_path.is_file() else {}
        for section, values in self.scenario.navigation_overrides.items():
            target = document.setdefault(section, {})
            if isinstance(values, dict):
                target.update(values)
        return json.dumps(document)

    def _configure_from_scenario(self) -> None:
        scenario = self.scenario
        self.plant.reset(scenario.start_x_m, scenario.start_y_m, scenario.start_theta_rad)
        self.uwb.config.quality_base = scenario.quality_base
        self.uwb.reset(now_ms=0)
        self.uwb.set_noise(scenario.noise_enabled, scenario.noise_std_m)
        self.uwb.set_dropout(scenario.dropout_enabled, scenario.dropout_probability)
        self.uwb.set_invalid(scenario.invalid_enabled, scenario.invalid_probability)
        self.uwb.set_low_quality(scenario.low_quality_enabled)
        self.uwb.set_frozen(scenario.frozen_enabled)

        # Scenario 11 requests an invalid destination on purpose: the engine must
        # not pre-validate it, otherwise the error path would never be exercised.
        self.core.request_destination(scenario.destination_node)
        self.state.true_pose = (scenario.start_x_m, scenario.start_y_m, scenario.start_theta_rad)

    # ------------------------------------------------------------------
    # Control surface
    # ------------------------------------------------------------------
    @property
    def true_pose(self) -> tuple[float, float, float]:
        state = self.plant.state
        return (state.x_m, state.y_m, state.theta_rad)

    @property
    def destination_position(self) -> tuple[float, float] | None:
        node = self.core.status().destination_node
        return self.market_map.node_position(node) if node >= 0 else None

    @property
    def metrics(self) -> Metrics:
        return self.metrics_accumulator.metrics

    @property
    def destination_candidates(self) -> list[int]:
        """Selectable destination nodes, in ascending id order.

        Used by the interactive destination selection: nodes typed as
        `destination` come first (they are the ones a customer would pick), then
        every remaining node so any graph point can be targeted.
        """
        preferred = [n.node_id for n in self.market_map.destination_nodes()]
        rest = [node_id for node_id in sorted(self.market_map.nodes) if node_id not in preferred]
        return preferred + rest

    def nearest_node_to(self, x_m: float, y_m: float) -> tuple[int, float] | None:
        """Nearest graph node to a map position, with its distance in meters.

        Used by interactive destination selection: the operator clicks anywhere
        and the closest graph node becomes the target, because the planner routes
        between nodes.
        """
        best_id = -1
        best_distance = -1.0
        for node in self.market_map.nodes.values():
            distance = math.hypot(node.x_m - x_m, node.y_m - y_m)
            if best_distance < 0.0 or distance < best_distance:
                best_distance = distance
                best_id = node.node_id
        if best_id < 0:
            return None
        return best_id, best_distance

    def pick_destination_at(self, x_m: float, y_m: float,
                            max_pick_distance_m: float | None = None) -> bool:
        """Set the destination to the graph node nearest to (x_m, y_m).

        `max_pick_distance_m` defaults to `path.max_graph_snap_distance_m` so a
        click far from the network is reported instead of silently targeting a
        node metres away.
        """
        found = self.nearest_node_to(x_m, y_m)
        if found is None:
            self.state.notes.append("destination pick failed: graph is empty")
            return False
        node_id, distance = found
        limit = (max_pick_distance_m if max_pick_distance_m is not None
                 else self.config.max_pick_distance_m)
        if distance > limit:
            self.state.notes.append(
                f"destination pick ignored: nearest node {node_id} is {distance:.2f} m away "
                f"(limit {limit:.2f} m)")
            return False
        accepted = self.set_destination(node_id)
        if accepted:
            self.state.notes.append(f"picked destination node {node_id} ({distance:.2f} m from click)")
        return accepted

    def cycle_destination(self, direction: int) -> bool:
        """Move the destination cursor through the selectable nodes.

        `direction` is +1 for the next candidate and -1 for the previous one.
        The candidate list wraps, so repeated presses visit every node.
        """
        candidates = self.destination_candidates
        if not candidates:
            return False
        current = self.core.status().destination_node
        if current in candidates:
            index = (candidates.index(current) + (1 if direction >= 0 else -1)) % len(candidates)
        else:
            index = 0
        return self.set_destination(candidates[index])

    def set_start_position(self, x_m: float, y_m: float, theta_rad: float | None = None) -> None:
        """Move the trolley to an arbitrary start position.

        The effective start is recorded so the ideal-path metric stays correct.
        """
        self.plant.reset(x_m, y_m,
                         self.scenario.start_theta_rad if theta_rad is None else theta_rad)
        self.effective_start_m = (x_m, y_m)

    def set_destination(self, node_id: int) -> bool:
        accepted = self.core.request_destination(node_id)
        self.state.notes.append(
            f"destination {'accepted' if accepted else 'rejected'}: node {node_id}"
        )
        return accepted

    def replan(self) -> None:
        self.core.request_replan()
        self.state.notes.append("manual replan requested")

    def emergency_stop(self) -> None:
        self.core.emergency_stop()
        self.plant.stop_immediately()
        self.state.notes.append("EMERGENCY STOP engaged")

    def clear_emergency_stop(self) -> None:
        self.core.clear_emergency_stop()
        self.state.notes.append("emergency stop cleared (start required)")

    def start_navigation(self) -> None:
        accepted = self.core.start()
        self.state.notes.append(
            f"start {'accepted' if accepted else 'rejected (no held destination)'}"
        )

    def cancel(self) -> None:
        self.core.cancel()
        self.state.notes.append("navigation cancelled")

    def toggle_pause(self) -> None:
        self.state.paused = not self.state.paused

    # ------------------------------------------------------------------
    # Fault injection (keyboard + scenario events)
    # ------------------------------------------------------------------
    def toggle_noise(self) -> None:
        enabled = not self.uwb.faults.noise_enabled
        self.uwb.set_noise(enabled)
        self.state.notes.append(f"UWB noise {'on' if enabled else 'off'}")

    def toggle_dropout(self) -> None:
        enabled = not self.uwb.faults.dropout_enabled
        self.uwb.set_dropout(enabled, probability=0.35 if enabled else 0.0, burst_s=1.0)
        self.state.notes.append(f"UWB dropout {'on' if enabled else 'off'}")

    def toggle_invalid(self) -> None:
        enabled = not self.uwb.faults.invalid_enabled
        self.uwb.set_invalid(enabled, probability=0.3 if enabled else 0.0)
        self.state.notes.append(f"UWB invalid samples {'on' if enabled else 'off'}")

    def toggle_low_quality(self) -> None:
        enabled = not self.uwb.faults.low_quality_enabled
        self.uwb.set_low_quality(enabled)
        self.state.notes.append(f"UWB low quality {'on' if enabled else 'off'}")

    def toggle_frozen(self) -> None:
        enabled = not self.uwb.faults.frozen_enabled
        self.uwb.set_frozen(enabled)
        self.state.notes.append(f"UWB frozen position {'on' if enabled else 'off'}")

    def trigger_position_jump(self, magnitude_m: float = 1.5) -> None:
        offset = self.uwb.trigger_jump(magnitude_m)
        self.state.notes.append(f"position jump {offset[0]:+.2f},{offset[1]:+.2f} m")

    def teleport_trolley(self, dx_m: float, dy_m: float) -> None:
        """Physically displace the plant (route-deviation drill)."""
        state = self.plant.state
        state.x_m += dx_m
        state.y_m += dy_m
        self.state.notes.append(f"trolley displaced {dx_m:+.2f},{dy_m:+.2f} m")

    # ------------------------------------------------------------------
    # Events
    # ------------------------------------------------------------------
    def _apply_event(self, event: TimedEvent) -> None:
        action = event.action
        if action == "dropout":
            self.uwb.trigger_dropout(float(event.value or 1.0), int(self.state.sim_time_s * 1000))
            self.state.notes.append(f"forced dropout for {float(event.value or 1.0):.1f} s")
        elif action == "low_quality_on":
            self.uwb.set_low_quality(True)
            self.state.notes.append("low quality fault enabled")
        elif action == "low_quality_off":
            self.uwb.set_low_quality(False)
            self.state.notes.append("low quality fault cleared")
        elif action == "invalid_on":
            self.uwb.set_invalid(True, probability=float(event.value or 0.3))
            self.state.notes.append("invalid samples enabled")
        elif action == "invalid_off":
            self.uwb.set_invalid(False)
            self.state.notes.append("invalid samples cleared")
        elif action == "frozen_on":
            self.uwb.set_frozen(True)
            self.state.notes.append("frozen position enabled")
        elif action == "frozen_off":
            self.uwb.set_frozen(False)
            self.state.notes.append("frozen position cleared")
        elif action == "position_jump":
            self.trigger_position_jump(float(event.value or 1.5))
        elif action == "clear_jump":
            self.uwb.clear_jump()
            self.state.notes.append("position jump cleared")
        elif action == "teleport":
            self.teleport_trolley(float(event.value or 1.0), float(event.value2 or 0.0))
        elif action == "change_destination":
            self.set_destination(int(event.value or 0))
        elif action == "request_unreachable":
            self.set_destination(int(event.value or UNREACHABLE_NODE_ID))
        elif action == "emergency_stop":
            self.emergency_stop()
        elif action == "clear_emergency_stop":
            self.clear_emergency_stop()
        elif action == "start_navigation":
            self.start_navigation()
        elif action == "replan":
            self.replan()
        else:
            self.state.notes.append(f"unknown scenario action ignored: {action}")

    def _process_events(self) -> None:
        for event in self.events:
            if not event.fired and self.state.sim_time_s >= event.at_s:
                event.fired = True
                self._apply_event(event)
        # Timed destination change declared directly on the scenario.
        if (
            self.scenario.destination_change_node is not None
            and self.scenario.destination_change_at_s is not None
            and self.state.sim_time_s >= self.scenario.destination_change_at_s
        ):
            self.set_destination(self.scenario.destination_change_node)
            self.scenario.destination_change_node = None

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    def step(self) -> None:
        """Advance the simulation by one fixed timestep."""
        if self.state.finished:
            return
        dt_s = self.config.simulation.dt_s
        self._process_events()

        # 1. Sensor model: only the measured position reaches navigation.
        now_ms = self._clock_offset_ms + int(round(self.state.sim_time_s * 1000.0))
        sample = self.uwb.poll(self.plant.state.x_m, self.plant.state.y_m, now_ms)
        if sample is not None:
            self._last_sample = sample
            self._samples_submitted += 1
            status = self.core.submit_position(
                sample.x_m, sample.y_m, sample.quality, sample.timestamp_ms, sample.valid
            )
            self.state.last_measurement_status = status
            if status != MeasurementStatus.OK:
                # A sample was actually delivered and the core rejected it: that
                # is a genuine invalid sample, unlike "no sample has arrived yet".
                self._samples_rejected += 1
            # Keep the live panel counters in step with the run.
            self.state.samples_received = self._samples_submitted
            self.state.samples_invalid = self._samples_rejected
            self.state.samples_valid = self._samples_submitted - self._samples_rejected
            self.state.measured = (sample.x_m, sample.y_m)
            self.state.uwb_fault = sample.fault
        elif self.uwb.is_dropout_active(now_ms):
            self.state.uwb_fault = "dropout"
        self.state.active_faults = self.uwb.active_fault_names(now_ms)

        # 2. One control cycle of the shared C++ state machine.
        self.core.update(now_ms)
        snapshot = self.core.status()

        # 3. Actuate the plant with the commands the core produced.
        if snapshot.state == NavState.EMERGENCY_STOP:
            # A software E-stop cuts power immediately (no coasting).
            self.plant.stop_immediately()
        else:
            self.plant.step(snapshot.left_motor, snapshot.right_motor, dt_s)

        # 4. Metrics and periodic logging.
        self.metrics_accumulator.observe(
            self.plant.state.x_m, self.plant.state.y_m, snapshot,
            self.destination_position, self.state.sim_time_s,
            measured_xy=self.state.measured,
        )
        self._telemetry_accumulator_s += dt_s
        if self._telemetry_accumulator_s >= self._telemetry_period_s:
            self._telemetry_accumulator_s = 0.0
            self._log(now_ms, snapshot)

        # 5. Time bookkeeping.
        self.state.sim_time_s += dt_s
        self.state.steps += 1
        self.state.true_pose = self.true_pose

        self._check_finished(snapshot)

    def _log(self, now_ms: int, snapshot) -> None:
        target = self.market_map.node_position(snapshot.active_waypoint_node)
        self.logger.record(
            timestamp_ms=now_ms,
            sim_time_s=self.state.sim_time_s,
            true_pose=self.true_pose,
            measured=self.state.measured,
            nav=snapshot,
            target=target,
            uwb_fault=self.state.uwb_fault,
        )

    def _check_finished(self, snapshot) -> None:
        if snapshot.state == NavState.ARRIVED:
            # Let the trolley settle before ending the run.
            if not self._settling:
                self._settling = True
                self._settle_started_s = self.state.sim_time_s
            settled = self.plant.state.linear_velocity_mps < self._settle_speed_eps
            if settled or (self.state.sim_time_s - self._settle_started_s) >= self._settle_grace_s:
                self._finish("destination reached")
            return
        elif snapshot.state == NavState.ERROR and self.scenario.expect_success is False:
            self._finish(f"navigation error: {snapshot.error}")
        elif self.state.sim_time_s >= self.scenario.timeout_s:
            self._finish("timeout")
        elif snapshot.state == NavState.ERROR:
            self._finish(f"navigation error: {snapshot.error}")

    def _finish(self, reason: str) -> None:
        if self.state.finished:
            return
        self.state.finished = True
        self.state.running = False
        self._finish_reason = reason
        self.state.notes.append(f"finished: {reason}")

    def run_to_completion(self, max_steps: int | None = None) -> Metrics:
        """Run until the scenario finishes (used by the headless sweep)."""
        self.state.running = True
        limit = max_steps or int(self.scenario.timeout_s / self.config.simulation.dt_s) + 10
        while not self.state.finished and self.state.steps < limit:
            self.step()
        return self.finalize()

    def finalize(self) -> Metrics:
        """Compute final metrics and export the CSV + JSON summary."""
        snapshot = self.core.status()
        metrics = self.metrics_accumulator.finalize(
            self.plant.state.x_m, self.plant.state.y_m, self.destination_position,
            self.state.sim_time_s, self.uwb.statistics.as_dict(),
        )
        # Ideal (straight-line) length of the route the trolley was told to
        # drive: from where it started, through each planned node.
        waypoints = [self.market_map.node_position(node) for node in snapshot.route]
        points = [self.effective_start_m]
        points += [p for p in waypoints if p is not None]
        metrics.ideal_path_distance_m = sum(
            math.dist(points[i], points[i + 1]) for i in range(len(points) - 1)
        )

        metrics.samples_received = self._samples_submitted
        metrics.samples_valid = self._samples_submitted - self._samples_rejected
        metrics.samples_invalid = self._samples_rejected
        metrics.samples_dropped = self.uwb.statistics.dropped

        metrics.success = metrics.success or snapshot.state == NavState.ARRIVED
        metrics.final_state = snapshot.state_name
        # Scenario 11 is a deliberate failure drill: record it as not successful
        # even if the FSM merely stopped in ERROR.
        if not self.scenario.expect_success:
            metrics.success = False
        # Classify against the configured acceptance bands.  Reported only: a
        # metric outside its target never fails the run by itself.
        # A terminal navigation state means the run played out; NAVIGATING or
        # IDLE here means it was stopped early.
        metrics.completed = snapshot.state in (NavState.ARRIVED, NavState.ERROR)
        metrics.classify_against_benchmarks(self.config.benchmarks)
        self.logger.export(
            metrics,
            extra={
                "description": self.scenario.description,
                "expectation": self.scenario.expectation,
                "start_pose": {
                    "x_m": self.scenario.start_x_m,
                    "y_m": self.scenario.start_y_m,
                    "theta_rad": self.scenario.start_theta_rad,
                },
                "random_seed": self.seed,
                "finish_reason": self._finish_reason,
                "uwb_config": {
                    "noise_std_m": self.scenario.noise_std_m,
                    "noise_enabled": self.scenario.noise_enabled,
                    "update_rate_hz": self.config.uwb.update_rate_hz,
                },
            },
        )
        return metrics

    def close(self) -> None:
        self.core.close()
        if self.stepper_pair is not None:
            self.stepper_pair.close()
            self.stepper_pair = None

    def __enter__(self) -> "SimulationEngine":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # ------------------------------------------------------------------
    # Rendering helpers (pure presentation, never fed back into navigation)
    # ------------------------------------------------------------------
    def planned_route(self) -> list[int]:
        return list(self.core.status().route)

    def route_polyline(self) -> list[tuple[float, float]]:
        return self.market_map.route_polyline(self.planned_route())

    def trajectory(self) -> list[tuple[float, float]]:
        """Travelled path, taken from the telemetry log (true poses)."""
        return [(record.true_x_m, record.true_y_m) for record in self.logger.records]

    def measured_trajectory(self) -> list[tuple[float, float]]:
        points = []
        for record in self.logger.records:
            if not math.isnan(record.measured_x_m):
                points.append((record.measured_x_m, record.measured_y_m))
        return points
