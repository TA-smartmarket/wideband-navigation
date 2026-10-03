"""Experiment logging: CSV telemetry, JSON summary and performance metrics.

Kept entirely outside the control path: the simulation loop appends a record
per step and the exporter writes files when the run ends.  Nothing in the
navigation core is aware of logging.
"""

from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

from .config import LoggingConfig, REPO_ROOT
from .native_core import NavState, NavSnapshot

#: Column order of the exported CSV (matches docs/simulator.md).
CSV_COLUMNS = [
    "timestamp_ms",
    "sim_time_s",
    "true_x_m",
    "true_y_m",
    "true_theta_rad",
    "measured_x_m",
    "measured_y_m",
    "nav_x_m",
    "nav_y_m",
    "estimated_heading_rad",
    "target_x_m",
    "target_y_m",
    "target_bearing_rad",
    "heading_error_rad",
    "distance_to_waypoint_m",
    "distance_to_destination_m",
    "cross_track_error_m",
    "position_error_m",
    "linear_velocity_mps",
    "angular_velocity_radps",
    "left_motor",
    "right_motor",
    "navigation_state",
    "active_waypoint_node",
    "waypoint_index",
    "waypoint_count",
    "destination_node",
    "position_quality",
    "position_valid",
    "uwb_fault",
    "replan_count",
    "position_loss_events",
    "invalid_sample_count",
]


@dataclass
class TelemetryRecord:
    """One row of the experiment log."""

    timestamp_ms: int
    sim_time_s: float
    true_x_m: float
    true_y_m: float
    true_theta_rad: float
    measured_x_m: float
    measured_y_m: float
    nav_x_m: float
    nav_y_m: float
    estimated_heading_rad: float
    target_x_m: float
    target_y_m: float
    target_bearing_rad: float
    heading_error_rad: float
    distance_to_waypoint_m: float
    distance_to_destination_m: float
    cross_track_error_m: float
    position_error_m: float
    linear_velocity_mps: float
    angular_velocity_radps: float
    left_motor: float
    right_motor: float
    navigation_state: str
    active_waypoint_node: int
    waypoint_index: int
    waypoint_count: int
    destination_node: int
    position_quality: float
    position_valid: bool
    uwb_fault: str
    replan_count: int
    position_loss_events: int
    invalid_sample_count: int

    def as_row(self) -> dict:
        row = {}
        for column in CSV_COLUMNS:
            value = getattr(self, column)
            if isinstance(value, bool):
                row[column] = "true" if value else "false"
            elif isinstance(value, float):
                row[column] = f"{value:.6f}"
            else:
                row[column] = value
        return row


@dataclass
class Metrics:
    """Aggregated experiment metrics (the values reported in the summary)."""

    planned_distance_m: float = 0.0

    # --- distance accounting (three independent, clearly labelled sources) ---
    #: MAIN travelled distance: accumulated ground-truth translation.  This is the
    #: only figure that represents how far the trolley actually moved, and it is
    #: what "Travelled Distance" reports.
    ground_truth_distance_m: float = 0.0
    #: Path length of the raw UWB measurement stream.  Always larger than the
    #: ground truth because each noisy sample adds a spurious displacement - it is
    #: a measure of sensor noise, NOT of trolley motion.
    uwb_measured_distance_m: float = 0.0
    #: Distance integrated from wheel rotation (odometry).  Pure spin-in-place
    #: contributes ~0 because the two wheels travel equal and opposite amounts.
    wheel_odometry_distance_m: float = 0.0
    #: Straight-line length of the route the trolley was told to drive.
    ideal_path_distance_m: float = 0.0
    #: ground_truth - planned (negative means the trolley beat the straight-line
    #: estimate, which happens when it cuts the corner inside the tolerance).
    excess_distance_m: float = 0.0
    #: planned / ground_truth * 100.  Around 100 % means no detours.
    path_efficiency_percent: float = 0.0

    completion_time_s: float = 0.0
    success: bool = False
    #: True when the run reached a terminal navigation state (ARRIVED or ERROR).
    #: A run stopped early by the operator (or by quitting the window) is *not*
    #: a navigation failure, and route-completion metrics such as path efficiency
    #: are meaningless for it.
    completed: bool = False
    final_state: str = ""
    destination_error_m: float = 0.0

    # --- position error, split by what is being compared --------------------
    #: RMSE of the UWB measurement against ground truth (sensor accuracy).
    uwb_rmse_m: float = 0.0
    #: RMSE of the position the navigation core used against ground truth.  Equal
    #: to the UWB figure when no navigation-side filter is enabled.
    nav_input_rmse_m: float = 0.0
    mean_position_error_m: float = 0.0
    max_position_error_m: float = 0.0

    mean_cross_track_error_m: float = 0.0
    max_cross_track_error_m: float = 0.0

    # --- heading tracking (NAVIGATING samples only) -------------------------
    mean_absolute_heading_error_deg: float = 0.0
    max_absolute_heading_error_deg: float = 0.0

    # --- control activity ---------------------------------------------------
    mean_absolute_angular_velocity: float = 0.0
    max_absolute_angular_velocity: float = 0.0
    #: Samples where either wheel command moved by more than
    #: `control.command_change_epsilon` since the previous sample.
    motor_command_change_count: int = 0
    #: Sign changes of the steering term (right - left).  This is the real
    #: oscillation indicator: a smooth ramp produces none, chatter produces many.
    steering_reversal_count: int = 0
    #: Samples where |omega| exceeded `control.large_steering_threshold_radps`.
    large_steering_correction_count: int = 0

    replan_count: int = 0
    position_loss_events: int = 0

    # --- sample accounting --------------------------------------------------
    samples_received: int = 0
    samples_valid: int = 0
    samples_invalid: int = 0
    samples_dropped: int = 0
    position_timeout_events: int = 0
    #: Rejections by the navigation core (its own counter, for cross-checking).
    core_invalid_sample_count: int = 0
    uwb_position_jumps: int = 0

    #: Per-metric PASS / WARNING / FAIL classification, filled by
    #: classify_against_benchmarks().  A failing band never aborts a run.
    benchmark_results: dict = field(default_factory=dict)
    #: Worst classification across all benchmarks ("PASS" / "WARNING" / "FAIL").
    benchmark_overall: str = "PASS"

    steps: int = 0
    #: Wall-clock cost of the whole Dijkstra call, measured by the C++ core.
    last_plan_time_us: int = 0
    route: list[int] = field(default_factory=list)
    start_node: int = -1
    destination_node: int = -1

    def as_dict(self) -> dict:
        return {
            "success": self.success,
            "completed": self.completed,
            "final_state": self.final_state,
            "planned_distance_m": round(self.planned_distance_m, 4),
            "travelled_distance_m": round(self.ground_truth_distance_m, 4),
            "ground_truth_distance_m": round(self.ground_truth_distance_m, 4),
            "uwb_measured_distance_m": round(self.uwb_measured_distance_m, 4),
            "wheel_odometry_distance_m": round(self.wheel_odometry_distance_m, 4),
            "ideal_path_distance_m": round(self.ideal_path_distance_m, 4),
            "excess_distance_m": round(self.excess_distance_m, 4),
            "path_efficiency_percent": round(self.path_efficiency_percent, 1),
            "completion_time_s": round(self.completion_time_s, 3),
            "destination_error_m": round(self.destination_error_m, 4),
            "uwb_rmse_m": round(self.uwb_rmse_m, 4),
            "nav_input_rmse_m": round(self.nav_input_rmse_m, 4),
            "mean_position_error_m": round(self.mean_position_error_m, 4),
            "max_position_error_m": round(self.max_position_error_m, 4),
            "mean_absolute_heading_error_deg": round(self.mean_absolute_heading_error_deg, 2),
            "max_absolute_heading_error_deg": round(self.max_absolute_heading_error_deg, 2),
            "mean_absolute_angular_velocity_radps": round(self.mean_absolute_angular_velocity, 4),
            "max_absolute_angular_velocity_radps": round(self.max_absolute_angular_velocity, 4),
            "motor_command_change_count": self.motor_command_change_count,
            "steering_reversal_count": self.steering_reversal_count,
            "large_steering_correction_count": self.large_steering_correction_count,
            "mean_cross_track_error_m": round(self.mean_cross_track_error_m, 4),
            "max_cross_track_error_m": round(self.max_cross_track_error_m, 4),
            "replan_count": self.replan_count,
            "position_loss_events": self.position_loss_events,
            "position_timeout_events": self.position_timeout_events,
            "samples_received": self.samples_received,
            "samples_valid": self.samples_valid,
            "samples_invalid": self.samples_invalid,
            "samples_dropped": self.samples_dropped,
            "core_invalid_sample_count": self.core_invalid_sample_count,
            "uwb_position_jumps": self.uwb_position_jumps,
            "simulation_steps": self.steps,
            "dijkstra_time_us": self.last_plan_time_us,
            "route": self.route,
            "start_node": self.start_node,
            "destination_node": self.destination_node,
            "benchmarks": self.benchmark_results,
            "benchmark_overall": self.benchmark_overall,
        }

    def classify_against_benchmarks(self, benchmarks: dict) -> dict:
        """Evaluate this run against acceptance bands.

        Returns a mapping of metric -> {value, pass, warn, direction, status}.
        A metric that is not present in the run is skipped, and a band breach is
        reported rather than treated as an error: experimental metrics are allowed
        to fall outside the target.
        """
        results: dict = {}
        worst = "PASS"
        rank = {"PASS": 0, "WARNING": 1, "FAIL": 2}
        # Metrics that describe a finished route cannot be judged on a run that
        # was stopped early.
        route_metrics = {"path_efficiency_percent", "destination_error_m",
                         "mean_cross_track_error_m", "max_cross_track_error_m",
                         "mean_absolute_heading_error_deg"}
        for name, benchmark in benchmarks.items():
            if not hasattr(self, name):
                continue
            if not self.completed and name in route_metrics:
                continue
            value = float(getattr(self, name))
            status = benchmark.classify(value)
            results[name] = {
                "value": round(value, 4),
                "pass": benchmark.pass_value,
                "warn": benchmark.warn_value,
                "direction": benchmark.direction,
                "status": status,
            }
            if rank[status] > rank[worst]:
                worst = status
        self.benchmark_results = results
        self.benchmark_overall = worst if self.completed else "INCOMPLETE"
        return results

    def benchmark_report(self) -> str:
        """One line per benchmark, aligned for the console."""
        if not self.benchmark_results:
            return ""
        if not self.completed:
            return ("Result validation: run did not finish, so route metrics were not "
                    "evaluated")
        lines = ["Result validation:"]
        for name, entry in self.benchmark_results.items():
            band = (f"<= {entry['pass']}" if entry["direction"] == "max"
                    else f">= {entry['pass']}")
            lines.append(
                f"  {name:34s} {entry['value']:10.4f}  target {band:10s} "
                f"{entry['status']}"
            )
        lines.append(f"  {'OVERALL':34s} {'':10s}  {'':17s} {self.benchmark_overall}")
        return "\n".join(lines)

    def as_report(self) -> str:
        """Human readable block printed after each run."""
        if self.success:
            verdict = "SUCCESS"
        elif not self.completed:
            # Stopped early (operator quit, window closed): not a navigation fault.
            verdict = "INTERRUPTED (run not finished)"
        else:
            verdict = "FAILED"
        route = " -> ".join(str(n) for n in self.route) if self.route else "n/a"
        return "\n".join(
            [
                f"Navigation Result: {verdict}",
                f"Final State: {self.final_state}",
                f"Route: {route}",
                f"Planned Distance: {self.planned_distance_m:.2f} m",
                f"Travelled Distance: {self.ground_truth_distance_m:.2f} m (ground truth)",
                f"Excess Distance: {self.excess_distance_m:+.2f} m",
                (f"Path Efficiency: {self.path_efficiency_percent:.1f} %"
                 if self.completed else
                 "Path Efficiency: n/a (route not completed)"),
                f"  (UWB-stream path {self.uwb_measured_distance_m:.2f} m, "
                f"wheel odometry {self.wheel_odometry_distance_m:.2f} m)",
                f"Completion Time: {self.completion_time_s:.1f} s",
                f"Position RMSE: {self.uwb_rmse_m:.4f} m (UWB), "
                f"{self.nav_input_rmse_m:.4f} m (navigation input)",
                f"Mean Position Error: {self.mean_position_error_m:.4f} m",
                f"Max Position Error: {self.max_position_error_m:.4f} m",
                f"Mean Cross Track Error: {self.mean_cross_track_error_m:.3f} m",
                f"Maximum Cross Track Error: {self.max_cross_track_error_m:.3f} m",
                f"Heading Error: mean {self.mean_absolute_heading_error_deg:.2f} deg, "
                f"max {self.max_absolute_heading_error_deg:.2f} deg (NAVIGATING only)",
                f"Angular Velocity: mean {self.mean_absolute_angular_velocity:.3f}, "
                f"max {self.max_absolute_angular_velocity:.3f} rad/s",
                f"Control Activity: {self.motor_command_change_count} command changes, "
                f"{self.steering_reversal_count} steering reversals, "
                f"{self.large_steering_correction_count} large steering corrections",
                f"Destination Error: {self.destination_error_m:.3f} m",
                f"Replans: {self.replan_count}",
                f"Position Loss Events: {self.position_timeout_events}",
                f"Samples: {self.samples_received} received, {self.samples_valid} valid, "
                f"{self.samples_invalid} invalid, {self.samples_dropped} dropped",
                f"Dijkstra Time: {self.last_plan_time_us} us",
            ]
        )


class MetricsAccumulator:
    """Incremental metric accumulation (no full-record retention required)."""

    def __init__(self) -> None:
        self.metrics = Metrics()
        self._position_error_sum = 0.0
        self._position_error_sq_sum = 0.0
        self._cross_track_sum = 0.0
        self._cross_track_samples = 0
        self._position_error_samples = 0
        self._previous_true: tuple[float, float] | None = None
        self._previous_measured: tuple[float, float] | None = None
        self._previous_odometry: tuple[float, float] | None = None
        self._uwb_error_sq_sum = 0.0
        self._uwb_error_samples = 0
        self._nav_error_sq_sum = 0.0
        self._heading_error_deg_sum = 0.0
        self._heading_error_samples = 0
        self._angular_sum = 0.0
        self._previous_motor: tuple[float, float] | None = None
        self._previous_steering: float | None = None
        #: Supplies (received, valid, invalid, dropped) so the sample accounting is
        #: live rather than only available at finalize().
        self.sample_counter_provider = None
        self._ideal_path_m = 0.0
        #: Minimum wheel-command change counted as an activity step.
        self.command_change_epsilon = 0.01
        #: Set by the engine so odometry and timeout events can be folded in.
        self.odometry_distance_provider = None
        self.large_steering_threshold_radps = 0.6

    def reset(self) -> None:
        self.__init__()

    def observe(self,
                true_x_m: float,
                true_y_m: float,
                nav: NavSnapshot,
                true_destination: tuple[float, float] | None,
                sim_time_s: float,
                measured_xy: tuple[float, float] | None = None) -> None:
        """Fold one simulation step into the aggregate."""
        metrics = self.metrics
        metrics.steps += 1

        # Ground-truth travelled distance: pure translational motion.  A
        # rotation in place contributes ~0 because the reference point does not
        # translate.
        if self._previous_true is not None:
            metrics.ground_truth_distance_m += math.dist(
                (true_x_m, true_y_m), self._previous_true
            )
        self._previous_true = (true_x_m, true_y_m)

        # UWB measurement stream length (sensor-noise indicator, not motion).
        measured = measured_xy
        if measured is not None:
            if self._previous_measured is not None:
                metrics.uwb_measured_distance_m += math.dist(measured, self._previous_measured)
            self._previous_measured = measured

        # Navigation-input error, compared against ground truth *of the same
        # sample* (same row, therefore same timestamp).  Samples taken before the
        # first position is accepted are skipped: the navigation position is still
        # its (0,0) default then, which would inject a bogus multi-metre error.
        position_acquired = nav.position_age_ms != 0xFFFFFFFFFFFFFFFF
        if position_acquired:
            error = math.hypot(nav.x_m - true_x_m, nav.y_m - true_y_m)
            self._position_error_sum += error
            self._position_error_sq_sum += error * error
            self._nav_error_sq_sum += error * error
            self._position_error_samples += 1
            metrics.max_position_error_m = max(metrics.max_position_error_m, error)

        # UWB sensor error against the same ground-truth sample.
        if measured is not None:
            uwb_error = math.dist(measured, (true_x_m, true_y_m))
            self._uwb_error_sq_sum += uwb_error * uwb_error
            self._uwb_error_samples += 1

        # Tracking metrics are only meaningful while the controller is acting, so
        # IDLE / ARRIVED / ERROR / E-stop samples are excluded.
        if nav.state == NavState.NAVIGATING:
            self._cross_track_sum += abs(nav.cross_track_error_m)
            self._cross_track_samples += 1
            metrics.max_cross_track_error_m = max(
                metrics.max_cross_track_error_m, abs(nav.cross_track_error_m)
            )

            heading_deg = abs(math.degrees(nav.heading_error_rad))
            self._heading_error_deg_sum += heading_deg
            self._heading_error_samples += 1
            metrics.max_absolute_heading_error_deg = max(
                metrics.max_absolute_heading_error_deg, heading_deg
            )

            omega = abs(nav.angular_velocity_radps)
            self._angular_sum += omega
            metrics.max_absolute_angular_velocity = max(
                metrics.max_absolute_angular_velocity, omega
            )
            if omega > self.large_steering_threshold_radps:
                metrics.large_steering_correction_count += 1

            # Command changes beyond a small epsilon, plus genuine steering
            # reversals.  A pure slew-limited ramp changes the command slightly
            # every cycle, so an epsilon-free count would only measure the ramp.
            current_motor = (nav.left_motor, nav.right_motor)
            if self._previous_motor is not None:
                if (abs(current_motor[0] - self._previous_motor[0]) > self.command_change_epsilon or
                        abs(current_motor[1] - self._previous_motor[1]) > self.command_change_epsilon):
                    metrics.motor_command_change_count += 1
            self._previous_motor = current_motor

            steering = nav.right_motor - nav.left_motor
            if self._previous_steering is not None:
                if steering * self._previous_steering < 0.0:
                    metrics.steering_reversal_count += 1
            self._previous_steering = steering
        else:
            self._previous_motor = None
            self._previous_steering = None

        metrics.planned_distance_m = nav.planned_distance_m
        metrics.core_invalid_sample_count = nav.invalid_sample_count
        metrics.position_timeout_events = nav.position_loss_events

        # Keep the derived figures current so the live panel, the CSV and the final
        # summary never disagree.
        self._refresh_derived(metrics)
        metrics.replan_count = nav.replan_count
        metrics.position_loss_events = nav.position_loss_events
        metrics.invalid_sample_count = nav.invalid_sample_count
        metrics.last_plan_time_us = max(metrics.last_plan_time_us, nav.plan_time_us)
        metrics.route = list(nav.route)
        metrics.start_node = nav.start_node
        metrics.destination_node = nav.destination_node
        metrics.final_state = nav.state_name
        # A terminal navigation state means the run played out to the end.  This
        # is tracked every cycle (not only in finalize()) so the headless sweep,
        # which prints metrics straight from run_to_completion(), agrees with the
        # interactive run about whether the route was actually completed.
        metrics.completed = nav.state in (NavState.ARRIVED, NavState.ERROR)

        if nav.state in (NavState.ARRIVED,):
            metrics.success = True
            metrics.completion_time_s = sim_time_s
        if nav.state == NavState.ARRIVED and true_destination is not None:
            metrics.destination_error_m = math.hypot(
                true_x_m - true_destination[0], true_y_m - true_destination[1]
            )

        if self._position_error_samples > 0:
            metrics.mean_position_error_m = self._position_error_sum / self._position_error_samples
            metrics.nav_input_rmse_m = math.sqrt(
                self._nav_error_sq_sum / self._position_error_samples
            )
        if self._uwb_error_samples > 0:
            metrics.uwb_rmse_m = math.sqrt(self._uwb_error_sq_sum / self._uwb_error_samples)
        if self._cross_track_samples > 0:
            metrics.mean_cross_track_error_m = self._cross_track_sum / self._cross_track_samples
        if self._heading_error_samples > 0:
            metrics.mean_absolute_heading_error_deg = (
                self._heading_error_deg_sum / self._heading_error_samples
            )
            metrics.mean_absolute_angular_velocity = self._angular_sum / self._heading_error_samples

    def _refresh_derived(self, metrics: Metrics) -> None:
        """Recompute the values derived from other metrics.

        Called every cycle so a partially completed run already reports meaningful
        efficiency, odometry and sample counts instead of zeros.
        """
        if self.odometry_distance_provider is not None:
            metrics.wheel_odometry_distance_m = float(self.odometry_distance_provider())
        if self.sample_counter_provider is not None:
            received, valid, invalid, dropped = self.sample_counter_provider()
            metrics.samples_received = int(received)
            metrics.samples_valid = int(valid)
            metrics.samples_invalid = int(invalid)
            metrics.samples_dropped = int(dropped)
        metrics.excess_distance_m = (metrics.ground_truth_distance_m -
                                     metrics.planned_distance_m)
        # Path efficiency compares the *whole* plan against the *whole* travelled
        # distance, so it is only meaningful once the route was completed.  On an
        # interrupted run it would report a nonsense figure above 100 %.
        if (metrics.completed and metrics.ground_truth_distance_m > 1e-6 and
                metrics.planned_distance_m > 1e-6):
            metrics.path_efficiency_percent = (
                100.0 * metrics.planned_distance_m / metrics.ground_truth_distance_m
            )

    def finalize(self,
                 true_x_m: float,
                 true_y_m: float,
                 true_destination: tuple[float, float] | None,
                 sim_time_s: float,
                 uwb_statistics: dict | None = None) -> Metrics:
        metrics = self.metrics
        if true_destination is not None:
            metrics.destination_error_m = math.hypot(
                true_x_m - true_destination[0], true_y_m - true_destination[1]
            )
        if not metrics.completion_time_s:
            metrics.completion_time_s = sim_time_s
        # Odometry, sample accounting and the derived distances were already kept
        # current by _refresh_derived(); recompute once more against the final
        # totals so the summary is exact.
        self._refresh_derived(metrics)

        # Only fill the sensor-side counters that the engine does not own; the
        # received/valid/invalid triple is set by the engine, which is the only
        # component that knows what was actually delivered to navigation.
        if uwb_statistics:
            metrics.uwb_position_jumps = int(uwb_statistics.get("jumps", 0))
        return metrics


class ExperimentLogger:
    """Writes ``logs/*.csv`` and ``results/*_summary.json`` for one run."""

    def __init__(self, config: LoggingConfig, scenario_name: str, run_index: int = 1):
        self.config = config
        self.scenario_name = scenario_name
        self.run_index = run_index
        self.records: list[TelemetryRecord] = []
        self.started_at = datetime.now()
        self._stamp = self.started_at.strftime("%Y-%m-%d_%H-%M-%S")

    # -- record collection -------------------------------------------------
    def record(self,
               timestamp_ms: int,
               sim_time_s: float,
               true_pose: tuple[float, float, float],
               measured: tuple[float, float] | None,
               nav: NavSnapshot,
               target: tuple[float, float] | None,
               uwb_fault: str = "") -> TelemetryRecord:
        entry = TelemetryRecord(
            timestamp_ms=int(timestamp_ms),
            sim_time_s=float(sim_time_s),
            true_x_m=true_pose[0],
            true_y_m=true_pose[1],
            true_theta_rad=true_pose[2],
            measured_x_m=measured[0] if measured else float("nan"),
            measured_y_m=measured[1] if measured else float("nan"),
            nav_x_m=nav.x_m,
            nav_y_m=nav.y_m,
            estimated_heading_rad=nav.heading_rad if nav.heading_valid else float("nan"),
            target_x_m=target[0] if target else float("nan"),
            target_y_m=target[1] if target else float("nan"),
            target_bearing_rad=nav.target_bearing_rad,
            heading_error_rad=nav.heading_error_rad,
            distance_to_waypoint_m=nav.distance_to_waypoint_m,
            distance_to_destination_m=nav.distance_to_destination_m,
            cross_track_error_m=nav.cross_track_error_m,
            position_error_m=math.hypot(nav.x_m - true_pose[0], nav.y_m - true_pose[1]),
            linear_velocity_mps=nav.linear_velocity_mps,
            angular_velocity_radps=nav.angular_velocity_radps,
            left_motor=nav.left_motor,
            right_motor=nav.right_motor,
            navigation_state=nav.state_name,
            active_waypoint_node=nav.active_waypoint_node,
            waypoint_index=nav.waypoint_index,
            waypoint_count=nav.waypoint_count,
            destination_node=nav.destination_node,
            position_quality=nav.position_quality,
            position_valid=nav.position_valid,
            uwb_fault=uwb_fault,
            replan_count=nav.replan_count,
            position_loss_events=nav.position_loss_events,
            invalid_sample_count=nav.invalid_sample_count,
        )
        self.records.append(entry)
        return entry

    # -- export ------------------------------------------------------------
    def _directory(self, relative: str) -> Path:
        path = Path(relative)
        directory = path if path.is_absolute() else REPO_ROOT / path
        directory.mkdir(parents=True, exist_ok=True)
        return directory

    def csv_path(self) -> Path:
        return self._directory(self.config.directory) / (
            f"{self.scenario_name}_run{self.run_index:03d}_{self._stamp}.csv"
        )

    def summary_path(self) -> Path:
        return self._directory(self.config.results_directory) / (
            f"{self.scenario_name}_run{self.run_index:03d}_{self._stamp}_summary.json"
        )

    def export(self, metrics: Metrics, extra: dict | None = None) -> tuple[Path | None, Path | None]:
        """Write the CSV and the JSON summary.  Returns the written paths."""
        if not self.config.enabled:
            return None, None

        csv_file: Path | None = None
        summary_file: Path | None = None

        if self.config.export_csv and self.records:
            csv_file = self.csv_path()
            with csv_file.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=CSV_COLUMNS)
                writer.writeheader()
                for entry in self.records:
                    writer.writerow(entry.as_row())

        if self.config.export_summary:
            summary_file = self.summary_path()
            payload = {
                "scenario": self.scenario_name,
                "run_index": self.run_index,
                "generated_at": self.started_at.isoformat(timespec="seconds"),
                **metrics.as_dict(),
            }
            if extra:
                payload.update(extra)
            with summary_file.open("w", encoding="utf-8") as handle:
                json.dump(payload, handle, indent=2)
                handle.write("\n")

        return csv_file, summary_file
