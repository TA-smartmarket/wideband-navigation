# Data Contracts

Three JSON documents define every interface in this project: the **position**
sample the navigation system consumes, the **telemetry** it publishes, and the
**configuration** files it loads. Each is validated at runtime, and each is
covered by tests.

## 1. Position sample (input)

The UWB/EKF positioning subsystem publishes processed positions. Raw ranging
values are never consumed by navigation.

```json
{
  "schema_version": 1,
  "trolley_id": "TROLLEY_01",
  "frame_id": "smart_market_map",
  "timestamp_ms": 1000,
  "position": { "x_m": 1.25, "y_m": 2.70 },
  "quality": 0.95,
  "valid": true
}
```

| Field | Type | Required | Meaning |
|---|---|---|---|
| `schema_version` | integer | yes | must equal `1` |
| `trolley_id` | string | yes | must match the configured trolley |
| `frame_id` | string | yes | must be `smart_market_map` |
| `timestamp_ms` | integer | yes | monotonic milliseconds, non-zero |
| `position.x_m` | number | yes | meters, `+X` right |
| `position.y_m` | number | yes | meters, `+Y` up |
| `quality` | number | yes | $[0, 1]$ confidence |
| `valid` | boolean | yes | `false` = unusable measurement |

### Validation rules

Evaluated in this order; the first failure rejects the sample
(`nav::validateMeasurement`). The `MeasurementStatus` code is what the firmware
and simulator report.

| Order | Condition | Status |
|---|---|---|
| 1 | `schema_version != 1` | `BAD_SCHEMA_VERSION` |
| 2 | `frame_id != "smart_market_map"` | `WRONG_FRAME_ID` |
| 3 | `trolley_id` empty | `WRONG_TROLLEY_ID` |
| 4 | `x_m` or `y_m` NaN / infinite | `NON_FINITE_COORDINATE` |
| 5 | `valid == false` | `INVALID_FLAG` |
| 6 | `quality < position.minimum_quality` (default 0.60) | `LOW_QUALITY` |
| 7 | outside the map rectangle by more than `map_margin_m` | `OUT_OF_MAP_BOUNDS` |
| 8 | timestamp zero, or older than `stale_sample_ms` | `STALE_TIMESTAMP` |

Rejected samples increment `invalid_sample_count`; they do **not** by themselves
change the navigation state. If rejections continue, the position ages out and
`POSITION_LOST` stops the trolley after `position.timeout_ms`.

### Wire format

One JSON object per line, `\n` terminated, UTF-8. The console/Serial parser also
accepts an optional `POS:` prefix, so commands and positions can share one link:

```text
POS:{"schema_version":1,"trolley_id":"TROLLEY_01","frame_id":"smart_market_map", ... }
CMD:destination 11
```

The parser is a bounded, allocation-free recursive-descent reader
(`navigation_core/src/json_parser.cpp`) with a 320-byte line buffer. A line split
across transport reads is reassembled; a line that overflows the buffer is dropped
rather than corrupting the next parse. Up to four reassembled samples are buffered
in a FIFO, so a burst is not lost.

## 2. Telemetry (output)

Published at `control.telemetry_rate_hz` (default 5 Hz) as a single JSON line:

```json
{
  "schema_version": 1,
  "trolley_id": "TROLLEY_01",
  "frame_id": "smart_market_map",
  "timestamp_ms": 5000,
  "navigation": {
    "state": "NAVIGATING",
    "error": "none",
    "destination_node": 11,
    "active_waypoint_node": 7,
    "waypoint_index": 2,
    "waypoint_count": 5,
    "distance_to_waypoint_m": 0.72,
    "distance_to_destination_m": 4.15,
    "cross_track_error_m": 0.11,
    "planned_distance_m": 10.0,
    "travelled_distance_m": 3.42,
    "heading_error_rad": 0.05,
    "remaining_distance_m": 4.15,
    "replan_count": 0,
    "position_loss_events": 0,
    "invalid_sample_count": 0,
    "plan_time_us": 12,
    "route_length": 5
  },
  "pose": {
    "x_m": 2.55,
    "y_m": 3.20,
    "heading_rad": 1.12,
    "heading_valid": true
  },
  "control": {
    "linear_velocity_mps": 0.25,
    "angular_velocity_radps": 0.42,
    "left_motor": 0.31,
    "right_motor": 0.64,
    "rotate_in_place": false
  },
  "position_quality": 0.92,
  "position_valid": true,
  "position_fresh": true
}
```

`state` is one of `BOOT`, `IDLE`, `WAITING_FOR_POSITION`, `READY`, `PLANNING`,
`NAVIGATING`, `REPLANNING`, `POSITION_LOST`, `ARRIVED`, `ERROR`,
`EMERGENCY_STOP`. `error` is one of `none`, `invalid position`, `position
timeout`, `invalid destination`, `start node not found`, `route not found`,
`graph invalid`, `config invalid`, `not ready`, `cancelled`,
`emergency stop active`.

Non-finite values are never emitted: every field passes through
`sanitizeFinite()`. The line is produced by `nav::statusToJson()` and the
`TelemetryPublisher` is deliberately separate from the control loop.

### Human readable status

The `status` console command prints the same information as text:

```text
Trolley: TROLLEY_01
State: NAVIGATING
Error: none
Position: (3.21, 4.08)
Heading: 92.5 deg
Start Node: 2
Destination Node: 11
Current Waypoint: 7
Waypoint: 3/5
Distance to Waypoint: 0.48 m
Distance to Destination: 4.15 m
Cross Track Error: 0.11 m
Planned Distance: 10.00 m
Travelled Distance: 3.42 m
UWB Quality: 0.94
Position Valid: yes
Left Motor: 0.43
Right Motor: 0.56
Replans: 0
Position Loss Events: 0
```

## 3. Command contract (input)

Operator or host commands, one per line. An optional `CMD:` prefix is accepted.

| Command | Effect |
|---|---|
| `help` | print the command list |
| `status` | human readable navigation status |
| `graph` | graph summary (node/edge counts) |
| `route` | planned route and distance |
| `position` | latest accepted position |
| `destination <n>` (or `dest <n>`) | request destination node `n` |
| `start` | start/resume navigation to the held destination |
| `stop` | stop and keep the destination |
| `cancel` | cancel navigation and clear the destination |
| `estop` | software emergency stop |
| `clear_estop` | clear the E-stop (does not resume motion) |
| `replan` | force a replan on the next cycle |
| `debug on` / `debug off` | verbose logging |

JSON position lines are never mistaken for commands (they start with `{`, or are
explicitly prefixed `POS:`). A malformed command argument is reported, not
guessed: `destination` without a valid integer leaves the argument at `-1`, which
the caller rejects.

The equivalent JSON form for a backend is:

```json
{ "command": "navigate",           "trolley_id": "TROLLEY_01", "destination_node": 11 }
{ "command": "stop",               "trolley_id": "TROLLEY_01" }
{ "command": "cancel_navigation",  "trolley_id": "TROLLEY_01" }
```

## 4. Configuration documents

### `config/map.json`

```json
{
  "map_id": "SMART_MARKET_MAIN",
  "map_version": 1,
  "frame_id": "smart_market_map",
  "width_m": 12.0,
  "height_m": 8.0,
  "origin_x_m": 0.0,
  "origin_y_m": 0.0
}
```

`map_version` must match `graph.json → map.map_version`; a mismatch is rejected so
a stale graph cannot be navigated.

### `config/graph.json`

```json
{
  "map": { "map_id": "SMART_MARKET_MAIN", "map_version": 1, "frame_id": "smart_market_map",
           "width_m": 12.0, "height_m": 8.0 },
  "nodes": [
    { "id": 1, "x_m": 1.5, "y_m": 2.0, "type": "entry", "name": "masuk" }
  ],
  "edges": [
    { "from": 1, "to": 2, "bidirectional": true }
  ]
}
```

* `type` is one of `intersection`, `aisle`, `destination`, `parking`, `entry`,
  `exit`, `checkout`.
* `name` is optional and gives the node a human handle (`produk-susu`). Names
  must be unique (compared case-insensitively) and are resolved to node ids by
  the simulator, so the navigation core still works purely with integer ids.
  `python simulator/main.py --list-nodes` prints the table; `--destination
  produk-susu` selects by name. The name is also what the map label and the
  status panel show.
* `weight_m` is optional; when absent the Euclidean distance is used.
* `bidirectional` defaults to `true`.
* Validation rejects duplicate ids, unknown endpoints, self loops, negative
  weights, non-finite coordinates and an empty node list.

### `config/navigation.json`

Two keys select and shape the actuator:

```json
{
  "drive_kind": "stepper",
  "stepper": {
    "common": {
      "steps_per_revolution": 200,
      "microsteps": 16,
      "max_step_rate_hz": 20000,
      "min_step_rate_hz": 2,
      "max_step_accel_hz_per_s": 20000
    },
    "left":  { "invert_direction": false },
    "right": { "invert_direction": false },
    "enable_odometry": true
  }
}
```

* `drive_kind` is `"stepper"` (NEMA 17 + A4988, step/dir) or `"pwm_h_bridge"`
  (DC motor, duty cycle). The navigation algorithms are identical either way.
* `microsteps` must match the A4988 MS1/MS2/MS3 wiring (1, 2, 4, 8, 16).
* `max_step_rate_hz` must not exceed the firmware STEP timer base
  (`kStepperTimerBaseHz`, 20 kHz), or the pulse train cannot be generated.
* `invert_direction` flips the DIR level for a motor mounted mirrored.

See [README §11](../README.md#11-configuration-parameters) for the rest of the
parameter table. Every value is range-checked; a failure yields `CONFIG_INVALID` and the
motors stay stopped. Cross-field rules include
`max_cross_track_error_m <= replan_cross_track_error_m`,
`min_linear_speed_mps <= max_linear_speed_mps`, and
`max_wheel_speed_mps >= max(max_linear_speed_mps, 0.5 * max_angular_speed_radps *
wheel_base_m)` (the two twists the speed policy can actually request, since it
never asks for full speed and full turn simultaneously).

### `config/simulator.json`

Describes the experiment environment only: window geometry, physics, the UWB
sensor model, logging paths and render options. It never contains navigation
parameters.

## 5. Experiment artefacts

### CSV telemetry (`logs/`)

One row per logged cycle. Columns:

```text
timestamp_ms, sim_time_s, true_x_m, true_y_m, true_theta_rad,
measured_x_m, measured_y_m, nav_x_m, nav_y_m, estimated_heading_rad,
target_x_m, target_y_m, target_bearing_rad, heading_error_rad,
distance_to_waypoint_m, distance_to_destination_m, cross_track_error_m,
position_error_m, linear_velocity_mps, angular_velocity_radps,
left_motor, right_motor, navigation_state, active_waypoint_node,
waypoint_index, waypoint_count, destination_node, position_quality,
position_valid, uwb_fault, replan_count, position_loss_events,
invalid_sample_count
```

`true_*` is ground truth (evaluation only), `measured_*` is the UWB output,
`nav_*` is what the core used.

`completed` distinguishes a finished route from a run the operator stopped early
(closing the window, quitting mid-route). Route-completion metrics - path
efficiency, destination error, cross-track error, heading error - are only
meaningful when `completed` is true; on an interrupted run they are left
unevaluated and `benchmark_overall` is `INCOMPLETE` rather than `FAIL`, so
stopping the simulator never looks like a navigation fault.

### JSON summary (`results/`)

Actual output of `scenario_04_moderate_uwb_noise` (sigma = 0.10 m), abridged.
Note `uwb_measured_distance_m` is 34.89 m: that is the length of the *noisy
measurement stream*, roughly three times the true 11.66 m path, which is exactly
why travelled distance is taken from ground truth.

```json
{
  "scenario": "scenario_04_moderate_uwb_noise",
  "success": true,
  "completed": true,
  "final_state": "ARRIVED",
  "planned_distance_m": 11.3045,
  "travelled_distance_m": 11.6641,
  "ground_truth_distance_m": 11.6641,
  "uwb_measured_distance_m": 34.8887,
  "wheel_odometry_distance_m": 11.6641,
  "ideal_path_distance_m": 11.3038,
  "excess_distance_m": 0.3596,
  "path_efficiency_percent": 96.9,
  "completion_time_s": 30.16,
  "uwb_rmse_m": 0.1152,
  "nav_input_rmse_m": 0.1152,
  "mean_absolute_heading_error_deg": 12.74,
  "max_absolute_heading_error_deg": 69.71,
  "mean_absolute_angular_velocity_radps": 0.2621,
  "max_absolute_angular_velocity_radps": 1.3388,
  "motor_command_change_count": 759,
  "steering_reversal_count": 46,
  "large_steering_correction_count": 150,
  "mean_cross_track_error_m": 0.1572,
  "max_cross_track_error_m": 0.3924,
  "destination_error_m": 0.0594,
  "replan_count": 0,
  "samples_received": 302,
  "samples_valid": 302,
  "samples_invalid": 0,
  "samples_dropped": 0,
  "position_timeout_events": 0,
  "benchmarks": {
    "destination_error_m": {"value": 0.0594, "pass": 0.2, "warn": 0.3, "direction": "max",
                            "status": "PASS"}
  },
  "benchmark_overall": "PASS",
  "mean_position_error_m": 0.1013,
  "max_position_error_m": 0.2987,
  "position_loss_events": 0,
  "route": [1, 2, 3, 7, 11],
  "random_seed": 12345,
  "finish_reason": "destination reached"
}
```

The values above are copied from a real
`results/scenario_04_moderate_uwb_noise_run001_*_summary.json`; a run rewrites the
file, so `results/` is authoritative if the two ever disagree.

## 6. Fixtures

| File | Consumed by |
|---|---|
| `test_data/route_fixtures.json` | `tests/test_command_parser` (C++) and the simulator's fixture checks |
| `test_data/scenarios.json` | scenario definitions, regenerated from `scenario.py` |
| `test_data/uwb_valid.jsonl` | clean position stream |
| `test_data/uwb_noise.jsonl` | σ = 0.10 m noise |
| `test_data/uwb_dropout.jsonl` | 1.5 s gap |
| `test_data/uwb_invalid.jsonl` | `valid=false` and low quality samples |
| `test_data/uwb_jump.jsonl` | 1.5 m position jump |

Regenerate with `python scripts/generate_route_fixtures.py` and
`python scripts/generate_test_data.py`; verify with `--check`.
