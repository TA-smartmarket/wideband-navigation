# Testing

## 1. Test suites

```bat
pio test -e native                       :: all C++ suites
pio test -e native -f test_dijkstra      :: one suite
pio test -e native -f test_integration   :: one suite
```

| Suite | Location | Focus |
|---|---|---|
| `test_graph_dijkstra` | `tests/test_graph_dijkstra/` | graph construction and validation, Dijkstra correctness (including the specification's A–E example), nearest node and snap limits, reachability, polyline geometry |
| `test_geometry_pid_drive` | `tests/test_geometry_pid_drive/` | Euclidean distance, angle normalisation, shortest angular distance, bearings, point-to-segment distance, PID (proportional, saturation, integral clamp, anti-windup, derivative, degenerate dt/NaN), differential drive (inverse/forward kinematics, saturation, deadband, gain trim, config validation) |
| `test_waypoint_fsm` | `tests/test_waypoint_fsm/` | waypoint progression, destination detection, snap-to-nearest, passed-waypoint detection, remaining distance and cross-track, lookahead, deviation confirmation and cooldown, heading estimation, follower behaviour, position validation rules, the full FSM (boot sequence, invalid destination, unreachable destination, position timeout, E-stop, cancel/stop, invalid config, snap limit, replan), JSON parsing, providers, config loading, telemetry contract |
| `test_command_parser` | `tests/test_command_parser/` | console command parsing (including that position JSON is never mistaken for a command) and the shared route fixtures |
| `test_stepper` | `tests/test_stepper/` | STEP-rate/RPM/velocity conversion, steps per meter, configuration validation, continuous mode, DDS step accumulation, direction and inversion, position mode (exact stop, deceleration profile, negative moves), differential stepper drive, spin-in-place producing ~0 translational odometry |
| `test_integration` | `tests/test_integration/` | the complete pipeline with a plant in the loop |

Current status: **111 test cases, all passing**
(graph_dijkstra 15 + geometry_pid_drive 20 + waypoint_fsm 35 + command_parser 10 +
stepper 19 + integration 12).

## 2. Integration tests

These drive the real navigation core exactly as the simulator and firmware do:
positions are validated, snapped, planned, followed and actuated over many control
cycles, with a differential-drive plant in the loop.

| Test | Verifies |
|---|---|
| `full_route_from_arbitrary_start` | a start at (0.8, 0.9) snaps to node 1, plans `1→2→3→7→11` (10.0 m), arrives within tolerance, and stops the motors |
| `start_on_node_does_not_drive_backwards` | standing on node 1 leaves node 2 as the first waypoint (no turn-around) |
| `position_loss_stops_then_recovers` | a position timeout stops the trolley, no travel occurs while lost, and navigation resumes and completes |
| `invalid_and_low_quality_samples_are_rejected` | invalid samples and low quality increment the rejection counter and can age the position out; a healthy stream then completes |
| `emergency_stop_is_immediate_and_needs_restart` | the E-stop zeroes the motors immediately, the trolley does not move while latched, commands are rejected, `clearEmergencyStop()` alone does not resume, and an explicit `start` completes the route |
| `destination_change_replans` | changing the destination mid-route triggers a replan with a different route, and the trolley arrives at the new destination |
| `invalid_destination_reports_error_without_motion` | an unknown node is rejected and the motors stay at zero |
| `unreachable_destination_stops_safely` | an isolated node yields `ROUTE_NOT_FOUND`, `ERROR`, and zero motor output |
| `route_deviation_triggers_replan` | a perpendicular displacement is observed above the replan threshold and produces a replan with the trolley back on corridor |
| `telemetry_matches_status` | the published JSON agrees with the FSM status (state, destination, active waypoint, position) |
| `planning_time_is_measured` | the Dijkstra timing is non-zero and far inside the control period |
| `noise_does_not_prevent_arrival` | measurement error does not stop arrival and does not cause a replan storm |

## 3. Simulation scenarios as tests

```bat
python simulator/main.py --headless --all
python scripts/run_simulator.py --all --headless --json results/sweep.json
```

The sweep fails (non-zero exit) when a scenario that should succeed does not
arrive, or when the deliberate failure drill does not report `ERROR`. See
[simulator.md](simulator.md#10-reference-results) for the reference table.

## 4. Configuration and fixture checks

```bat
python scripts/validate_config.py                    :: validate all four config files
python scripts/generate_route_fixtures.py --check    :: fixtures still match the C++ core
python scripts/generate_test_data.py --check         :: JSONL fixtures up to date
```

`generate_reference_results.py` re-runs all fourteen scenarios and rewrites the
reference table in `docs/simulator.md`. The table is evidence, so it is generated
rather than hand-maintained: a hand-edited table silently goes stale the moment
the controller is tuned, and a stale table is worse than none. `--check` fails
when the committed table no longer matches a fresh run.

`validate_config.py` enforces the same rules as the C++ loader plus cross-file
consistency (map/graph version and frame) and physical consistency
(`max_wheel_speed_mps` must be able to realise the commanded twist). It is the
quickest way to catch a typo that would otherwise surface as odd driving
behaviour.

## 5. Fixtures

| File | Purpose |
|---|---|
| `test_data/route_fixtures.json` | planned routes and distances for eight start/destination pairs, generated **from the C++ core** |
| `test_data/scenarios.json` | the fourteen scenario definitions |
| `test_data/uwb_valid.jsonl` | clean position stream (σ = 0.02 m) |
| `test_data/uwb_noise.jsonl` | moderate noise (σ = 0.10 m) |
| `test_data/uwb_dropout.jsonl` | a 1.5 s gap in the stream |
| `test_data/uwb_invalid.jsonl` | `valid=false` and quality below the threshold |
| `test_data/uwb_jump.jsonl` | a 1.5 m position jump for 2 s |

The scenario suite also covers the invalid-sample and frozen-position faults
(scenarios 13 and 14), which are the two remaining §39 fault classes that have no
dedicated stream fixture: they are driven as timed events so the *navigation*
reaction is what is measured.

The route fixtures are the cross-language contract: the generator asks the C++
Dijkstra to plan each route, the C++ fixture test re-plans and compares, and the
Python tooling reads the same file. If the two implementations ever disagreed,
the fixture check fails.

## 6. Cross-language consistency

`navigation_core` is the single source of truth. The simulator does not
reimplement any navigation algorithm: it loads the compiled core through
`navigation_core/include/navigation/c_api.h`. `native_core.load_library()` calls
`nav_state_name()` for every enum value and compares it with the Python table, so
an ABI or enum drift fails immediately with an actionable message rather than
silently mis-indexing a status struct.

## 7. Test design rules

* **Behaviour, not implementation.** Tests assert on observable outcomes
  (routes, distances, states, motor commands, error codes), not on internal
  fields or source text.
* **Determinism.** No test depends on wall-clock time: the FSM is fed explicit
  monotonic timestamps, and the simulator uses a seeded RNG.
* **Isolation.** Each test builds its own graph, configuration and FSM, so tests
  can run in any order.
* **Boundaries and failures matter.** Saturation, NaN, infinity, zero dt, zero
  wheel base, degenerate segments, unreachable destinations, invalid samples and
  latched E-stops all have explicit cases.
* **No tautologies.** No test asserts only that a function returned without
  throwing, that a container is non-empty, or that a value equals itself.

## 8. Adding a test

1. Add the case to the relevant suite in `tests/<suite>/test_main.cpp`.
2. Register it with `RUN_TEST(...)` in that file's `main()`.
3. Run `pio test -e native -f <suite>`.

A new suite is a new directory `tests/test_<name>/` containing `test_main.cpp`
with `setUp`, `tearDown` and `main`. PlatformIO discovers suites whose directory
name starts with `test_`; flat `.cpp` files directly in `tests/` are compiled into
every suite and must not define `main`.

## 9. Failure policy

A failing test is a bug to fix, not a test to relax. During development this rule
caught several real defects, including:

* the follower assuming `dt = 1/navigation_rate` (the simulator steps the plant
  at 50 Hz, so yaw dead reckoning ran 2.5× too fast and the trolley spun forever);
* cross-track error measured on the leg *after* the current waypoint, making a
  correctly tracking trolley look metres off route (215 spurious replans);
* position-derived heading corrections applied during in-place rotation, where
  the displacement is tangential (a permanent spin limit cycle);
* a replan failing its own snap test because the trolley is by definition
  off-corridor when a replan is triggered;
* PID anti-windup implemented as conditional integration, which suppressed the
  first response to a large error;
* Dijkstra tie-breaking depending on heap pop order, so the chosen route varied
  between equal-length alternatives;
* the replan snapping the trolley to the nearest node *behind* it, commanding it
  to drive backwards and oscillating forever between that node and the
  destination (now restricted to the part of the route still ahead);
* the yaw dead reckoning gated on `has_heading_`, which left the estimate frozen
  while the trolley rotated in place so the heading error never shrank;
* the follower assuming the trolley already faced its target when no heading had
  been observed, which produced a zero error and a full-speed straight run in the
  wrong direction (fixed; the turn-heavy scenario now reports a mean cross-track
  error of 0.103 m and a mean absolute heading error of 12.1 deg);
* the travelled-distance metric accumulating the *measurement* stream, which
  inflated it because every noisy sample adds a spurious displacement (16.09 m
  reported for a 12.98 m path, an overstatement of 24 %);
* the DDS step accumulator zeroing its fractional remainder whenever a cycle
  produced no whole step, which on the firmware's 20 kHz pulse base meant the
  stepper would **never emit a pulse** at any usable speed;
* the stepper position profile capping only the profile target, not the emitted
  rate, so it could ask the motor to decelerate harder than its torque allows
  (step loss);
* `planned_distance_m` reporting the plan from the second waypoint, so a route
  whose first node the trolley already stood on was under-reported (10.00 m for
  an 11.30 m plan) and path efficiency was meaningless;
* a replan overwriting `planned_distance_m` with the *remaining* distance, which
  made path efficiency depend on when the replan happened;
* the navigation-side `heading_kp` being silently overridden by a duplicated
  value in the `heading_pid` block, so the documented tuning knob did nothing;
* position RMSE including samples taken before the first position was accepted,
  where the navigation position is still the (0,0) default (a bogus 2.5 m max);
* the simulator clock starting at 0 ms, so the very first, perfectly good sample
  was rejected as a stale timestamp and counted as an invalid sample.
