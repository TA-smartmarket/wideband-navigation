# Desktop Simulator

The simulator is a 2D pygame application that drives the **same C++ navigation
core** as the ESP32 firmware. It adds only what is environment specific: trolley
physics, a UWB measurement model, rendering and experiment logging.

## 1. Installation

```bat
python -m venv .venv
.venv\Scripts\activate
python -m pip install -r simulator/requirements.txt
```

`simulator/requirements.txt` installs `pygame` and `numpy`. A C++ compiler must be
available for the native core (`g++` from MSYS2 on Windows); the build script
finds it automatically at `C:\msys64\ucrt64\bin\g++.exe` if it is not on `PATH`.

The navigation core is compiled into `build/native/navcore.dll` on first use (or
explicitly with `python scripts/build_native_core.py --force`).

## 2. Running

```bat
python simulator/main.py                    :: interactive, scenario 1
python simulator/main.py --scenario 5       :: start on scenario 5
python simulator/main.py --list             :: list scenarios
python simulator/main.py --speed 2          :: 2x time scale (0.5, 1, 2, 5)
python simulator/main.py --seed 42          :: override the random seed
python simulator/main.py --no-log           :: no CSV/JSON output
python simulator/main.py --headless --all   :: sweep all scenarios, print a table
python simulator/main.py --headless --all --json results/sweep.json
```

Scenario 11 is a deliberate failure drill (it must report `ERROR`); the sweep
counts it as a pass only when the FSM errors with the motors stopped.

`scripts/run_simulator.py` is a thin wrapper with the same options, provided
because the project structure documents it as the scenario runner:

```bat
python scripts/run_simulator.py --scenario 4 --headless
python scripts/run_simulator.py --all --headless --json results/sweep.json
```

## 3. Window layout

```text
+----------------------------------------------------+------------------+
|  SMART_MARKET_MAIN v1  12.0 x 8.0 m                |  NAVIGATION      |
|                                                    |  POSE            |
|   grid, graph edges, node ids                      |  CONTROL         |
|   planned route (orange), completed part (grey)    |  UWB / FAULTS    |
|   trajectory (cyan), UWB measurement (red)         |  METRICS (live)  |
|   true trolley (white body + nose)                 |  DEBUG           |
|   navigation input point (cyan ring)               |  CONTROLS        |
+----------------------------------------------------+------------------+
```

Rendering is a pure observer: it reads simulation state and never writes back
into navigation. World meters are converted to pixels only inside
`renderer.Viewport` (the vertical axis is flipped there).

## 4. Controls

The panel lists these on screen; the full reference is here.

| Key | Action | Key | Action |
|---|---|---|---|
| `1`–`9` | switch scenario | `SPACE` | run / pause |
| **left-click map** | set destination to the nearest node | **right-click** | start / resume |
| **middle-click** | cancel navigation | | |
| `Z` / `X` | previous / next destination node | `ENTER` | start / resume navigation |
| `R` | reset the run | `P` | force a replan |
| `N` | toggle UWB noise | `D` | toggle UWB dropout |
| `J` | inject a 1.5 m position jump | `I` | toggle invalid samples |
| `Q` | toggle low quality | `F` | toggle frozen position |
| `E` | emergency stop | `C` | clear E-stop (does **not** resume) |
| `G` | toggle node labels | `T` | toggle trajectory |
| `H` | toggle debug overlay | `ESC` | quit |

### Destination selection (any point)

Three ways to choose where the trolley goes, all without editing configuration:

| Method | How |
|---|---|
| **Click the map** | Left-click anywhere in the map area. The nearest graph node becomes the destination and the route is replanned immediately. A magenta ring shows which node a click will select *before* you commit (hover highlight). |
| **Cycle with keys** | `Z` / `X` step the destination through the graph nodes (typed `destination` nodes first, then the rest, wrapping). |
| **Script it** | `--destination <node-or-name>`, `--destination-at <x_m> <y_m>`, `--start-at <x_m> <y_m>` |
| **List the names** | `--list-nodes` prints every node with its name, type and coordinates |

```bat
python simulator/main.py --destination 4                   :: go to node 4
python simulator/main.py --destination produk-susu         :: go to node 10 by name
python simulator/main.py --destination-at 9.0 5.5          :: nearest node to that point
python simulator/main.py --start-at 9.5 5.8 --destination 1  :: custom start AND destination
```

A destination may be a numeric node id or a node `name` from `config/graph.json`
(`masuk`, `lorong-a1`, `kasir`, `persimpangan-b1`...`b4`, `lorong-c1`,
`produk-susu`, `produk-roti`, `parkir`), matched case-insensitively. An
unrecognised name is an error that lists the known names, rather than a silent
fallback to the scenario default.

A click (or `--destination-at`) is refused when the nearest node is further than
`max_pick_distance_m` (default 1.5 m, the same limit the planner uses to snap a
position onto the graph); the reason appears in RECENT EVENTS. This prevents a
click in an empty corner from silently targeting a node metres away.

The same applies to the start: `--start-at` places the trolley anywhere, and the
`ideal path` metric is measured from that effective start, so path efficiency
stays correct.

Other mouse actions: **right-click** starts/resumes navigation, **middle-click**
cancels it.

Closing the window (or pressing `ESC`) prints an `INTERRUPTED (run not finished)`
summary. It does **not** claim a navigation failure, and it does not evaluate
route metrics that only make sense once the route is complete: path efficiency
shows `n/a` and validation shows `INCOMPLETE` instead of `FAIL`, so stopping the
simulator never looks like a fault. A run that genuinely fails to arrive is
reported as `TIMEOUT` or `FAILED`, which is a different outcome.

`ENTER` also starts or resumes, and is the only way to resume after an emergency
stop: `C` clears the latch but deliberately leaves the trolley stopped so the
position and route are re-validated first.

The same list is shown in the status panel.

## 5. Map and graph

Loaded from `config/map.json` and `config/graph.json`. The loader validates the
frame id, the dimensions, duplicate node ids, unknown endpoints and negative
weights, and cross-checks `map_version` between the two files. Navigation itself
uses the C++ loader on the same JSON text.

## 6. True position vs. measured position vs. navigation input

Three distinct positions are maintained and rendered separately:

| Marker | Meaning |
|---|---|
| white body + nose | ground truth (plant) — evaluation only |
| red dot (+ line to truth) | UWB measurement sent to navigation |
| cyan ring | position the navigation core actually used |

Ground truth is never passed to the navigation core, so the reported metrics
measure the real effect of sensor error.

## 7. Sensor model and faults

`smart_trolley_sim/uwb_simulator.py` implements the measurement model:

| Fault | Behaviour | Keyboard |
|---|---|---|
| Gaussian noise | independent per-axis noise, σ configurable (0.02–0.20 m) | `N` |
| Dropout | no sample emitted at all (forced window or probabilistic burst) | `D` |
| Invalid sample | sample emitted with `valid = false` | `I` |
| Low quality | quality forced below `position.minimum_quality` | `Q` |
| Frozen output | the previous sample is repeated while the trolley moves | `F` |
| Position jump | measured position offset by 1.5 m for a window | `J` |

The quality model degrades with the noise level, so the navigation-side quality
gate is exercised realistically rather than being a separate switch.

### Published estimate (EKF stand-in)

The navigation contract states that navigation consumes the positioning
subsystem's final *estimated* position, so the simulator does not publish raw
white noise. `uwb.filter_alpha` sets a first-order smoother whose gain adapts to
the noise level, mirroring how a Kalman gain falls as the measurement covariance
grows:

$$\alpha = 1 - (1 - \alpha_{\min}) \cdot \min\left(\frac{\sigma}{\sigma_{\text{ref}}}, 1\right)$$

with `filter_alpha_min` = 0.45 and `filter_alpha_reference_sigma` = 0.20 m. A
clean sensor therefore keeps the filter open (α = 1, no smoothing) because
smoothing would only add lag through a turn, while a noisy sensor is smoothed
strongly. Measured over the scenario suite this cuts the published-sample error by
about 30 % on average, and by 43 % at σ = 0.20 m.

A deliberate position jump is applied *after* the smoother, as a step: a jump is
an estimator divergence, not something a filter should hide, and navigation must
react to it.

## 8. Scenarios

Fourteen deterministic scenarios (`smart_trolley_sim/scenario.py`). Each declares a
start pose, a destination, a UWB configuration and timed events. For a fixed
`random_seed` the noise pattern is reproducible, which makes runs comparable.

| # | Name | Expectation |
|---|---|---|
| 1 | `scenario_01_basic_navigation` | follows 1 → 2 → 3 → 7 → 11 and stops at node 11 |
| 2 | `scenario_02_long_route` | full traversal across the market |
| 3 | `scenario_03_multiple_turns` | corner handling keeps the trolley inside the acceptance radius |
| 4 | `scenario_04_moderate_uwb_noise` | completes with σ = 0.10 m |
| 5 | `scenario_05_high_uwb_noise` | completes with σ = 0.20 m, no replan storm |
| 6 | `scenario_06_position_dropout` | `POSITION_LOST` stops the motors, then recovery |
| 7 | `scenario_07_low_quality_position` | low quality samples rejected like invalid ones |
| 8 | `scenario_08_position_jump` | deviation confirmed after 500 ms → replan |
| 9 | `scenario_09_route_deviation` | cross-track error exceeds the replan threshold |
| 10 | `scenario_10_destination_change` | replans and diverts to the new destination |
| 11 | `scenario_11_unreachable_destination` | `ROUTE_NOT_FOUND` → `ERROR`, motors stopped |
| 12 | `scenario_12_emergency_stop` | instant stop, then safe recovery after `start` |
| 13 | `scenario_13_invalid_samples` | `valid=false` samples rejected; position ages out, then recovers |
| 14 | `scenario_14_frozen_position` | a frozen UWB output is treated as stale data |

Scenario 11 is a deliberate failure drill: it is expected **not** to arrive, and
the sweep counts it as a pass when the FSM reports `ERROR` with the motors
stopped.

The scenario definitions are also exported to `test_data/scenarios.json`
(`python -c "from smart_trolley_sim.scenario import export_scenarios_json; export_scenarios_json()"`).

## 9. Logging and metrics

Every run writes two artefacts (paths use the current date, never a hard-coded one):

```text
logs/<scenario>_runNNN_<YYYY-MM-DD_HH-MM-SS>.csv          raw telemetry, one row per logged cycle
results/<scenario>_runNNN_<YYYY-MM-DD_HH-MM-SS>_summary.json   aggregated metrics
```

The CSV columns are listed in [data_contract.md](data_contract.md#5-experiment-artefacts).
Metrics computed per run:

| Metric | Definition |
|---|---|
| planned distance | length of the planned route |
| travelled distance | distance actually covered (from accepted measurements) |
| completion time | simulation time at `ARRIVED` |
| **travelled distance** | `ground_truth_distance_m`: accumulated from the plant's true pose, i.e. pure translational motion. Never from measurements (each noisy sample adds a spurious displacement) and never from wheel rotation while spinning in place |
| UWB-stream path | length of the raw measurement stream. Always larger than the truth; it is a sensor-noise indicator, not motion |
| wheel odometry | from the plant's wheel travel (or the real STEP-pulse count with `use_stepper_model`). Matches the ground truth to 0.0000 m except when the trolley is externally displaced, which no wheel sensor can measure |
| ideal path distance | straight-line length of the route the trolley was told to drive, including the entry leg from where it started |
| excess distance | `ground_truth - planned` (negative means the trolley cut the corner inside the tolerance) |
| path efficiency | `planned / ground_truth * 100`. ~100 % means no detours |
| UWB RMSE | RMSE of the UWB measurement against ground truth **of the same sample** (sensor accuracy) |
| navigation-input RMSE | RMSE of the position the core used against ground truth of the same sample. Equals the UWB figure when no navigation-side filter is active |
| heading error (mean/max) | absolute heading error, **NAVIGATING samples only** |
| angular velocity (mean/max) | `abs(omega)`, NAVIGATING samples only |
| control activity | command changes beyond 0.01, plus genuine steering reversals (sign changes of right−left) and large steering corrections |
| sample accounting | `received` / `valid` / `invalid` / `dropped`. `invalid` counts only a *delivered* sample the core rejected; "no sample yet" is not an invalid sample |
| mean / max cross-track error | deviation from the route while navigating |
| destination error | final ground-truth distance to the destination coordinate |
| replan count | number of replans performed |
| position loss events | number of `POSITION_LOST` transitions |
| invalid samples | number of rejected samples |
| Dijkstra time | planning time in microseconds, measured by the C++ core |
| UWB counters | delivered / dropped / invalid / jumps |

The console prints a report at the end of each run:

```text
Navigation Result: SUCCESS
Final State: ARRIVED
Route: 2 -> 3 -> 7 -> 11
Planned Distance: 11.30 m
Travelled Distance: 12.14 m (ground truth)
Excess Distance: +0.84 m
Path Efficiency: 93.1 %
  (UWB-stream path 27.63 m, wheel odometry 12.14 m)
Completion Time: 32.8 s
Position RMSE: 0.3762 m (UWB), 0.3762 m (navigation input)
Mean Position Error: 0.1467 m
Max Position Error: 1.5986 m
Mean Cross Track Error: 0.195 m
Maximum Cross Track Error: 1.578 m
Heading Error: mean 20.58 deg, max 79.32 deg (NAVIGATING only)
Angular Velocity: mean 0.328, max 1.360 rad/s
Control Activity: 822 command changes, 22 steering reversals, 244 large steering corrections
Destination Error: 0.136 m
Replans: 2
Position Loss Events: 0
Samples: 329 received, 329 valid, 0 invalid, 0 dropped
Dijkstra Time: 2 us
```

This is the actual output of `scenario_08_position_jump`. Note the three
distance figures are all different on purpose: 11.30 m planned, 12.14 m
travelled (ground truth, and identical to wheel odometry because the wheels did
turn that far), and a 27.63 m UWB measurement stream inflated by noise. The
large max cross-track error is the 1.5 m injected jump being corrected.

## 10. Reference results

From `python simulator/main.py --headless --all` on the default configuration
(seed 12345, σ as configured per scenario):

<!-- BEGIN GENERATED REFERENCE TABLE -->
<!-- Generated by scripts/generate_reference_results.py - do not edit by hand. -->

| Scenario | State | Time (s) | Travelled (m) | Efficiency (%) | Mean / max XTE (m) | Dest. err (m) | Replans |
|---|---|---|---|---|---|---|---|
| 1 basic navigation | ARRIVED | 30.2 | 11.48 | 98.5 | 0.11 / 0.28 | 0.11 | 0 |
| 2 long route | ARRIVED | 32.9 | 12.98 | 100.2 | 0.11 / 0.25 | 0.13 | 0 |
| 3 multiple turns | ARRIVED | 34.1 | 12.91 | 100.8 | 0.10 / 0.28 | 0.13 | 0 |
| 4 moderate uwb noise | ARRIVED | 30.2 | 11.66 | 96.9 | 0.16 / 0.39 | 0.06 | 0 |
| 5 high uwb noise | ARRIVED | 32.6 | 11.90 | 95.0 | 0.21 / 0.56 | 0.14 | 0 |
| 6 position dropout | ARRIVED | 30.3 | 11.51 | 98.2 | 0.14 / 0.32 | 0.09 | 0 |
| 7 low quality position | ARRIVED | 32.9 | 11.43 | 98.9 | 0.11 / 0.32 | 0.16 | 0 |
| 8 position jump | ARRIVED | 32.8 | 12.14 | 93.1 | 0.20 / 1.58 | 0.14 | 2 |
| 9 route deviation | ARRIVED | 30.7 | 13.45 | 84.0 | 0.17 / 1.72 | 0.13 | 1 |
| 10 destination change | ARRIVED | 25.6 | 10.48 | 107.9 | 0.13 / 0.32 | 0.16 | 1 |
| 11 unreachable destination (drill) | ERROR | 0.0 | 0.00 | — | — | — | 0 |
| 12 emergency stop | ARRIVED | 32.6 | 11.46 | 93.4 | 0.14 / 0.32 | 0.10 | 0 |
| 13 invalid samples | ARRIVED | 30.2 | 11.55 | 97.9 | 0.14 / 0.33 | 0.09 | 0 |
| 14 frozen position | ARRIVED | 29.9 | 11.51 | 98.2 | 0.13 / 0.32 | 0.16 | 0 |
<!-- END GENERATED REFERENCE TABLE -->

Reading the table:

* **Efficiency above 100 %** (scenario 10 at 107.9 %) means the trolley travelled
  *less* than the straight-line plan, because it cut a corner inside the
  waypoint tolerance. That is normal and not a metric error.
* **Lower efficiency** appears only where the run is deliberately disturbed:
  scenario 9 displaces the trolley off the corridor and scenario 8 injects a
  position jump, so the extra distance is the recovery manoeuvre, not wandering.
  A clean run stays at 95-101 %.
* **Max XTE near 1.6-1.7 m** on scenarios 8 and 9 is the injected fault itself
  (a 1.5 m jump, a 2 m displacement); the *mean* stays near 0.20 m.
* **Scenario 6 shows 0 replans**: the dropout is handled by `POSITION_LOST`
  (motors stopped, then resume on the same route) rather than by replanning,
  which is the intended safer behaviour.
* **Scenario 11** is a failure drill and is expected to stop in `ERROR` with the
  motors cut; it reports `—` because no distance is travelled.


## 11. Result validation

Every run is classified against configurable acceptance bands
(`config/simulator.json → benchmarks`), reported per metric:

```text
Result validation:
  destination_error_m                    0.0594  target <= 0.2     PASS
  mean_cross_track_error_m               0.1572  target <= 0.25    PASS
  max_cross_track_error_m                0.3924  target <= 0.8     PASS
  path_efficiency_percent               96.9173  target >= 90.0    PASS
  replan_count                           0.0000  target <= 0.0     PASS
  position_loss_events                   0.0000  target <= 0.0     PASS
  mean_absolute_heading_error_deg       12.7416  target <= 30.0    PASS
  uwb_rmse_m                             0.1152  target <= 0.25    PASS
  OVERALL                                                          PASS
```

A breach is **reported, never fatal**: experimental metrics are allowed to fall
outside a target, and the sweep still counts a scenario as passing when the
trolley arrives. Bands are engineering bands that must also accept a 0.20 m-noise
run and a deliberate deviation drill, so they are looser than the per-scenario
targets (for example mean XTE ≤ 0.10 m on a clean route).

## 12. Stepper drivetrain (optional)

`physics.use_stepper_model = true` drives the simulated wheels through the same
C++ stepper model the A4988 firmware uses, so:

* the plant cannot accelerate faster than the configured STEP acceleration allows,
* `wheel_odometry_distance_m` becomes a genuine STEP-pulse measurement,
* the step-rate conversion is exercised end to end.

It is off by default so the validated baseline is unchanged.

## 13. Performance

The simulator runs comfortably in real time: the physics loop is a fixed-step
integrator decoupled from the render frame rate (an accumulator, capped at 200
steps per frame), and the navigation core is compiled at `-O2`. Planning happens
only at navigation start, on a destination change or on a replan — never per tick.

## 14. Design notes

* The plant uses a discrete first-order motor lag with
  $\alpha = 1 - e^{-\Delta t/\tau}$ (unconditionally stable) and midpoint
  integration for the unicycle model, so trajectories are accurate at 50 Hz.
* The trolley is never teleported along the route: it moves only in response to
  motor commands, which is what makes the PID and follower results meaningful.
* `right_gain = 0.97` models a small gearbox mismatch, so the heading estimator
  and the controller are exercised against a slightly asymmetric chassis.
* `wheel_noise_std_mps` is 0 by default so scenarios are bit-for-bit
  reproducible; raise it to study disturbance rejection.
