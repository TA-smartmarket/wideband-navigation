# Troubleshooting

Symptoms are grouped by where they appear. Each entry names the likely cause and
the concrete check or fix.

## 1. Simulator will not start

### `ModuleNotFoundError: No module named 'pygame'`

```bat
python -m venv .venv
.venv\Scripts\activate
python -m pip install -r simulator/requirements.txt
```

### `no C++ compiler found`

The simulator compiles the navigation core into a shared library. Install MSYS2
and its UCRT64 g++ (`pacman -S mingw-w64-ucrt-x86_64-gcc`), or point the build at
an existing compiler:

```bat
set CXX=C:\path\to\g++.exe
python scripts/build_native_core.py --force --verbose
```

The script also auto-detects `C:\msys64\ucrt64\bin\g++.exe` and
`C:\msys64\mingw64\bin\g++.exe`.

### `Could not find module 'navcore.dll' (or one of its dependencies)`

The library was built against a dynamically linked C++ runtime. Rebuild with the
provided script, which links the runtime statically:

```bat
python scripts/build_native_core.py --force
```

### `navigation core ABI mismatch: nav_state_name(...)`

The compiled library is stale relative to the Python bindings. Rebuild:

```bat
python scripts/build_native_core.py --force
```

### `nav_create failed: ...` on start-up

The graph or navigation configuration was rejected. The message names the
problem:

```bat
python scripts/validate_config.py
```

### `font not initialized`

Only occurs if `Renderer` is constructed before pygame is initialised. The
renderer calls `pygame.init()`/`pygame.font.init()` itself; if you are embedding
it in your own script, construct the renderer before creating fonts, or call
`pygame.init()` first.

### The window opens but the trolley never moves

Check the status panel:

| Panel shows | Meaning | Fix |
|---|---|---|
| `State WAITING_FOR_POSITION` | no valid sample accepted yet | press `N` to re-enable noise, `Q`/`I`/`F` to clear stuck faults, or `R` to reset |
| `State IDLE` after pressing `C` | the emergency stop was cleared but not resumed | press `ENTER` to start (the position and route are re-validated first) |
| `Quality` below 0.60 with `INVALID` | samples rejected | check `position.minimum_quality` |
| `State ERROR`, `start node not found` | the position is more than 1.5 m from every node | confirm the start pose is inside the map |
| `State ERROR`, `route not found` | the destination is unreachable | pick a different destination (scenario 11 does this on purpose) |
| `Left motor` / `Right motor` both 0 with `State NAVIGATING` | the trolley is spinning to align | wait a moment; if it never resolves, see §4 |

## 2. Simulator behaves oddly

### The trolley circles forever

Two historical causes, both fixed; if it reappears, check that the core is
rebuilt:

1. **Heading corrections during in-place rotation.** A spin produces a tangential
   displacement, so a position-derived heading would be ~90° off. The estimator
   now skips corrections while rotating in place and relies on commanded yaw-rate
   dead reckoning.
2. **Wrong `dt`.** The follower must use the measured elapsed time, not
   `1/navigation_rate`. If a build assumes the nominal rate while the plant steps
   faster, the dead reckoning runs too fast.

```bat
python scripts/build_native_core.py --force
```

### The trolley replans constantly

`replan_count` climbing every second means the cross-track error is above
`replan_cross_track_error_m` most of the time. Causes:

* noise σ above the replan threshold (scenario 5 with σ = 0.20 m is intentionally
  close to the limit) — reduce `uwb.noise_std_m` or raise the threshold;
* the cross-track reference covering only one leg — the implementation takes the
  minimum over all remaining legs, so this should not occur with a current build;
* a coordinate mismatch (map origin/frame) — verify with
  `python scripts/validate_config.py`.

### The trolley overshoots corners

Corner slowdown is quadratic with `corner_slowdown_distance_m`. Increase it
(0.80 m → 1.00 m) or reduce `motion.max_linear_speed_mps`.

### The trolley stops just short of the destination

The destination radius is `path.destination_tolerance_m` (0.15 m) while the
waypoint radius is 0.20 m. With high noise, increase
`destination_tolerance_m` slightly (0.15 → 0.20 m), but keep it smaller than the
waypoint radius so the destination remains the tighter criterion.

### The trolley weaves on long straights

`motion.heading_deadband_deg` (default 1.5 deg) tapers the angular command
inside that band so residual UWB noise stops steering the trolley. Raise it (up to
~3 deg) for a cleaner sensor, or set 0 to disable the taper. If the weave persists,
raise `motion.max_motor_command_change_per_s` is *not* the fix - check the chassis
calibration (`robot.wheel_base_m`, `robot.wheel_radius_m`) instead.

### The path looks wiggly / the trolley wanders

Check `Path Efficiency` in the run summary. Around 100 % means the trolley drove
the route as planned; much below that means real detours.

* With a very low `uwb.noise_std_m` the estimate should be almost unfiltered. If
  it is not, raise `uwb.filter_alpha` towards 1.0 (or lower
  `filter_alpha_reference_sigma`): over-smoothing a clean sensor adds lag through
  turns, which shows up as wide corner-cutting arcs.
* With a noisy sensor, lower `filter_alpha_min` for stronger smoothing.
* Persistent weaving at a constant offset usually means the chassis is not
  calibrated: set the true `robot.wheel_base_m` and `robot.wheel_radius_m`.

### `speed_multiplier` does not change behaviour

The multiplier only changes how much simulation time passes per rendered frame.
Navigation always uses the correct simulated `dt`, so the trajectory is identical
at 1x and 5x — only the wall-clock duration differs. That is intentional.

### Runs are not reproducible

Set a fixed seed (the default is 12345):

```bat
python simulator/main.py --seed 12345
```

`physics.wheel_noise_std_mps` must be 0 (the default) for bit-for-bit
reproducibility; any non-zero value introduces an unseeded disturbance.

## 3. C++ tests

### `Error: Nothing to build. Please put your test suites to the 'tests' folder`

PlatformIO discovers suites in directories whose name starts with `test_`. A
flat `.cpp` file in `tests/` is compiled into every suite and must not define
`main`. Put each suite in `tests/test_<name>/test_main.cpp`.

### `multiple definition of 'main'` / `'setUp'`

Two files in the same suite both define `main`. Exactly one file per suite
directory may define `main`, `setUp` and `tearDown`.

### `navigation/dijkstra.hpp: No such file or directory`

The include path is missing. `platformio.ini` sets
`-I navigation_core/include` in `[common_flags]`; confirm that section was not
edited and that the file lives at `navigation_core/include/navigation/dijkstra.hpp`.

### `-Wpedantic` errors from tinyusb headers

`-Wpedantic` is intentionally enabled only for the native environment. The ESP32
Arduino framework headers use flexible array members and anonymous structs, which
`-Wpedantic` rejects. Do not add it to `[common_flags]`.

## 4. ESP32 firmware

### Build: `undefined reference to 'setup()'` / `'loop()'`

`build_src_filter` excluded `firmware/src`. The ESP32 environment must compile its
own sources as well as the core:

```ini
build_src_filter =
    +<*.cpp>
    ${core_sources.build_src_filter}
```

### Build: `'isFinite' is not a member of 'nav'`

`nav::isFinite` lives in `navigation/geometry.hpp`. Include it in any header or
source that uses it.

### Upload fails or the port is busy

* Close `pio device monitor` (or any other program) holding the port.
* Windows: check Device Manager for the COM number; use `COM3`, `COM4`, `COM5`, …
* If the board is not detected, hold **BOOT**, press **RESET**, release **BOOT**,
  then upload.
* On boards with native USB, the port may change after reset; re-check
  `pio device list`.

### Boots but motors never move

Read the boot log:

| Log line | Meaning |
|---|---|
| `H-bridge init failed: falling back to the null driver` | LEDC configuration failed; check `pin_config.hpp` for pin conflicts |
| `motor output disabled at build time` | the environment was built with `NAVIGATION_ENABLE_MOTOR_OUTPUT=0` |
| `graph load failed` / `navigation config load failed` | the embedded JSON is malformed (should not occur in a released build) |
| `ready: motors stopped, waiting for a destination` | normal; send `CMD:destination 11` then `CMD:start` |

Motors also stay stopped until a **valid** position has been accepted and a route
has been planned. Check `CMD:status`.

### `POSITION_LOST` in telemetry

No valid sample arrived within `position.timeout_ms` (750 ms). Check:

* the generator's rate: `--rate` must keep samples inside the timeout window;
* `frame_id`, `schema_version`, `trolley_id` spelling;
* `quality` ≥ `position.minimum_quality`;
* the timestamp is non-zero and not stale.

`CMD:status` shows `UWB Quality` and `Position Valid`; `invalid_sample_count` in
telemetry counts rejections.

### `START_NODE_NOT_FOUND`

The accepted position is more than `path.max_graph_snap_distance_m` (1.5 m) from
every graph node. Usually a frame/origin mismatch or the trolley is outside the
map rectangle. Verify the coordinate convention in
[coordinate_system.md](coordinate_system.md).

### Trolley drives in the wrong direction

Do **not** negate commands in the navigation core. Swap the affected motor's
`IN1`/`IN2` pins in `pin_config.hpp`. If both wheels are reversed, swap both
pairs. If the trolley turns the wrong way for a left turn, the left/right motor
assignments are swapped.

### Serial commands are ignored

* Commands must be newline-terminated.
* Position JSON must not be prefixed with `CMD:`.
* Check the monitor baud rate: 115200.
* If position JSON and commands share the link, prefix commands explicitly:
  `CMD:destination 11`.

### Watchdog resets while navigating

The navigation task resets a 5 s watchdog every cycle. A reset means the loop
stalled — usually a blocking call added to `TaskNavigation`. The control loop must
never perform serial I/O, file access or network calls; keep those in the I/O
tasks. `SYS samples=... overruns=...` in the periodic log reports loop overruns.

## 4b. Stepper motors (NEMA 17 + A4988)

### The motors buzz but do not turn

* `stepper.microsteps` must match the A4988 MS1/MS2/MS3 wiring. A mismatch does
  not stop motion, but every distance and RPM figure will be wrong by the ratio.
* Check `CMD:stepper`: it reports `steps/rev`, `steps/m`, the position in steps
  and revolutions, the STEP rate and the DIR level. If `rate` stays 0 while
  navigating, the commanded speed is below the STEP timer resolution.
* `max_step_rate_hz` must not exceed `kStepperTimerBaseHz` (20 kHz in
  `pin_config.hpp`), otherwise the pulse train cannot be generated at all.

### The motor loses steps / position drifts

Almost always acceleration. Lower `stepper.max_step_accel_hz_per_s` (the torque
available falls with speed, and a NEMA 17 at 12 V cannot follow a steep ramp), or
lower `motion.max_linear_speed_mps`. Confirm with `CMD:move 10` and compare the
reported revolutions with a mark on the shaft: the pulse count is what the driver
was *asked* for, not what the motor achieved.

### One wheel turns the wrong way

Set `stepper.left.invert_direction` / `stepper.right.invert_direction` to `true`
in `config/navigation.json`. Do **not** negate commands in the navigation core.

### The motors get hot while idle

They are holding position. Set `kStepperHoldWhenIdle = false` in
`pin_config.hpp` to disable the drivers while stopped (loses holding torque), or
lower the A4988 Vref.

### Spinning in place registers as travel in the odometry

It should not: the two wheels move equal and opposite amounts, so
`wheel_odometry_distance_m` stays near zero while the yaw changes. If it does not,
the two axes have different `microsteps` or wheel radius.

## 5. Configuration

### `navigation.json: robot.max_wheel_speed_mps (...) is below ...`

The chassis cannot realise the largest commanded twist. Either raise
`robot.max_wheel_speed_mps` or lower `motion.max_linear_speed_mps` /
`motion.max_angular_speed_radps`:

$$v_{w,\max} \ge v_{\max} + \tfrac{1}{2}\omega_{\max} L$$

### `graph.json: map.map_version (...) does not match map.json`

Bump `map_version` in **both** files together after editing node coordinates or
edges.

### `graph.json: duplicate node id` / `edge references a nonexistent node`

The message names the offending id. Node ids must be unique and every edge
endpoint must exist.

### `navigation.json: path.replan_cross_track_error_m must be >= max_cross_track_error_m`

The soft limit must not exceed the hard limit, otherwise the warning state would
never be observable.

## 6. Scripts

### `pyserial is required for serial output`

```bat
python -m pip install pyserial
```

Or use `--dry-run` to preview the generated stream without a port.

### `cannot open COM5: ...`

The port is in use or does not exist. Close `pio device monitor` first, and check
the port with `pio device list`.

### The mock stream "moves too fast" or "too slow"

The realised speed is `--spacing × --rate`. The script prints both and warns when
they disagree with `--speed`. To target 0.45 m/s at 10 Hz use `--spacing 0.045`.

### Closing the window says `INTERRUPTED (run not finished)`

That is correct and intended: the route was not completed, so route metrics
(path efficiency, destination error, cross-track error) are not evaluated and
validation reports `INCOMPLETE`. It is not a crash and not a navigation failure.
Let the run reach `ARRIVED`, or use `--headless --all` for a full report.

### `docs/simulator.md reference table is out of date`

The scenario results changed (a controller or configuration edit). Regenerate:

```bat
python scripts/generate_reference_results.py
```

### `route_fixtures.json is out of date`

The core changed (or a fixture was hand-edited). Regenerate:

```bat
python scripts/generate_route_fixtures.py
```

The same applies to `python scripts/generate_test_data.py --check`.

## 7. Diagnostics cheat sheet

```bat
:: configuration
python scripts/validate_config.py

:: core ABI and library
python -c "from smart_trolley_sim.native_core import core_library_path; print(core_library_path())"

:: one scenario, verbose console report
python simulator/main.py --scenario 1 --headless

:: every scenario
python simulator/main.py --headless --all

:: C++ tests
pio test -e native

:: firmware build
pio run -e esp32-s3-devkitc-1-mock

:: console command reference on the device
CMD:help
CMD:status
```
