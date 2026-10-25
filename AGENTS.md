# Smart Trolley Navigation — Project Review Guide

Read this file first when reviewing or changing this repository. It is an
evidence-based map of the workspace, not a replacement for `README.md` or the
documents in `docs/`.

## Project purpose

This repository implements autonomous navigation for a Smart Trolley:

```text
UWB position -> validation -> Dijkstra route -> waypoint/path follower
             -> differential-drive command -> motor driver -> trolley
```

The main engineering rule is that navigation logic has one source of truth.
`navigation_core/` is portable C++17 and is shared by the ESP32 firmware, native
C++ tests, and the Python simulator. Do not reimplement Dijkstra, path following,
or the navigation FSM in Python or firmware-local code.

## Repository map

| Path | Review focus / responsibility |
| --- | --- |
| `navigation_core/include/navigation/` | Public C++ interfaces: graph, Dijkstra, geometry, PID, differential drive, FSM, position validation, telemetry, C ABI. |
| `navigation_core/src/` | Portable implementation. No Arduino, GPIO, serial, Wi-Fi, graphics, or Python dependencies belong here. |
| `firmware/src/`, `firmware/include/` | ESP32-S3 Arduino/FreeRTOS integration: tasks, serial/UWB providers, TB6600 step/dir driver, telemetry, hardware-facing safety. |
| `simulator/` | Pygame simulation, UWB faults, trolley plant, renderer, CSV/JSON run output. It loads the native C++ core through `ctypes`; it is not a second navigation implementation. |
| `dashboard/` | Local FastAPI/WebSocket control station, browser UI, serial bridge, map/rack editing, and dashboard model tests. |
| `config/` | Versioned map, graph, navigation, and simulator configuration. `firmware/src/app_context.cpp` embeds development copies of graph/navigation configuration. |
| `test_data/` | Deterministic UWB, scenario, and route fixtures. |
| `tests/` | PlatformIO/Unity C++ unit and integration tests. |
| `scripts/` | Native-core build, configuration validation, simulator runner, fixture/reference-data generation, and mock-position tools. |
| `docs/` | Architecture, coordinate/data contracts, algorithms, integration, firmware, simulator, tests, and troubleshooting. |
| `wokwi/` | Optional logic-only ESP32 simulation assets. |
| `logs/`, `results/` | Generated simulator/dashboard outputs; do not treat these as source. |

## Start a review in this order

1. Read `README.md` for supported workflows and project status.
2. Read `docs/architecture.md` and `docs/data_contract.md` for layer and UWB
   contracts.
3. Inspect `config/map.json`, `config/graph.json`, and `config/navigation.json`.
4. Follow the C++ path from `navigation_core/include/navigation/c_api.h` and
   `navigation_core/include/navigation/navigation_fsm.hpp` into the relevant
   implementation.
5. Review the matching boundary:
   - simulator behaviour: `simulator/smart_trolley_sim/`;
   - ESP32 hardware: `firmware/src/main.cpp`, `motor_driver.cpp`, and
     `stepper_driver.cpp`;
   - local operator UI/serial: `dashboard/main.py` and `dashboard/serial_bridge.py`.
6. Review or run tests appropriate to the affected layer before drawing a
   conclusion.

## Verified project commands

Run commands from the repository root.

```bat
:: Validate JSON configuration and cross-file consistency
python scripts\validate_config.py

:: Native C++ unit + integration tests
pio test -e native

:: Build ESP32-S3 firmware (does not upload)
pio run -e esp32-s3-devkitc-1

:: Build firmware with mock positions on USB serial
pio run -e esp32-s3-devkitc-1-mock

:: Run all simulator scenarios without a window
python simulator\main.py --headless --all

:: Dashboard model tests
python -m unittest dashboard.test_models
```

For interactive simulator and dashboard startup, use the exact dependency and
launch commands in `README.md`.

## Review invariants and safety boundaries

- Preserve the simulator and its shared-core architecture.
- Preserve the `smart_market_map` frame: origin bottom-left, metres, +X right,
  +Y up. The external UWB message contract is documented in `docs/data_contract.md`.
- Position, graph/map, motor, and UI concerns must stay separated. A UWB
  position is global reference data; encoder odometry is relative motion data.
- Never claim a feature is hardware verified without physical evidence.
  Distinguish implemented, simulation-tested, compile-tested, unit-tested, and
  hardware-verified results.
- All real GPIO assignments and TB6600 enable polarity are centralised in
  `firmware/include/pin_config.hpp`. They are placeholders until verified on the
  actual ESP32-S3/TB6600 wiring; do not add pin numbers elsewhere.
- Validate finite numeric input at API, serial, and control boundaries. A bad
  serial line must never reach motor control.
- E-stop must remain latched and must stop motor output before any recovery.
- Do not edit generated files below `.pio/`, `build/`, `logs/`, or `results/`.

## Current feature status to report accurately

| Area | Status |
| --- | --- |
| Shared Dijkstra, waypoint, path follower, heading PID, differential drive, FSM | Implemented in `navigation_core/`; native tests cover these areas. |
| Pygame simulator, UWB fault scenarios, logs, metrics | Implemented. |
| ESP32 serial NDJSON control, manual heartbeat, TB6600 STEP/DIR path, E-stop | Implemented in source; requires firmware build and physical bench verification. |
| Dashboard map/rack editor, Dijkstra preview, serial connection UI | Implemented locally in `dashboard/`. |
| Real wheel encoders, wheel-velocity PID, encoder odometry, NVS configuration persistence | Not implemented/verified as a complete hardware loop. Do not present dashboard placeholders as real measurements. |
| Real UWB/EKF and final mechanical/pin/TB6600 DIP settings | External hardware integration required. |

## Change guidance

- A navigation/configuration schema change may require coordinated updates to
  `navigation_core`, `config/`, `firmware/src/app_context.cpp`, tests, docs, and
  simulator bindings. Check each deliberately.
- Keep the firmware navigation task non-blocking. High-frequency STEP timing
  belongs to a hardware/peripheral timer path, not `delay()` loops.
- Make small, focused changes. Existing user changes may be present; do not
  reset or overwrite unrelated work.
- For a review request, report: files inspected, observed architecture, tested
  evidence, defects/risks ordered by severity, and hardware-dependent unknowns.
