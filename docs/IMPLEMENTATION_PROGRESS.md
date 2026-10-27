# Implementation Progress

Updated: 2026-10-26

| Requirement | Status | Evidence / remaining work |
| --- | --- | --- |
| Shared Dijkstra, follower, heading PID, FSM | TESTED | native suites and 14 simulator scenarios |
| Arduino-ESP32 2.x STEP timer | TESTED | both ESP32 environments compile; physical pulse measurement pending |
| Dual AS5600 I2C acquisition | IMPLEMENTED | two independent buses, magnet/read fault handling; hardware pending |
| Angle unwrap and RPM filtering | TESTED | native wrap, inversion, ratio, stale/fault tests |
| Heading + left/right wheel PID | TESTED | bounded live updates reset controller state; portable tests and firmware integration compile; tuning pending |
| Encoder odometry and explicit heading | TESTED | native straight/rotation/heading tests; floor calibration pending |
| Encoder/mock/real UWB source model | IMPLEMENTED | explicit configuration, mock override, stop-before-switch command |
| Wheel/PID/odometry telemetry | IMPLEMENTED | NDJSON firmware/dashboard integration compiles; device test pending |
| NVS configuration persistence | IMPLEMENTED | versioned CRC + last-known-good fallback; ESP32 power-cycle test pending |
| USB dashboard/manual heartbeat | IMPLEMENTED | model tests; physical serial test pending |
| Wi-Fi dashboard control transport | NOT_STARTED | existing Wi-Fi adapters consume positioning only |
| Three.js visualization | TESTED | local room/racks/graph/route/trolley view, orbit/reset and stale warning rendered in Edge headless |
| Room/rack editor and route preview | TESTED | 3D move/resize/rotate gizmo, 2D drag editor, validated room resize, model tests and Edge render; preview is not the hardware planner |
| Map/graph persistence on ESP32 LittleFS | NOT_STARTED | repository/built-in graph remains authoritative at boot |
| Hardware wiring/power/mechanics | HARDWARE_VERIFICATION_PENDING | provisional documents and checklists added |

## Verified in this checkpoint

- `python scripts\validate_config.py`: pass.
- `pio test -e native`: 121/121 pass using a temporary w64devkit toolchain.
- `pio run -e esp32-s3-devkitc-1`: pass, RAM 16.5 %, Flash 13.3 %.
- `pio run -e esp32-s3-devkitc-1-mock`: pass, RAM 16.5 %, Flash 13.3 %.
- Dashboard model tests and JavaScript syntax check: pass.
- Dashboard state and local Three.js modules: HTTP 200; Edge headless rendered the scene.
- Manual ownership now stops the autonomous FSM before accepting commands;
  key release sends zero immediately and the 350 ms firmware watchdog remains active.
- Default editor scene contains three shelves and 15 editable product-to-node
  mappings; invalid graph-node mappings are rejected.
- `python simulator/main.py --headless --all`: 14/14 scenarios met expectations.

## Next safe implementation steps

1. Add device-hosted Wi-Fi command/telemetry transport with a single control lease.
2. Add transactional LittleFS map/graph storage and stop-before-apply workflow.
3. Perform wheels-raised AS5600, STEP timing, direction, E-stop, and NVS
   power-cycle tests before any floor test.
