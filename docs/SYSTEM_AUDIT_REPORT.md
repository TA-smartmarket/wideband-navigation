# System Audit Report

**Audit date:** 2026-11-07  
**Scope:** repository state, portable navigation core, simulator, dashboard model,
ESP32-S3 firmware build configuration, and documented hardware interfaces.

## Verdict

**SIMULATION VALIDATED — not ready for uncontrolled hardware motion.** The
portable core is shared by the simulator and firmware and the current simulator
suite meets its scenario expectations. Firmware is compile-tested for the
declared N16R8 target. No physical trolley, TB6600, AS5600, UWB unit, power
supply, or mechanical measurements were available, so no hardware claim is
made.

## Architecture and ownership

| Concern | Authoritative implementation | Status |
|---|---|---|
| Graph / Dijkstra / FSM / heading follower | `navigation_core/` C++17 | Simulation verified |
| Simulator control decisions | `navigation_core` via C ABI; Python owns plant, UWB faults, rendering | Simulation verified |
| ESP32 integration | `firmware/` Arduino/FreeRTOS adapters | Compile tested |
| Dashboard | local FastAPI/WebSocket/serial model; no navigation reimplementation | Model tested |
| UWB/EKF | external JSON position producer | Interface only; hardware blocked |
| AS5600 feedback and odometry | two I2C controllers, firmware source | Compile tested; hardware blocked |

## Executed evidence

| Command | Result |
|---|---|
| `python scripts\\validate_config.py` | PASS: map/graph/navigation/simulator cross-file validation |
| `python -m unittest dashboard.test_models` | PASS: 17 tests |
| `python scripts\\benchmark_navigation.py --iterations 100` | PASS: 144/144 start-goal pairs against Bellman-Ford; median 38.5 us, p95 58.4 us, worst 257 us including JSON parsing |
| `python simulator\\main.py --headless --all` | PASS: 14/14 scenarios met expected outcomes using the existing native DLL. Disturbance scenarios intentionally report warning/fail quality bands while meeting safe-state expectations. |
| `pio run -e esp32-s3-devkitc-1` | Compile completed for local N16R8 configuration; not flashed or run on hardware. |
| `pio run -e esp32-s3-devkitc-1-mock` | Compile completed for local N16R8 configuration; not flashed or run on hardware. |
| `pio test -e native` and `python scripts\\build_native_core.py --force` | BLOCKED: host `gcc`/`g++` is unavailable from this workstation. The benchmark and simulator used the pre-existing native DLL and must not be treated as a fresh rebuild of the current C++ core. |

## Corrected defects

1. **P1: incompatible ESP32 board profile.** PlatformIO's built-in
   `esp32-s3-devkitc-1` identifies N8/no-PSRAM, while the declared device is
   N16R8. `boards/esp32-s3-devkitc-1-n16r8.json` and `platformio.ini` now select
   16 MB QIO flash, 8 MB OPI PSRAM, and `default_16MB.csv`.
2. **P1: STEP/DIR timing violation.** The timer ISR changed `DIR` on the same
   edge as a rising `STEP`, providing no controlled setup time. It now sets DIR
   after a falling edge and only emits the next rising STEP after one 25 us
   timer half-period. This is compile-tested, not scope-verified on the actual
   TB6600 module.

## Hardware feasibility and safety findings

- With a provisional 100 mm wheel, 0.5 m/s requires 95.5 wheel RPM and
  10,186 pulses/s at 1/16 microstepping (200 full steps/rev), within the
  20 kHz firmware ceiling. This is not proof of NEMA-23 torque at speed.
- Under the documented 15 kg, 0.30 m/s2, 3 degree, 0.03 rolling-resistance and
  5 N caster-drag assumptions, estimated demand is 0.64 N m per wheel. The
  2.0 N m advertised holding torque is not a running-torque specification.
- A 12 V/5 A supply remains **unverified and not approved**: phase-current
  settings, TB6600 variant efficiency, acceleration current, cable losses and
  thermal behaviour require measurement. Two AS5600 units require independent
  I2C buses or an address isolator; the firmware uses two controllers.
- GPIO assignments and TB6600 enable polarity are explicit placeholders in
  `firmware/include/pin_config.hpp`. Validate logic voltage, common-ground or
  isolation wiring, DIP current/microstep settings, direction, and pulse timing
  before connecting motor power.
- A software E-stop cannot replace a normally-closed, independent physical
  E-stop in the driver-enable or motor-power path. Static-map planning does not
  detect people or newly placed objects; unattended public-area operation is
  out of scope.

## Requirements traceability

| ID | Requirement | Implementation | Evidence | Status |
|---|---|---|---|---|
| NAV-01 | shortest graph route | `navigation_core/src/dijkstra.cpp` | independent 144-pair oracle | PASS |
| NAV-02 | safe position loss / E-stop | FSM and simulator scenarios 6/12-14 | headless scenario sweep | SIMULATION VERIFIED |
| ACT-01 | differential STEP/DIR output | `stepper.cpp`, `stepper_driver.cpp` | ESP32 compilation | PARTIAL |
| ACT-02 | TB6600 electrical/timing compatibility | pin/timer abstractions | no physical module or datasheet | BLOCKED |
| ENC-01 | dual AS5600 wheel feedback | `as5600_encoder.cpp`, dual I2C pins | compilation only | PARTIAL |
| UWB-01 | validated UWB position input | position provider/validator contract | simulator faults | SIMULATION VERIFIED |
| COM-01 | dashboard serial model and telemetry | `dashboard/`, serial protocol | 17 model tests | PASS |
| SAFE-01 | independent hardware stop and obstacle protection | documented requirement only | hardware absent | BLOCKED |

## Mandatory bring-up sequence

1. With motor power removed, flash the mock build and verify configuration,
   telemetry, command parsing, and latched E-stop state.
2. Verify each GPIO with an oscilloscope/logic analyser: disabled EN at boot,
   DIR setup/hold, STEP pulse width/rate, and zero output after E-stop.
3. Confirm TB6600 input polarity/voltage/current and DIP current/microstep
   settings from the exact module documentation; use a current-limited supply.
4. Wheels raised: validate one motor at <=10 RPM, then both; confirm encoder
   direction, wrap tracking, stale/error handling, and commanded-vs-measured
   speed.
5. Measure supply voltage/current and motor/driver temperature over 30 minutes
   at increasing load. Establish a measured torque-speed envelope before
   raising speed or acceleration limits.
6. Calibrate wheel radius, drive-wheel centre separation, encoder-to-wheel
   ratio, UWB frame/origin, timestamp age, and UWB confidence. Then run a
   restrained, spotter-supervised low-speed floor test with a physical E-stop.

## Remaining blockers

- Install a host GCC/MinGW toolchain and rerun native Unity tests after a fresh
  build.
- Provide exact TB6600 module documentation, motor torque-speed curve, wheel
  diameter/track, transmission ratio, and a power/thermal measurement.
- Complete physical UWB/EKF contract validation and all encoder, E-stop, and
  motor bench tests before autonomous floor testing.
