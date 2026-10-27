# Navigation Algorithm and PID Audit

**Audit date:** 2026-10-27  
**Scope:** shared Dijkstra planner, heading PID, wheel-velocity PID, dashboard
tuning path, configuration validation, simulator evidence, and firmware build.

## Executive result

- Dijkstra is correct for all 144 ordered start/destination pairs in the current
  12-node graph when compared with an independent Bellman-Ford oracle.
- The planner uses a bounded adjacency list and binary min-heap with complexity
  `O((V + E) log V)`. It is appropriate for the ESP32-S3 graph limits.
- The former dashboard-only Python Dijkstra was removed. Dashboard, simulator,
  and firmware now use `navigation_core` as the single planner implementation.
- Heading PID has output saturation, circular error handling, derivative on
  measurement, derivative filtering, anti-windup, measured `dt`, and state reset
  after live gain changes.
- Heading and left/right wheel gains can be changed from the dashboard in the
  range `[0, 10]`. `APPLY` changes runtime values; `SAVE` persists accepted
  values to NVS for the next boot.
- Wheel PID defaults remain provisional until measured on the physical trolley.
  Simulation cannot identify TB6600/NEMA23 load, wheel slip, backlash, battery
  sag, encoder alignment, or mechanical asymmetry.

## Reproducible Dijkstra benchmark

Command:

```text
python scripts/benchmark_navigation.py --iterations 500
```

Current Windows host result for 72,000 public-C-ABI calls:

| Metric | Result |
| --- | ---: |
| Correctness pairs | 144 / 144 |
| Median | 38.0 us |
| Mean | 46.6 us |
| p95 | 88.1 us |
| p99 | 115.8 us |
| Worst observed | 4,361 us |
| Throughput | 21,452 queries/s |

This is a conservative end-to-end measurement because every C ABI call parses
the graph JSON. With the graph already loaded, simulator telemetry measured the
pure planning operation at roughly 1-3 us. Both are comfortably below the
50 ms navigation period, but host timings are not a substitute for an on-device
ESP32 timing capture.

## PID evidence

The deterministic 14-scenario sweep met all scenario expectations. In the
clean/moderate-noise scenarios, destination error was 0.10-0.15 m, mean
cross-track error was 0.11-0.17 m, and mean absolute heading error was
16.4-26.0 degrees. High UWB noise reached 0.29 m destination error and was
correctly classified as a warning. Position jumps, dropouts, route deviation,
and E-stop recovery remained safe but naturally produced warning/fail benchmark
bands for those disturbance-specific runs.

The current starting gains are:

| Loop | Kp | Ki | Kd | Status |
| --- | ---: | ---: | ---: | --- |
| Heading | 1.60 | 0.00 | 0.05 | Simulation-tested baseline |
| Left wheel RPM | 0.35 | 0.08 | 0.01 | Provisional; physical tuning required |
| Right wheel RPM | 0.35 | 0.08 | 0.01 | Provisional; physical tuning required |

Tune wheel loops with the trolley raised and current-limited, one loop at a
time. Start with `Ki=0` and `Kd=0`, increase `Kp` until response is fast without
sustained oscillation, add only enough `Ki` to remove steady-state error, then
add small `Kd` only if measured noise permits. Verify both directions and
multiple RPM targets before floor testing. Use the dashboard chart to compare
target versus actual RPM, apply gains temporarily, and persist only a stable
set.

## Remaining risks

1. `config/map.json` and `config/graph.json` currently describe a 9 x 6 m map,
   while `config/navigation.json` and the firmware's embedded development copy
   still describe 12 x 8 m. `scripts/validate_config.py` correctly rejects this.
   Resolve the intended physical map and update all copies together before a
   hardware navigation test.
2. Native Unity tests could not run on this workstation because native `gcc` /
   `g++` is not installed. The changed C++ sources did compile successfully for
   ESP32-S3, and Python/dashboard tests passed, but this is not equivalent to a
   native unit-test pass.
3. The Dijkstra scratch heap is static by design and is not reentrant. The
   firmware navigation task is single-owner; dashboard access is serialized by
   a lock.
4. PID gains cannot be called optimal until wheel RPM step-response data and
   floor-route data are captured from the assembled trolley. Report software
   baseline, simulation-tested, compile-tested, and hardware-verified states
   separately.
