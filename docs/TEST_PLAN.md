# Test Plan

| ID | Test | Current software evidence | Hardware pass criterion |
| --- | --- | --- | --- |
| 1 | Single motor STEP | portable stepper tests; firmware compiles | frequency/pulse width match command |
| 2 | AS5600 wrap | native forward/reverse wrap tests | no false revolution across 0/4095 |
| 3 | Encoder direction | inversion unit test | positive wheel motion reports positive RPM |
| 4–5 | RPM PID/step response | controller unit tests | logged error/overshoot within selected target |
| 6 | Straight 1–3 m | odometry integration test | measured drift reported against tape reference |
| 7–8 | 90°/180° turn | odometry rotation test | angle error reported against independent reference |
| 9–10 | Square/drift | procedure documented | repeatable endpoint and drift statistics |
| 11 | Dijkstra | native graph/integration suites | route matches graph on ESP32 telemetry |
| 12–15 | shelves/products/unreachable | simulator/dashboard route tests | no obstacle crossing; safe unreachable error |
| 16–19 | manual/link loss/E-stop | heartbeat and FSM tests | motion stops within measured bounds |
| 20 | persistence | CRC/NVS firmware implementation compiles | revision survives power cycle |
| 21 | encoder disconnect | wheel controller fault unit test | motor command goes to zero |
| 22 | mock UWB faults | 14 deterministic simulator scenarios | expected stop/recovery states |
| 23 | source switching | explicit source configuration | stop-apply-restart, no command jump |
| 24 | graph benchmark | plan time telemetry/native tests | bounded time/memory on target graph sizes |
| 25 | sustained operation | not physically run | no watchdog/reset/fault over selected duration |

Automated commands are listed in `README.md`. Physical tests require a dated
CSV/JSONL log, hardware revision, payload, supply/current settings, wheel and
track measurements, PID revision, operator, and pass/fail notes. A compile result
is not hardware verification.
