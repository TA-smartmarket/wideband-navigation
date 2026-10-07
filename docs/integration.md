# Integration Guide

This document is the contract between the navigation subsystem and the modules it
depends on. It states exactly what each side must provide and what changes at
integration time.

## 1. Responsibility boundary

```text
+--------------------------+--------------------------------------------------+
| Module                   | Responsibility                                   |
+--------------------------+--------------------------------------------------+
| UWB anchors              | RF ranging (raw distances)                       |
| Positioning subsystem    | anchor processing, position estimation, EKF,     |
| (external, "Yusuf")      | validity and quality estimation                  |
| Server / backend         | map, graph, product locations, destination       |
| (external, "Dzaki")      | requests, navigation session status              |
| Navigation (this repo)   | current position + destination -> wheel commands |
| Motor layer              | executes normalised left/right commands          |
+--------------------------+--------------------------------------------------+
```

Navigation **never** consumes raw ranging values. It consumes the final processed
position, so the internal design of the positioning subsystem is not this
project's concern.

## 2. What the positioning subsystem must send

Minimum viable payload, one JSON object per line:

```json
{
  "schema_version": 1,
  "trolley_id": "TROLLEY_01",
  "frame_id": "smart_market_map",
  "timestamp_ms": 123456,
  "position": { "x_m": 3.42, "y_m": 5.11 },
  "quality": 0.96,
  "valid": true
}
```

### Coordinate contract (must match exactly)

| Property | Value |
|---|---|
| Origin | bottom-left corner of the Smart Market map |
| `X+` | right |
| `Y+` | up |
| Unit | meter |
| `frame_id` | `smart_market_map` |
| Angle origin (if heading is ever added) | `+X`, counter-clockwise positive |

### Field requirements

| Field | Requirement |
|---|---|
| `schema_version` | integer `1` |
| `trolley_id` | `TROLLEY_01` (must match the configured trolley) |
| `frame_id` | exactly `smart_market_map` |
| `timestamp_ms` | monotonic milliseconds, non-zero, not from the future |
| `position.x_m`, `position.y_m` | finite numbers, meters |
| `quality` | `[0, 1]`; below `position.minimum_quality` (0.60) the sample is rejected |
| `valid` | `false` means "do not use this sample" |

### Behaviour expectations

* **Rate:** 10 Hz is the validated default; 5–50 Hz is acceptable. Navigation
  stops the trolley if no valid sample arrives within `position.timeout_ms`
  (750 ms), so the update rate must be well inside that window.
* **Latency:** the timestamp is used only for staleness detection; samples older
  than `stale_sample_ms` (1000 ms) are rejected.
* **Gaps:** emitting *nothing* during a dropout is the correct behaviour and is
  handled safely (`POSITION_LOST`). Emitting a repeated stale value is also
  handled (it ages out), but is worse practice.
* **Quality:** must degrade honestly with the true error. The heading estimator
  widens its displacement baseline as quality falls, so an over-optimistic
  quality value degrades navigation accuracy.
* **Accuracy:** the navigation side is verified against a position RMSE band. The
  simulator's acceptance limit (`config/simulator.json → benchmarks`) is
  `uwb_rmse_m <= 0.25` (warn above 0.4). The scenario suite stays inside it:
  0.1152 m at σ = 0.10 m and 0.1679 m at σ = 0.20 m, with the trolley still
  arriving at both (destination error 0.06 m and 0.14 m). Treat 0.25 m as the
  design target for the estimated position; if the real subsystem is worse, the
  cross-track error and replan rate are the first things to degrade, and
  `path.replan_cross_track_error_m` (0.60 m) is the limit that turns that error
  into replanning rather than a smooth route.
* **Units:** meters. Do not send millimeters or pixels.
* **Frame:** do not rotate or translate the map; the origin is the map's
  bottom-left corner as defined in `config/map.json`.

### Adding a heading (optional, future)

If the positioning subsystem later provides a yaw estimate, add it as
`"heading_rad"` inside `"position"`. The navigation core currently estimates the
heading itself (position differences plus commanded yaw-rate dead reckoning), and
a directly measured heading would improve in-place rotation accuracy. This is
backwards compatible: the field is ignored until the core is updated to use it.

## 3. Transport options

| Transport | Status | Where |
|---|---|---|
| USB serial (console UART) | **implemented** (development default) | `nav::SerialPositionProvider` |
| UART1 (dedicated link) | **implemented**, needs pin confirmation | `firmware::RealPositionProvider` |
| UDP / Wi-Fi | interface ready, not implemented | add one `IPositionProvider` |
| MQTT | interface ready, not implemented | add one `IPositionProvider` |
| ESP-NOW | interface ready, not implemented | add one `IPositionProvider` |
| WebSocket | interface ready, not implemented | add one `IPositionProvider` |

Adding a transport means implementing one class:

```cpp
class IPositionProvider {
public:
    virtual bool poll(nav::PositionMeasurement& out, uint64_t now_ms) = 0;
    virtual const char* name() const = 0;
};
```

The JSON parsing, validation, planning, following and control are unchanged.

## 4. Server / backend contract

Development works entirely without a server (local configuration files plus the
simulator). When the backend exists, these are the interfaces.

### `GET /map`

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

### `GET /graph`

```json
{
  "map": { "map_id": "SMART_MARKET_MAIN", "map_version": 1, "frame_id": "smart_market_map" },
  "nodes": [
    { "id": 1,  "x_m": 1.5,  "y_m": 2.0, "type": "entry" },
    { "id": 11, "x_m": 7.5,  "y_m": 6.0, "type": "destination" }
  ],
  "edges": [
    { "from": 1, "to": 2, "bidirectional": true }
  ]
}
```

`weight_m` is optional (Euclidean distance when omitted). `map_version` must match
`/map`; the loader rejects a mismatch.

### `GET /api/v1/scene` (static obstacles)

Navigation loads this response once at startup and again only when the map/scene
changes. It does not poll static obstacles with every position sample.

```json
{
  "scene": {
    "room": { "width": 5, "depth": 4, "height": 2.7 },
    "obstacles": [
      {
        "id": "obstacle-1", "label": "Wall",
        "x": 1.5, "y": 1.5, "z": 1.35,
        "sx": 3.0, "sy": 0.2, "sz": 2.7, "rot": 0.0
      }
    ]
  }
}
```

| Field | Navigation meaning |
|---|---|
| `x`, `y` | obstacle centre in `smart_market_map`, metres |
| `sx`, `sy` | full width/depth, metres (not half-extents) |
| `rot` | radians, counter-clockwise from `+X`; optional, defaults to `0` |
| `z`, `sz`, `atten`, `label`, `id` | accepted but unused by 2D planning |

`scene.room.width/depth` must match navigation's configured map dimensions. All
geometry values must be finite and `sx/sy` must be positive. Malformed or
mismatched scene data is rejected as a whole; navigation never silently plans
with a partial obstacle set.

Before Dijkstra runs, navigation inflates each rectangle by the configured
clearance on every side and removes every graph edge that touches or crosses it.
Dijkstra then chooses an existing alternate graph route. The graph must describe
all usable aisles: filtering prevents an unsafe edge from being used, but does
not invent new free-space waypoints around a rack.

```python
core = NavigationCore(
    graph_json,
    navigation_json,
    scene_json=scene_response_text,
    obstacle_clearance_m=0.20,
)

# On a later scene change; this safely stops and resets the session.
blocked_edges = core.apply_scene(new_scene_response_text, 0.20)
# Submit a fresh position and destination before resuming.
```

For local verification:

```bat
python simulator/main.py --headless --scene config/scene.example.json --obstacle-clearance 0.20
```

REST is the recommended transport for this static payload because the endpoint
already exists and updates are infrequent. Periodic trolley positions keep using
the existing `IPositionProvider` stream (UART today; MQTT/UDP can be added as a
provider without changing planning).

### `GET /destination?trolley_id=TROLLEY_01`

```json
{ "trolley_id": "TROLLEY_01", "destination_node": 11 }
```

### `POST /navigation/status`

Body: the telemetry document from
[data_contract.md](data_contract.md#2-telemetry-output), published at 5 Hz.

### `POST /navigation/command` (optional, not required by the trolley)

```json
{ "command": "navigate",          "trolley_id": "TROLLEY_01", "destination_node": 11 }
{ "command": "stop",              "trolley_id": "TROLLEY_01" }
{ "command": "cancel_navigation", "trolley_id": "TROLLEY_01" }
```

The same three operations are available on the serial console
(`destination <n>`, `stop`, `cancel`), so the backend is never a prerequisite for
a working trolley.

**The server never performs the motor PID loop.** Real-time control stays local on
the ESP32-S3.

## 5. Map versioning

`config/map.json` carries `map_id` and `map_version`; `config/graph.json` repeats
them under `map`. The loader refuses a graph whose version differs from the map, so
a stale map cannot be navigated silently. Bump `map_version` whenever node
coordinates, ids or edges change.

The current scene response has no version field. Until positioning adds one,
reload it together with `/map`; a reload resets the active navigation session.
Adding `map_version` and `frame_id` to the scene response is recommended so all
three documents can be checked before motion is enabled.

## 6. Development without the real hardware

| Missing piece | Substitute | Behaviour |
|---|---|---|
| UWB positioning | simulator UWB model, or `scripts/generate_mock_position.py` over serial | identical JSON contract |
| Real motors | `NullMotorDriver` / `SimulatedMotorDriver` | the state machine runs unchanged; only the electrical output is absent |
| Server | local `config/*.json` | the firmware embeds the graph and parameters |

## 7. Integration checklist

1. Confirm the positioning subsystem emits the documented JSON (frame, units,
   field names, timestamp semantics).
2. Point the real provider at the correct UART and baud
   (`pin_config.hpp`: `kUwbUartRxPin`, `kUwbUartTxPin`, `kUwbUartBaud`).
3. Set `NAVIGATION_USE_MOCK_POSITION 0` in `firmware/include/app_config.hpp`.
4. Verify with `python scripts/generate_mock_position.py` that the firmware
   parses the contract *before* connecting the real sensor.
5. Then connect the real subsystem and confirm telemetry shows
   `position_valid: true` and a plausible `position_quality`.
6. Calibrate the chassis: measure the true wheel base and wheel radius, update
   `config/navigation.json → robot`, and re-run `scripts/validate_config.py`.
7. Check motor polarity: with a small destination request the trolley must move
   towards the target. If it moves backwards, swap the affected `IN1`/`IN2` pins
   (do **not** negate commands in the navigation core).
8. Tune, in this order: `motion.heading_kp` (turn responsiveness),
   `path.waypoint_tolerance_m` (corner behaviour),
   `path.max_cross_track_error_m` / `replan_cross_track_error_m` (deviation
   sensitivity).

## 8. What must NOT change at integration

* the coordinate frame and units,
* the graph format and the map versioning scheme,
* Dijkstra, waypoint tracking and the follower,
* the PID and the differential-drive conversion,
* the telemetry schema,
* the state machine and its safety invariants.

Only the position provider implementation changes. If an integration step appears
to require touching the navigation core, that is a signal that the boundary is
being violated — the core is deliberately independent of any transport.

## 9. Troubleshooting integration

| Symptom | Likely cause |
|---|---|
| telemetry shows `position_valid: false` constantly | wrong `frame_id`, `schema_version`, or a stale/zero timestamp |
| `invalid_sample_count` climbing | quality below 0.60, `valid: false`, or NaN coordinates |
| `POSITION_LOST` repeatedly | update rate slower than 750 ms, or gaps in the stream |
| `START_NODE_NOT_FOUND` | the position is more than 1.5 m from every graph node (frame or origin mismatch) |
| trolley drives away from the target | `X+`/`Y+` convention or motor polarity |
| trolley oscillates near a waypoint | `heading_kp` too high, or `waypoint_tolerance_m` smaller than the position noise |
| frequent replans | noise σ above `replan_cross_track_error_m`, or a coordinate frame offset |
