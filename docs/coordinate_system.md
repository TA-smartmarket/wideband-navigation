# Coordinate System and Units

Every module in this project — navigation core, firmware, simulator, logging,
tests — uses one coordinate convention. Mixing conventions is the most common
source of "the trolley drives the wrong way" bugs, so the rules are stated once
and enforced by tests.

## 1. Frame

| Property | Value |
|---|---|
| `frame_id` | `smart_market_map` |
| Origin | bottom-left corner of the Smart Market map |
| `X+` | right |
| `Y+` | up |
| Unit | meters |
| Angle origin | `+X` axis |
| Positive rotation | counter-clockwise |
| Angle range (internal) | radians, wrapped to $(-\pi, +\pi]$ |

```text
        Y+ (up)
         ^
         |
         |
   (0,0) +------------------> X+ (right)
   bottom-left corner
```

The map rectangle therefore spans $[0, \text{width}] \times [0, \text{height}]$
with the default development map being $12 \times 8$ m.

## 2. Units

| Quantity | Unit | Example identifier |
|---|---|---|
| Position | meters | `x_m`, `y_m` |
| Distance | meters | `distance_to_waypoint_m` |
| Linear velocity | m/s | `linear_velocity_mps` |
| Angular velocity | rad/s | `angular_velocity_radps` |
| Heading / bearing / error | radians | `heading_rad`, `heading_error_rad` |
| Angular acceleration | rad/s² | `max_angular_accel_radps2` |
| Time | milliseconds (timestamps), seconds (dt) | `timestamp_ms`, `dt_s` |
| Quality | dimensionless $[0, 1]$ | `quality` |
| Motor command | dimensionless $[-1, +1]$ | `left_motor`, `right_motor` |

Angles are configured in degrees **only** where a human tunes them
(`rotate_in_place_threshold_deg`, `heading_error_slow_deg`); they are converted
internally. Every identifier carries its unit suffix, so a bare `distance` or
`angle` never appears in the code.

## 3. Angle conventions in detail

* `0 rad` points along `+X` (right). `+π/2 rad` points along `+Y` (up).
* Counter-clockwise is positive: a left turn increases the heading.
* `normalizeAngle(θ)` wraps into $(-\pi, +\pi]$ and returns `0` for NaN/inf.
* `shortestAngularDistance(from, to) = normalizeAngle(to - from)` always takes
  the short way around the ±π branch cut.
* `calculateBearing(current, target) = atan2(Δy, Δx)`.

Consequences used by the controller:

* heading error $e_\psi = \text{shortestAngularDistance}(\psi, \psi_{\text{des}})$;
* a positive $e_\psi$ means "turn left", so the differential drive produces
  $v_R > v_L$;
* the rotate-in-place threshold is compared on $|e_\psi|$.

## 4. Where pixel conversion happens

**Only** in the simulator renderer (`simulator/smart_trolley_sim/renderer.py`).
`Viewport.to_screen()` and `Viewport.to_world()` are the single conversion points,
and the vertical axis is flipped there because screen `Y+` points down while world
`Y+` points up.

No navigation algorithm, test, configuration value or log field uses pixels. The
renderer's scale factor is derived from the map rectangle and the window size, so
world units never depend on the screen resolution.

## 5. Multiple position concepts in the simulator

The simulator deliberately keeps three positions separate:

| Concept | Meaning | Used for |
|---|---|---|
| Ground truth | the true pose of the simulated plant | rendering, evaluation metrics only |
| UWB measured | ground truth + noise/jump/dropout | what is sent to navigation |
| Navigation input | the position the C++ core accepted | control decisions |

The navigation core only ever receives the **UWB measured** value. Ground truth
never enters the control loop — this is what makes the metrics meaningful and is
rendered as three distinct markers so an experiment can show the difference.

## 6. Time conventions

* External samples carry `timestamp_ms`; it is used **only** to detect stale
  samples, never to compute the control `dt`.
* The control loop measures `dt` from a local monotonic clock
  (`nav::monotonicMillis()` / `monotonicNanos()` on the host,
  `esp_timer`-backed equivalent on the ESP32 via the same header).
* Remote clocks are not trusted: a sample from the future or a timestamp that
  jumps backwards cannot distort the loop.

## 7. Validation of positions

A sample is rejected when any of the following holds
(see [data_contract.md](data_contract.md) for the exact rules):

* `frame_id != "smart_market_map"`,
* a coordinate is NaN or infinite,
* `valid == false`,
* `quality < position.minimum_quality`,
* the position is further outside the map rectangle than `map_margin_m`,
* the timestamp is missing or older than `stale_sample_ms`.

`config/map.json` also carries `map_id` and `map_version`; the loader refuses a
graph whose `map.map_version` disagrees with the map, so a stale map cannot be
navigated by accident.
