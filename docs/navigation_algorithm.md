# Navigation Algorithm

This document describes every algorithm the trolley runs, with the equations as
implemented in `navigation_core/`. Symbols follow the code: meters, radians,
seconds.

## 1. Coordinate frame and notation

The frame is `smart_market_map`: origin at the bottom-left of the market, `X+`
right, `Y+` up, angles counter-clockwise from `+X`. All navigation maths uses
meters and radians; pixel conversion happens only in the simulator renderer.

* $p = (x, y)$ — position in meters
* $\psi$ — heading (yaw) in radians, $\psi \in (-\pi, \pi]$
* $v$ — forward (linear) velocity in m/s
* $\omega$ — angular velocity in rad/s

## 2. Graph model

The market is a weighted graph $G = (V, E)$:

* a node $n \in V$ has an id, coordinates $(x_n, y_n)$ and a type
  (`intersection`, `aisle`, `destination`, `parking`, `entry`, `exit`, `checkout`);
* an edge $e = (u, v, w)$ has a weight $w$ in meters; when the configuration omits
  `weight_m`, the Euclidean distance is used:

$$w_{uv} = \lVert p_v - p_u \rVert_2 = \sqrt{(x_v - x_u)^2 + (y_v - y_u)^2}$$

Edges are bidirectional unless `"bidirectional": false`. The graph is validated at
start-up: duplicate ids, unknown endpoints, self loops, negative weights, invalid
coordinates and an empty graph are all rejected with a specific
`GraphIssue`.

Storage is a fixed-capacity adjacency list (`kMaxGraphNodes = 96`,
`kMaxGraphEdges = 384`) so the ESP32 build performs no heap allocation while
planning.

## 3. Nearest-node search

Given the measured position $p$, the planner's start node is

$$n^\* = \arg\min_{n \in V} \lVert p - p_n \rVert_2$$

The trolley is **not** teleported to $p_{n^\*}$: motion control keeps using the
real measured position. The snap is accepted only when

$$\lVert p - p_{n^\*} \rVert_2 \le d_{\text{snap}}$$

with $d_{\text{snap}}$ = `path.max_graph_snap_distance_m` (1.5 m). Otherwise the
position cannot be mapped onto the graph and the FSM reports
`START_NODE_NOT_FOUND` (`ERROR`), leaving the motors stopped.

When *replanning*, a wider radius
`path.replan_max_graph_snap_distance_m` (2.5 m) is used, because a replan is
triggered precisely when the trolley is already off the corridor; using the
initial limit there would turn a recoverable deviation into a fatal fault.

## 4. Dijkstra shortest path

Implemented from scratch (`navigation_core/src/dijkstra.cpp`), no third-party
shortest-path library.

**Algorithm.** Binary min-heap with lazy deletion:

```text
dist[s] = 0; dist[others] = INF
heap.push(0, s)
while heap not empty:
    (d, u) = heap.pop()
    if settled[u] or d > dist[u]: continue        # stale entry
    settled[u] = true
    if u == destination: break                    # settled with final cost
    for each neighbour (v, w) of u:
        if dist[u] + w < dist[v]:
            dist[v] = dist[u] + w; prev[v] = u; heap.push(dist[v], v)
```

Complexity is $O((V + E)\log V)$; the heap holds at most one entry per
relaxation, so its capacity is $2E + V$ and it is a fixed-size static buffer.

**Deterministic tie-breaking.** A uniform grid admits several equally short
routes. On equal cost the relaxation prefers the path whose predecessor has the
*smaller node id*:

```text
better  = candidate < dist[v]
prefer  = (candidate == dist[v]) and (id(prev[u]) < id(prev[v]))
```

Without this, the chosen route would depend on heap pop order. With it, the
development route from node 1 to node 11 is always `1 → 2 → 3 → 7 → 11` (10.0 m)
rather than the equally long `1 → 2 → 6 → 7 → 11`.

**Handled cases:** `start == destination` (single-node route, distance 0, not an
error), unknown start or destination (`INVALID_START` / `INVALID_DESTINATION`),
empty graph (`EMPTY_GRAPH`), unreachable destination (`NO_ROUTE`), directed edges,
multiple candidate routes.

**Timing.** The call is bracketed with a nanosecond monotonic clock and reported
in telemetry as `plan_time_us` (rounded up to whole microseconds, so a
sub-microsecond plan on a desktop CPU is not reported as 0). Planning runs only
at navigation start, on a destination change, or when a replan is required —
never every tick.

## 5. Route to waypoints

The node-id sequence becomes a coordinate sequence, because motion control needs
positions, not ids:

```text
waypoint_i = (node_id_i, x_i, y_i)
```

When the trolley already sits on the first node (within
`waypoint_tolerance_m`), that node is dropped from the waypoint list so the
trolley does not drive backwards to a node centre it is standing on.

## 6. Heading estimation

The UWB subsystem reports positions only, so the heading is estimated from
motion. A position difference **cannot observe rotation in place** (the trolley
spins about its own centre), so the estimator is a predictor–corrector:

**Prediction (every cycle).** Integrate the actually commanded yaw rate:

$$\psi_{k} = \mathrm{wrap}\left(\psi_{k-1} + \omega_{k-1}\,\Delta t\right)$$

**Correction (when a displacement is trusted).** Let
$\Delta p = p_k - p_{k-1}$, $d = \lVert \Delta p \rVert_2$. When
$d \ge d_{\text{min}}$ (a baseline that grows as the reported quality falls):

$$\psi_{\text{meas}} = \operatorname{atan2}(\Delta y, \Delta x), \qquad
\psi_k \leftarrow \mathrm{wrap}\left(\psi_{k-1} + \alpha \cdot \mathrm{wrap}(\psi_{\text{meas}} - \psi_{k-1})\right)$$

with `heading_correction_gain` $\alpha = 0.35$. Blending rather than assigning
matters: with σ comparable to the baseline the measured direction is tens of
degrees off, and an abrupt assignment makes the follower oscillate.

**Baseline.** `heading_min_displacement_m` = 0.50 m, scaled by quality:

$$d_{\text{min}} = d_{\text{min,base}} \left(1 + b\,(1 - q)\right)$$

with `heading_quality_baseline_boost` $b = 2.0$. While rotating in place the
correction is **skipped entirely**: the displacement produced by a spin is
tangential, so `atan2` would be ~90° away from the true heading and the follower
would rotate forever (a limit cycle observed during development).

Helpers: `normalizeAngle` wraps into $(-\pi, \pi]$;
`shortestAngularDistance(from, to) = wrap(to - from)`;
`calculateBearing(a, b) = atan2(b_y - a_y, b_x - a_x)`.

## 7. Waypoint tracking

A waypoint is reached when

$$\lVert p - p_{\text{wp}} \rVert_2 \le r_{\text{wp}}$$

with `waypoint_tolerance_m` = 0.20 m, and the final destination uses the tighter
`destination_tolerance_m` = 0.15 m. The index only ever increases.

To avoid oscillating at a node the manager also accepts a waypoint the trolley
has provably driven **past**:

$$\text{along} = \frac{(p - p_i)\cdot(p_{i+1}-p_i)}{\lVert p_{i+1}-p_i\rVert} \ge 0.5\, r_{\text{wp}},
\qquad d_{\perp} \le e_{\text{xt,max}}$$

where $d_\perp$ is the perpendicular distance to the leg's infinite line. A point
*behind* the waypoint ($\text{along} < 0$) never advances the route, so a single
noisy sample cannot.

**Entry leg.** The position where the route was planned is recorded as the entry
point, and while the trolley is still driving towards the first waypoint the
cross-track error is measured against the leg *entry → first waypoint*. Without
this, a trolley that had simply not yet joined the corridor would look metres off
route and replan in a loop.

**Cross-track error.** The reported value is the minimum distance from the
position to any leg the trolley still has to drive:

$$e_{\text{xt}} = \min_{i \ge i_{\text{current}}-1} d(p, \overline{p_i p_{i+1}})$$

Taking the minimum (rather than only the "active" leg) is what makes the metric
correct for a polyline: a trolley near the *next* leg is still on route. The
active leg for driving is the one leading **into** the current waypoint.

Point-to-segment distance:

$$d(p, \overline{ab}) = \left\lVert p - \left(a + \mathrm{clamp}\!\left(\frac{(p-a)\cdot(b-a)}{\lVert b-a\rVert^2}, 0, 1\right)(b-a)\right) \right\rVert_2$$

with the degenerate case $\lVert b-a \rVert \approx 0$ falling back to
$\lVert p - a \rVert$.

## 8. Path following

Each cycle, with the active waypoint target $p_t$:

1. $\psi_{\text{des}} = \operatorname{atan2}(p_{t,y} - y,\; p_{t,x} - x)$
2. $e_\psi = \mathrm{wrap}(\psi_{\text{des}} - \psi)$
3. $\omega_{\text{pid}} = \mathrm{PID}(\psi_{\text{des}}, \psi)$, saturated at
   $\pm\omega_{\max}$
4. $v_{\text{des}} = f(|e_\psi|, d_{\text{wp}}, \theta_{\text{turn}})$
5. rate-limit $v$ and $\omega$
6. convert to wheel commands

### Speed policy

| Condition | Forward speed |
|---|---|
| $\lvert e_\psi \rvert \le 25°$ | $v_{\max}$ |
| $25° < \lvert e_\psi \rvert < 55°$ | linear derating between $v_{\max}$ and $v_{\min}$ |
| $\lvert e_\psi \rvert \ge 55°$ | $0$ (rotate in place) |

$$v = v_{\min} + (v_{\max} - v_{\min})\left(1 - \frac{\lvert e_\psi\rvert - \theta_{\text{slow}}}{\theta_{\text{rot}} - \theta_{\text{slow}}}\right)$$

### Corner handling

A sharp turn at the *next* node derates the speed quadratically as the waypoint
approaches:

$$v_{\text{corner}} = v_{\min} + (v - v_{\min})\,t^2, \qquad t = \mathrm{clamp}\!\left(\frac{d_{\text{wp}}}{d_{\text{slow}}}, 0, 1\right)$$

with `corner_slowdown_distance_m` = 0.80 m and the turn angle measured between
the incoming and outgoing legs. The quadratic form matters: at cruise speed the
braking distance is comparable to the corner radius, and a linear ramp leaves the
trolley too fast to stay inside the acceptance radius on a 90° turn (this was
observed as a real overshoot during development).

### Lookahead (optional)

With `path.lookahead.enable = true`, the target is the point
`lookahead_distance_m` ahead along the route from the projection of the current
position, clamped to the next node. Default: **disabled** (the trolley drives to
node centres), so the acceptance-radius behaviour is the primary mode.

### Straight-segment stabilisation (angular deadband)

Residual UWB noise keeps injecting a small heading error even when the trolley is
tracking well, which shows up as weaving on long straights. Inside a configurable
deadband the angular command is tapered with a smoothstep:

$$s(e) = \begin{cases} 1 & |e| \ge \theta_{db} \\ x^2(3-2x),\; x = |e|/\theta_{db} & |e| < \theta_{db}\end{cases}
\qquad \omega = s(e)\cdot\omega_{PID}$$

with `motion.heading_deadband_deg` = 1.5°. Smoothstep rather than a hard threshold
because both the scale and its derivative vanish at the edge, so the command is
C1-continuous: a step would reintroduce chatter. The taper applies to the
**command only** - the reported heading error and all metrics keep the true value,
so the deadband cannot flatter the numbers.

### Final approach taper

Within `path.destination_slowdown_distance_m` of the destination the commanded
speed is scaled linearly down to `motion.min_linear_speed_mps`:

$$v \leftarrow \max\left(v_{min},\; v_{min} + (v - v_{min})\frac{d}{d_{slow}}\right)$$

so the trolley does not enter the acceptance radius at cruise speed, while still
being able to creep the last few centimetres instead of stalling outside it.

### Rate limiting

$$\Delta v_{\max} = a_{\max}\Delta t, \qquad \Delta\omega_{\max} = \alpha_{\max}\Delta t$$

This models the physical inability of the chassis to change speed instantly and
keeps the simulator's PID behaviour realistic.

**Wheel-command slew rate.** The body twist is rate limited, but the *wheel*
commands are limited again (`motion.max_motor_command_change_per_s`, 6.0/s) so a
real motor cannot be asked to reverse instantly. It is a second safety layer: with
the default tuning the upstream twist limiter is already tighter (0.064 per cycle
versus a 0.30 cap at 20 Hz), and it binds only if the other limits are relaxed.

**$\Delta t$ is measured, never assumed.** The loop uses the real elapsed time
between `update()` calls (clamped to $[10^{-4}, 4/\text{rate}]$ s). An early
revision used $1/\text{rate}$; the simulator steps the plant at 50 Hz while the
navigation rate is 20 Hz, so the yaw dead reckoning ran 2.5× too fast and the
trolley spun in place indefinitely. The firmware measures the same way, so a
scheduler glitch is handled identically.

## 9. PID heading controller

Derivative-on-measurement with a first-order filter and integrator clamping
(back-calculation):

$$u = K_p e + I + K_d \dot{d}_{\text{filt}}$$

where

* $e = \psi_{\text{des}} - \psi$ (the caller wraps the setpoint into range),
* the derivative acts on the measurement, $-(q_k - q_{k-1})/\Delta t$, then
  $d_{\text{filt}} \leftarrow \alpha d_{\text{raw}} + (1-\alpha) d_{\text{filt}}$
  with $\alpha = 0.25$; acting on the measurement avoids derivative kick from a
  setpoint step, and the filter limits noise amplification from a UWB-derived
  heading;
* the integral is clamped to `integral_limit`; when the sum saturates, the
  integrator is rewritten so that $K_p e + I + K_d d$ equals the saturated output
  exactly:

$$I \leftarrow \mathrm{clamp}(u_{\text{sat}} - K_p e - K_d d,\; I_{\min}, I_{\max})$$

This keeps the first response to a large error immediate (conditional
integration would refuse to accumulate and could output zero) while still
preventing windup.

**Robustness.** A zero, negative, non-finite or absurd $\Delta t$ falls back to
the nominal sample time; a non-finite measurement reuses the previous one and
leaves the integral untouched; a non-finite output resets the controller.

## 10. Differential drive

Given the body twist $(v, \omega)$ and wheel base $L$:

$$v_L = v - \frac{\omega L}{2}, \qquad v_R = v + \frac{\omega L}{2}$$

Forward kinematics: $v = \tfrac{1}{2}(v_L + v_R)$, $\omega = (v_R - v_L)/L$.
Wheel angular velocity: $\omega_w = v_w / r$.

Normalised commands in $[-1, +1]$:

$$c = \mathrm{clamp}\!\left(\frac{v_w}{v_{w,\max}} \cdot g,\; -1, +1\right)$$

with a per-side gain $g$ (motor mismatch trim) and a deadband below which the
command is zeroed so the controller does not park the motors in a buzzing
half-on state.

The chassis must be able to realise the largest commanded twist, so
`robot.max_wheel_speed_mps` must satisfy

$$v_{w,\max} \ge v_{\max} + \tfrac{1}{2}\omega_{\max} L$$

which `scripts/validate_config.py` enforces (0.45 + 0.5·1.5·0.32 = 0.69 m/s, so
the default is 0.70 m/s).

## 11. Deviation detection and replanning

The cross-track error is fed to a monitor with **time-based confirmation** and a
**cooldown**, so a single noisy sample cannot trigger a replan:

```text
if e_xt <= e_replan:                     # back inside the corridor
    reset pending deviation
elif not pending:
    pending = true; started_at = now     # first sample over the threshold
elif now - started_at < confirm_ms:
    keep waiting                        # 500 ms confirmation window
elif now - last_trigger < cooldown_ms:
    suppress                            # 1000 ms replan cooldown
else:
    trigger replan
```

* $e_{\text{xt}} > e_{\text{xt,soft}}$ (`max_cross_track_error_m` = 0.40 m):
  warning; the follower halves its forward speed while the window accumulates.
* $e_{\text{xt}} > e_{\text{xt,replan}}$ (`replan_cross_track_error_m` = 0.60 m)
  sustained for `route_deviation_confirm_ms` (500 ms): transition to `REPLANNING`.

A replan re-snaps the current position to the graph (wider radius), re-runs
Dijkstra to the same destination and rebuilds the waypoints. Replanning is also
triggered by a destination change and by position recovery after a significant
displacement.

## 12. Position loss safety

If no valid sample is accepted within `position.timeout_ms` (750 ms):

1. the motors are zeroed immediately,
2. the state becomes `POSITION_LOST`,
3. `position_loss_events` is incremented.

When a valid position returns, the trolley is re-snapped:

* the nearest node is unchanged **and** the cross-track error is inside the soft
  limit → resume `NAVIGATING` on the existing route;
* otherwise → `REPLANNING`, which re-derives the route from the current position.

## 13. Destination detection

`ARRIVED` when the distance to the final waypoint is within
`destination_tolerance_m` (0.15 m). The motors are then zeroed and the FSM stops
until a new destination is requested.

## 14. Turn authority

The yaw-rate ceiling `motion.max_angular_speed_radps` is the single most
performance-relevant limit for corner tracking. At 1.5 rad/s a 90° turn took
1.05 s, during which the trolley drifted ~0.5 m off the corridor and the
deviation monitor fired a replan. Raising it to 2.2 rad/s (turning time 0.71 s)
cut the mean cross-track error roughly in half on turn-heavy routes and removed
those spurious replans.

The chassis bound is **not** the naive $v_{max} + \tfrac{1}{2}\omega_{max}L$
combination: the speed policy derates $v$ as the heading error grows and commands
$v = 0$ at the rotate-in-place threshold, so the two twists it can actually
request are full speed straight ahead and full yaw rate on the spot:

$$v_{w,max} \ge \max\left(v_{max},\; \tfrac{1}{2}\omega_{max}L\right)$$

which `scripts/validate_config.py` and `validateConfig()` both enforce.

## 15. Configuration validation

Every parameter is range-checked before use (`validateConfig`): quality in
$[0,1]$, positive timeouts, positive tolerances, $e_{\text{xt,soft}} \le
e_{\text{xt,replan}}$, $v_{\min} \le v_{\max}$, rotation threshold in $[0,180]$,
positive accelerations, rates within bounds, positive wheel radius/wheel base,
and PID output/integral windows ordered. A failure puts the FSM in `ERROR` with
`CONFIG_INVALID` and the motors stay stopped.

## 16. Numerical robustness

Guarded throughout: division by zero (degenerate segments, zero wheel base, zero
dt), NaN/infinity in coordinates, angles, errors and outputs, zero-length vectors,
clamped control-loop $\Delta t$, and non-finite values in every telemetry field
(`sanitizeFinite`). Invalid input never propagates into the control loop.
