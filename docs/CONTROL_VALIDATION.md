# Control Validation

## Wheel velocity

Run fixed setpoints in both directions at 10, 25, 50, 75, and 100% of the
validated limit. Log target RPM, raw/filtered RPM, P/I/D terms, output RPM,
saturation, encoder angle/revolutions, STEP count, and timestamps.

For samples after the settling window calculate:

```text
MAE  = mean(abs(target - measured))
RMSE = sqrt(mean((target - measured)^2))
SSE  = mean(target - measured)
overshoot = (peak - target) / abs(target) * 100%
```

Initial engineering target: 5–10% steady-state RPM error, no persistent
oscillation, and no tracking fault under the documented mass/supply condition.
This is a target, not a measured result.

## Odometry

Measure wheel diameter under load and wheel track using repeated straight and
rotation trials. Compare encoder odometry against an independent tape/angle
reference; do not use STEP count as the ground truth. Test 1 m, 3 m, 90°, 180°,
and repeated square paths in both directions. Report bias, standard deviation,
final pose error, and drift per metre.

## PID procedure

1. Wheels raised; set Ki and Kd to zero and start with small Kp.
2. Increase Kp until response is prompt without sustained oscillation.
3. Add only enough Ki to remove repeatable steady-state error; verify anti-windup
   during saturation and obstruction.
4. Add filtered derivative only if it improves damping more than it amplifies
   AS5600 noise.
5. Repeat under floor load and both travel directions. Save each accepted
   revision to ESP32 NVS explicitly; slider movement must not write flash.

## Navigation metrics

The simulator already records completion time, destination error, cross-track
error, route length, replans, and UWB error. Hardware trials must label the pose
source and independent ground truth. Never combine simulated and physical
results in one unlabeled statistic.
