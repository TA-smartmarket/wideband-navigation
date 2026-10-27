# Power Distribution

## Provisional laboratory supply

Use a regulated, current-limited 24 V DC bench supply rated for at least 10 A
(240 W) for the initial two-driver setup, subject to confirmation that the
purchased TB6600 modules accept 24 V. This is intentionally a supply-side
allowance, not `2 motors × 2 phases × 4 A`: a chopper driver transforms voltage
and current, so phase current is not equal to DC input current.

The 0.8 ohm windings dissipate up to roughly `I²R = 12.8 W` per energized phase
at 4 A. Actual DC input power depends on chopping mode, speed, back EMF, driver
losses, and current setting. Measure it. A provisional design envelope is
35–60 W electrical input per motor/driver under motion, plus acceleration and
thermal margin.

Use separate branches:

```text
24 V source
  -> main fuse and disconnect
  -> physical E-stop contactor/safety relay
     -> left driver branch fuse
     -> right driver branch fuse
  -> fused DC-DC 24 V to 5 V, >=3 A
     -> ESP32-S3 board
     -> regulated sensor rail as required by actual breakouts
```

Start with a 10 A main fuse and 5 A supply-side branch fuses only as provisional
values; select final fuse curves from measured steady and inrush currents and
the cable ampacity. The fuse protects wiring, not the motor.

## Battery option

A later battery system may use a nominal 24 V pack only when its maximum charged
voltage is within every connected device rating. It requires a BMS with
over-current, over/under-voltage, temperature protection and cell balancing; a
compatible charger; main fuse close to the pack; keyed disconnect; contactor;
and protected low-voltage conversion. Required capacity is:

```text
Ah >= average_input_watts * runtime_hours / (nominal_voltage * usable_fraction)
```

Example only: 120 W average for 2 h, 24 V nominal, and 80% usable capacity gives
12.5 Ah before aging and cold-temperature margin. Measure actual duty-cycle
power before purchasing the pack.

## Grounding, noise, and validation

- Keep high-current motor returns out of the sensor-return path.
- Twist motor phase pairs and keep them away from I2C/UWB wiring.
- Add driver-side bulk capacitance only according to the driver manual.
- Verify USB and external 5 V backfeed behavior before connecting both.
- Log AS5600 I2C errors while switching and accelerating both motors.
- Measure rail ripple and ground shift with an oscilloscope at the ESP32 and
  encoder connectors. Noise is acceptable only when communication and RPM
  estimates remain within the documented test limit.
