# Motor and Transmission Sizing

## Scope and assumptions

This calculation is a design bound, not a substitute for a motor torque-speed
curve or a loaded trolley test. The current prototype inputs are:

- total mass: 15, 20, and 30 kg;
- drive-wheel diameter: 0.100 m (provisional);
- target speed: 0.50 m/s;
- acceleration target: 0.30 m/s²;
- rolling-resistance coefficient: 0.03 on a smooth floor;
- small incline: 3 degrees;
- additional caster/bearing drag allowance: 5 N total;
- wheel-side mechanical efficiency: 0.85;
- belt efficiency for the reduction option: 0.90.

The drive wheels turn at:

```text
wheel_rpm = 60 v / (pi D) = 95.49 RPM at 0.50 m/s
```

Direct drive therefore requires about 95.5 motor RPM. A 2:1 reduction (motor
turns twice per wheel turn) requires about 191 RPM.

## STEP frequency

For a 200-full-step/revolution motor:

| Microstep | Direct drive | 2:1 reduction |
| --- | ---: | ---: |
| Full step | 318.3 Hz | 636.6 Hz |
| 1/2 | 636.6 Hz | 1,273.2 Hz |
| 1/4 | 1,273.2 Hz | 2,546.5 Hz |
| 1/8 | 2,546.5 Hz | 5,093.0 Hz |
| 1/16 | 5,093.0 Hz | 10,185.9 Hz |

The firmware's 20 kHz per-axis ceiling covers both options at 1/16
microstepping, but the selected TB6600 pulse-width and setup/hold requirements
must be verified. The timer produces a 25 microsecond high phase at its maximum
configured rate.

## Traction torque estimate

The force model is:

```text
F = m a + m g (Crr + sin(incline)) + Fcaster
Twheel_each = F r / (2 efficiency)
```

| Mass | Flat force | Flat torque/wheel | 3° force | 3° torque/wheel | Motor torque with 2:1 reduction |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 15 kg | 13.91 N | 0.41 N·m | 21.62 N | 0.64 N·m | 0.35 N·m |
| 20 kg | 16.89 N | 0.50 N·m | 27.15 N | 0.80 N·m | 0.44 N·m |
| 30 kg | 22.83 N | 0.67 N·m | 38.23 N | 1.12 N·m | 0.62 N·m |

The final column includes the assumed 90% belt efficiency. These values do not
include curb impacts, a jammed caster, or dynamic shock. The HUIYINN motor's
2.0 N·m holding torque must not be used as its available running torque; no
manufacturer torque-speed curve is currently available.

## Recommendation

Use a 2:1 belt reduction with each wheel on an independently supported axle.
It reduces the motor torque requirement, avoids placing wheel radial load on the
6 mm motor shaft, permits pulley-ratio changes, and makes wheel-axle AS5600
measurement possible. The costs are extra parts, belt tensioning, backlash, and
possible belt slip. Keep `sensor_to_wheel_ratio`, wheel diameter, and motor
microsteps configurable until the actual assembly is measured.

## Required physical test

1. Raise the wheels and verify direction, current setting, pulse polarity, and
   encoder direction at no more than 10 RPM.
2. Record current and encoder RPM at 25, 50, 100, 150, and 200 motor RPM.
3. Apply a measured axle load or use a torque arm; repeat at the intended
   supply voltage and TB6600 current setting.
4. Repeat with 15, 20, and 30 kg on the trolley at low speed, then increase the
   acceleration limit only after no stalls or missed-step faults occur.
5. Record driver/motor temperatures for at least 30 minutes. The final speed
   and load envelope is the measured stable envelope, not the calculation.
