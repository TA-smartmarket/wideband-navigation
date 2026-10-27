# Hardware Wiring

All GPIO assignments below are provisional and centralized in
`firmware/include/pin_config.hpp`. Confirm them against the exact ESP32-S3 N16R8
DevKitC-1 and purchased TB6600 modules before applying motor power.

| Function | Left | Right | Verification |
| --- | --- | --- | --- |
| TB6600 STEP | GPIO 4 | GPIO 7 | oscilloscope pulse width/frequency |
| TB6600 DIR | GPIO 5 | GPIO 15 | wheels raised, low RPM |
| TB6600 ENABLE | GPIO 6 | GPIO 16 | active polarity on purchased module |
| AS5600 SDA | GPIO 17 | GPIO 8 | separate I2C controller per sensor |
| AS5600 SCL | GPIO 18 | GPIO 9 | 400 kHz provisional |
| Real UWB UART | RX 43 | TX 44 | 115200 baud, 3.3 V logic |

Both AS5600 sensors have fixed address `0x36`; they must not share an
unseparated bus. The implementation uses the ESP32-S3's two I2C controllers.
A TCA9548A is the fallback when the selected board or pin routing prevents two
independent buses.

## Signal path

```text
ESP32 GPIO -> verified level/interface stage -> TB6600 PUL/DIR/ENA inputs
24 V motor supply -> fuse -> physical E-stop/contactor -> two TB6600 drivers
regulated 5 V -> ESP32 board input (verify USB backfeed behavior)
regulated 3.3 V -> AS5600 breakout only if its board is 3.3 V compatible
```

Generic TB6600 modules use different optocoupler input circuits. Measure input
current and consult the exact module documentation. If 3.3 V GPIO does not meet
the guaranteed input threshold, use a proper transistor/line-driver stage; do
not connect the 24 V motor domain to ESP32 pins.

## Encoder mechanics

- Use a diametrically magnetized magnet, coaxial with the measured wheel axle.
- Verify AS5600 magnet-detected status before allowing closed-loop motion.
- Fix the air gap and prevent axial/radial movement.
- Prefer wheel-axle mounting. Motor-shaft mounting cannot observe belt slip and
  must use the exact reduction ratio.
- Route each SDA/SCL pair away from motor phases and STEP wiring; keep it short,
  use a ground reference, and verify actual pull-up voltage/resistance.

## Stepper phases

Never infer coil pairs from wire colors. With all power disconnected, use a
multimeter to identify the two continuity pairs, label them A+/A- and B+/B-,
then connect them according to the purchased motor and driver documentation.
Swapping one pair reverses the motor; prefer the configured direction inversion
after the electrical pairing is known.

## Bring-up checklist

1. Disconnect motor power and verify every GPIO has no conflict with boot,
   flash/PSRAM, or native USB.
2. Verify logic and sensor rails and common/reference wiring.
3. Verify ENABLE produces the safe disabled state at boot and E-stop.
4. Check STEP, DIR setup, and pulse levels with an oscilloscope.
5. Connect one driver and one motor with wheels raised and current limited.
6. Verify one AS5600, then the second, including wraparound and direction.
7. Connect both axes and repeat E-stop before any floor test.
