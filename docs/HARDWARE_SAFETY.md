# Hardware Safety

The software E-stop is latched and immediately disables STEP output, but it is
not the primary personnel-safety device. The physical E-stop must remove the
driver-enable or motor-power path through an independent, normally closed,
fail-safe circuit that does not depend on the ESP32, Wi-Fi, USB, or firmware.

Removing stepper power also removes holding torque. The trolley therefore needs
a mechanical parking strategy or a test surface where it cannot roll. Do not
test on ramps or in occupied public space.

## Safe state rules

- Boot, invalid configuration, encoder failure, stale position, tracking fault,
  communication timeout, and E-stop command all produce zero motion.
- Clearing E-stop never resumes a route. A new start command is required.
- Closed-loop firmware does not fall back to open-loop when AS5600
  initialization fails.
- Manual commands expire after 350 ms without a heartbeat.
- Map collision checks cover configured static rectangles only. They cannot
  detect people, carts, dropped objects, or any other dynamic obstacle.

## Progressive commissioning

1. Motor power disconnected: configuration, protocol, telemetry, and E-stop
   state tests.
2. Driver enabled but motor mechanically disconnected where practical.
3. Wheels raised, current-limited supply, one axis at a time.
4. Wheels raised, two-axis velocity and direction tests.
5. Restrained trolley at very low speed in a clear test area.
6. Controlled flat-floor experiments with a spotter and physical E-stop.

Autonomous operation around the public is outside the verified envelope until
independent obstacle/person detection and a formal risk assessment exist.
