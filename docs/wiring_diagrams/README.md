# Smart Trolley Wiring Diagrams

The diagrams in this folder are split into separate sheets so power, motor
control, encoders, and communications remain readable.

1. `01_power_distribution_estop.png` — 24 V distribution, fuses, contactor,
   physical E-stop, and low-voltage conversion.
2. `02_tb6600_motor_control.png` — ESP32-S3 STEP/DIR/ENABLE signals, verified
   interface stages, TB6600 terminals, and NEMA23 coil wiring.
3. `03_dual_as5600_encoders.png` — independent I2C buses and mechanical magnet
   alignment for both AS5600 encoders.
4. `04_uwb_usb_optional_io.png` — UART UWB/EKF, USB service, MQTT telemetry,
   status LED, and optional buzzer.

These diagrams are commissioning references, not proof of physical
verification. GPIO assignments, TB6600 input topology and enable polarity,
fuse values, sensor supply compatibility, and all purchased-module ratings
must be checked before motor power is applied.
