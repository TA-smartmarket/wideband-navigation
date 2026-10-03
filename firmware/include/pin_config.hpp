// GPIO / peripheral pin assignment for the ESP32-S3 DevKitC-1.
//
// These are *assumptions* documented in docs/esp32_firmware.md: any pin can be
// changed here without touching the navigation code.  Pins are chosen to avoid
// the ESP32-S3 flash/PSRAM pins (26-32) and the native USB pins (19/20) used by
// the console.
#pragma once

// --- H-bridge motor driver (e.g. TB6612FNG / L298N / DRV8833) ---------------
//
// Two driven wheels, one H-bridge channel each.  PWM uses the ESP32 LEDC
// peripheral; IN1/IN2 select the direction.

struct MotorPins {
    int left_pwm;
    int left_in1;
    int left_in2;

    int right_pwm;
    int right_in1;
    int right_in2;
};

/// Left motor on GPIO 4/5/6, right motor on GPIO 7/15/16.
inline constexpr MotorPins kMotorPins{
    /*left_pwm=*/4,
    /*left_in1=*/5,
    /*left_in2=*/6,
    /*right_pwm=*/7,
    /*right_in1=*/15,
    /*right_in2=*/16,
};

// --- LEDC PWM configuration -------------------------------------------------
inline constexpr int kPwmFrequencyHz = 20000;  // above audible range
inline constexpr int kPwmResolutionBits = 10;  // 0..1023 duty
inline constexpr int kLeftPwmChannel = 0;
inline constexpr int kRightPwmChannel = 1;

// --- Stepper drivers (NEMA 17 + A4988 class) --------------------------------
//
// A step/dir driver replaces the H-bridge when `drive_kind` is "stepper" in
// config/navigation.json.  Each axis needs three signals:
//
//   STEP : one pulse per microstep; frequency sets the speed
//   DIR  : logic level selects the direction (set before the pulse train)
//   EN   : active LOW enable; HIGH disables the driver (no holding torque)
//
// The A4988 needs STEP pulses at least 1 us wide, so the pulse generator must
// never run faster than the timer base frequency; `max_step_rate_hz` in the
// configuration is validated against it.

struct StepperPins {
    int step;
    int dir;
    int enable;  // active LOW on A4988-class drivers
};

/// Left axis on GPIO 4/5/6, right axis on GPIO 7/15/16 (same pins the H-bridge
/// used, so the wiring harness does not change).
inline constexpr StepperPins kStepperPinsLeft{
    /*step=*/4,
    /*dir=*/5,
    /*enable=*/6,
};
inline constexpr StepperPins kStepperPinsRight{
    /*step=*/7,
    /*dir=*/15,
    /*enable=*/16,
};

/// STEP pulse generator base frequency.  One timer tick emits at most one pulse
/// per axis, so `stepper.max_step_rate_hz` must not exceed this value.
inline constexpr uint32_t kStepperTimerBaseHz = 20000;

/// Leave the drivers enabled when both axes are idle, so the motors hold their
/// position (holding torque).  Set false to disable them while stopped and save
/// power at the cost of losing position under load.
inline constexpr bool kStepperHoldWhenIdle = true;

// --- Optional quadrature encoders (CLOSED_LOOP_ENCODER mode) ----------------
//
// Not used in the default OPEN_LOOP_SIMULATION build; the pins are reserved so
// the wiring harness does not have to change when encoders are added.
inline constexpr int kLeftEncoderA = 17;
inline constexpr int kLeftEncoderB = 18;
inline constexpr int kRightEncoderA = 8;
inline constexpr int kRightEncoderB = 9;

// --- Status LED / buzzer (optional operator feedback) -----------------------
inline constexpr int kStatusLedPin = 2;  // onboard RGB LED data pin on many boards
inline constexpr int kBuzzerPin = 10;
inline constexpr bool kHasStatusLed = true;
inline constexpr bool kHasBuzzer = false;

// --- Position source --------------------------------------------------------
//
// The real UWB/EKF subsystem is expected on UART1 (or over Wi-Fi in a later
// revision).  The default development build reads mock JSON lines from the
// console UART instead, so these pins stay unused until integration.
inline constexpr int kUwbUartRxPin = 43;
inline constexpr int kUwbUartTxPin = 44;
inline constexpr uint32_t kUwbUartBaud = 115200;
