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

// --- TB6600 STEP / DIR / ENABLE interface -----------------------------------
//
// Each TB6600-class driver needs three isolated logic inputs:
//
//   STEP : one pulse per microstep; frequency sets the speed
//   DIR  : logic level selects the direction (set before the pulse train)
//   EN   : enable polarity depends on the actual TB6600 wiring
//
// IMPORTANT: GPIO numbers and enable polarity below are placeholders retained
// only to keep a compileable default.  They MUST be verified against the real
// ESP32-S3 board, the TB6600 PUL/DIR/ENA wiring, and any level-shifting circuit
// before power is applied to a motor.  Do not use the values as a wiring guide.

struct StepperPins {
    int step;
    int dir;
    int enable;  // polarity is configured by kStepperEnableActiveLow
};

/// TODO: VERIFY WITH REAL HARDWARE.  Centralised placeholder assignments.
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

/// TODO: VERIFY WITH REAL HARDWARE.  Set according to the chosen TB6600 input
/// wiring; many boards are wired active-low, but that is not universal.
inline constexpr bool kStepperEnableActiveLow = true;

/// STEP pulse generator base frequency.  One timer tick emits at most one pulse
/// per axis, so `stepper.max_step_rate_hz` must not exceed this value.
inline constexpr uint32_t kStepperTimerBaseHz = 20000;

/// Leave the drivers enabled when both axes are idle, so the motors hold their
/// position.  Holding torque is not equivalent to available moving torque.
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
