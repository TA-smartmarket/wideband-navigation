// Stepper motor control: step-rate conversion, position tracking and
// acceleration profiling.
//
// Why a dedicated module: a stepper driven by an A4988-class driver is not a
// PWM device.  Speed is the STEP pulse frequency, direction is a logic level, and
// position is the *count* of pulses.  None of that is expressible as a duty
// cycle, so the differential-drive command path needs an explicit conversion.
//
// The maths and the state machine live here (portable C++17, no hardware) so the
// step generation logic is unit-tested on the host; the firmware ISR only calls
// StepperAxis::advance() at a fixed base frequency.
#pragma once

#include <cstdint>

#include "navigation/geometry.hpp"

namespace nav {

struct StepperConfig {
    /// Full steps per motor revolution (NEMA 17 is 200, i.e. 1.8 deg/step).
    int32_t steps_per_revolution{200};

    /// Microstepping divisor set by the A4988 MS1/MS2/MS3 pins:
    /// 1, 2, 4, 8 or 16.
    int32_t microsteps{16};

    /// Wheel radius, used to relate motor revolutions to ground distance.
    float wheel_radius_m{0.05f};

    /// Highest STEP frequency the driver/ISR may emit, per axis.  The A4988 needs
    /// a minimum 1 us pulse, so the electrical limit is ~500 kHz; the practical
    /// limit is the pulse generator, so this defaults well below it.
    float max_step_rate_hz{20000.0f};

    /// Below this frequency the axis is treated as stopped (avoids emitting
    /// single stray pulses from a nearly-zero command).
    float min_step_rate_hz{2.0f};

    /// Maximum change of STEP frequency per second.  A stepper that is commanded
    /// to accelerate faster than the torque allows simply loses steps, so this is
    /// the single most important parameter for reliable motion.
    float max_step_accel_hz_per_s{20000.0f};

    /// Invert the DIR level for this axis (mechanical mounting orientation).
    bool invert_direction{false};
};

/// Result of validating a stepper configuration.
struct StepperValidation {
    bool valid{true};
    const char* message{"ok"};

    void fail(const char* reason) {
        valid = false;
        message = reason;
    }
};

StepperValidation validateStepperConfig(const StepperConfig& config);

/// Microsteps actually applied per full step (always >= 1).
inline int32_t effectiveMicrosteps(const StepperConfig& config) {
    return config.microsteps < 1 ? 1 : config.microsteps;
}

/// STEP pulses per motor revolution, including microstepping.
inline int32_t stepsPerRevolution(const StepperConfig& config) {
    const int32_t base = config.steps_per_revolution < 1 ? 200 : config.steps_per_revolution;
    return base * effectiveMicrosteps(config);
}

/// STEP pulses per meter of ground travel.
float stepsPerMeter(const StepperConfig& config);

/// Motor RPM produced by a STEP frequency.
float stepRateToRpm(float step_rate_hz, const StepperConfig& config);

/// STEP frequency required for a motor RPM.
float rpmToStepRate(float rpm, const StepperConfig& config);

/// STEP frequency required for a wheel linear velocity (m/s).
float wheelVelocityToStepRate(float velocity_mps, const StepperConfig& config);

/// Wheel linear velocity produced by a STEP frequency.
float stepRateToWheelVelocity(float step_rate_hz, const StepperConfig& config);

/// Revolutions corresponding to a step count.
float stepsToRevolutions(int64_t steps, const StepperConfig& config);

/// Ground distance corresponding to a step count.
float stepsToDistanceM(int64_t steps, const StepperConfig& config);

/// Operating mode of one axis.
enum class StepperMode {
    /// Follow a commanded STEP frequency (used by the navigation PID).
    CONTINUOUS = 0,
    /// Drive to an absolute step position with a trapezoidal profile.
    POSITION,
};

/// One stepper axis: rate limiting, step generation and position accounting.
///
/// The caller drives it with a fixed time step:
///   * firmware: the step-generator ISR calls advance() every base period;
///   * simulator/tests: the control loop calls advance(dt_s).
/// The phase accumulator makes the average STEP frequency exact regardless of how
/// the requested rate divides into the base period (fractional steps carry over),
/// which is what keeps a commanded 0.45 m/s actually mean 0.45 m/s.
class StepperAxis {
public:
    void configure(const StepperConfig& config);

    /// Clear position, rate and phase (keeps the configuration).
    void reset();

    /// Declare the current physical position as step 0.
    void zeroPosition() { position_steps_ = 0; }

    void setPositionSteps(int64_t steps) { position_steps_ = steps; }

    // ---- continuous (velocity) mode ------------------------------------
    /// Command a signed STEP frequency.  The sign selects the direction; the
    /// magnitude is clamped to max_step_rate_hz.
    void setTargetRateHz(float rate_hz);

    /// Command a signed motor RPM.
    void setTargetRpm(float rpm);

    // ---- position (move) mode ------------------------------------------
    /// Drive to an absolute step position using a trapezoidal profile that
    /// decelerates in time to stop exactly on target.
    void setTargetPositionSteps(int64_t steps);

    /// Relative move expressed in motor revolutions.
    void moveRevolutions(float revolutions);

    /// Return to CONTINUOUS mode (cancels any pending move).
    void setContinuousMode();

    StepperMode mode() const { return mode_; }
    bool positionReached() const;

    // ---- execution ------------------------------------------------------
    /// Advance by `dt_s`.  Returns the number of STEP pulses to emit (always
    /// >= 0; the direction is available from direction()).
    int32_t advance(float dt_s);

    // ---- observation ----------------------------------------------------
    int64_t positionSteps() const { return position_steps_; }
    int64_t targetPositionSteps() const { return target_position_steps_; }
    float revolutions() const { return stepsToRevolutions(position_steps_, config_); }
    float distanceM() const { return stepsToDistanceM(position_steps_, config_); }

    /// Signed STEP frequency currently being emitted.
    float currentRateHz() const { return current_rate_hz_; }
    float targetRateHz() const { return target_rate_hz_; }
    float currentRpm() const { return stepRateToRpm(current_rate_hz_, config_); }
    float targetRpm() const { return stepRateToRpm(target_rate_hz_, config_); }

    /// -1, 0 or +1 (as driven on the DIR pin, after inversion).
    int direction() const { return direction_; }
    bool atTargetRate() const { return current_rate_hz_ == target_rate_hz_; }
    bool moving() const { return direction_ != 0; }

    /// Accumulated STEP pulses emitted since the last reset (diagnostics).
    uint64_t emittedSteps() const { return emitted_steps_; }

    const StepperConfig& config() const { return config_; }

private:
    /// Rate the axis may not exceed if it is to stop on target.
    float stoppingRateLimit() const;
    /// Apply the acceleration limit towards `desired`.
    void rampTowards(float desired, float dt_s);

    StepperConfig config_{};
    StepperMode mode_{StepperMode::CONTINUOUS};

    int64_t position_steps_{0};
    int64_t target_position_steps_{0};

    float current_rate_hz_{0.0f};
    float target_rate_hz_{0.0f};
    /// Magnitude of the rate the position profile is heading for.
    float position_rate_hz_{0.0f};

    int direction_{0};
    /// Fractional step carried between advances (DDS phase accumulator).
    float phase_{0.0f};
    uint64_t emitted_steps_{0};
};

/// Differential-drive stepper pair: converts a body twist into two axes.
///
/// This is the stepper counterpart of twistToMotorCommand(): it keeps the PID and
/// the path follower unchanged and puts the stepper-specific conversion in one
/// place.
class StepperDrive {
public:
    void configure(const StepperConfig& left, const StepperConfig& right,
                   float wheel_base_m, float max_wheel_speed_mps);

    void reset();

    /// Command a body twist (m/s, rad/s) for `dt_s`; returns the two STEP pulse
    /// counts to emit this cycle.
    void commandTwist(float linear_mps, float angular_radps, float dt_s);

    StepperAxis& left() { return left_; }
    StepperAxis& right() { return right_; }
    const StepperAxis& left() const { return left_; }
    const StepperAxis& right() const { return right_; }

    /// Average of the two wheel distances (odometry from step counting).
    float odometryDistanceM() const;
    /// Yaw change estimated from the difference of the two wheel distances.
    float odometryYawRad() const;

private:
    StepperAxis left_{};
    StepperAxis right_{};
    float wheel_base_m_{0.32f};
    float max_wheel_speed_mps_{0.70f};
};

}  // namespace nav
