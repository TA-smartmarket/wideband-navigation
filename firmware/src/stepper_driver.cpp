#include "stepper_driver.hpp"

#include "navigation/geometry.hpp"
#include "navigation/logging.hpp"

namespace firmware {

namespace {
constexpr const char* kTag = "STEP";
StepperMotorDriver* g_stepper = nullptr;

/// Minimum STEP pulse width.  The A4988 datasheet requires >= 1 us; 5 us gives
/// margin for opto-isolated breakouts and keeps the pulse visible on a scope.
constexpr uint32_t kStepPulseWidthUs = 5;

float clampCommand(float value) {
    if (!nav::isFinite(value)) {
        return 0.0f;
    }
    return value < -1.0f ? -1.0f : (value > 1.0f ? 1.0f : value);
}
}  // namespace

bool StepperMotorDriver::begin(const nav::StepperConfig& left_config,
                               const nav::StepperConfig& right_config,
                               float wheel_base_m, float max_wheel_speed_mps,
                               const StepperPins& left_pins, const StepperPins& right_pins) {
    left_pins_ = left_pins;
    right_pins_ = right_pins;

    // Safe state first: drivers disabled (EN high) before anything else, so a
    // boot glitch cannot energise the coils.
    pinMode(left_pins_.enable, OUTPUT);
    pinMode(right_pins_.enable, OUTPUT);
    digitalWrite(left_pins_.enable, HIGH);
    digitalWrite(right_pins_.enable, HIGH);

    pinMode(left_pins_.step, OUTPUT);
    pinMode(left_pins_.dir, OUTPUT);
    pinMode(right_pins_.step, OUTPUT);
    pinMode(right_pins_.dir, OUTPUT);
    digitalWrite(left_pins_.step, LOW);
    digitalWrite(right_pins_.step, LOW);
    digitalWrite(left_pins_.dir, LOW);
    digitalWrite(right_pins_.dir, LOW);

    drive_.configure(left_config, right_config, wheel_base_m, max_wheel_speed_mps);
    max_wheel_speed_mps_ = max_wheel_speed_mps > 0.0f ? max_wheel_speed_mps : 0.70f;
    enabled_ = true;
    drivers_energised_ = false;

    NAV_LOG_INFO(kTag, "A4988 step/dir ready: L(step=%d dir=%d en=%d) R(step=%d dir=%d en=%d)",
                 left_pins_.step, left_pins_.dir, left_pins_.enable,
                 right_pins_.step, right_pins_.dir, right_pins_.enable);
    NAV_LOG_INFO(kTag, "microsteps=%ld, steps/rev=%ld, steps/m=%.1f, max rate=%.0f Hz",
                 static_cast<long>(nav::effectiveMicrosteps(left_config)),
                 static_cast<long>(nav::stepsPerRevolution(left_config)),
                 static_cast<double>(nav::stepsPerMeter(left_config)),
                 static_cast<double>(left_config.max_step_rate_hz));
    return true;
}

void StepperMotorDriver::setEnabled(bool enable) {
    if (!enabled_) {
        return;
    }
    // A4988 EN is active LOW.
    const int level = enable ? LOW : HIGH;
    digitalWrite(left_pins_.enable, level);
    digitalWrite(right_pins_.enable, level);
    drivers_energised_ = enable;
}

void StepperMotorDriver::applyCommand(float command, nav::StepperAxis& axis) {
    // Normalised command -> wheel velocity -> STEP frequency.  The axis converts
    // through stepsPerMeter, so the commanded ground speed is what is produced.
    // The normalised command is relative to the wheel-speed ceiling, which the
    // navigation core already applied when it built the command.
    const float wheel_speed_mps = clampCommand(command) * max_wheel_speed_mps_;
    axis.setTargetRateHz(nav::wheelVelocityToStepRate(wheel_speed_mps, axis.config()));
}

void StepperMotorDriver::setLeft(float command) { applyCommand(command, drive_.left()); }
void StepperMotorDriver::setRight(float command) { applyCommand(command, drive_.right()); }

void StepperMotorDriver::setBoth(float left, float right) {
    setLeft(left);
    setRight(right);
    // Energise on demand: leaving the drivers enabled while the trolley is
    // stopped wastes power and heats the motors, but disabling them loses
    // holding torque, so the behaviour is configurable.
    if (!drivers_energised_ && !idle()) {
        setEnabled(true);
    } else if (drivers_energised_ && idle() && !kStepperHoldWhenIdle) {
        setEnabled(false);
    }
}

void StepperMotorDriver::stop() {
    drive_.left().setTargetRateHz(0.0f);
    drive_.right().setTargetRateHz(0.0f);
    drive_.left().setContinuousMode();
    drive_.right().setContinuousMode();
    if (!enabled_) {
        digitalWrite(left_pins_.enable, HIGH);
        digitalWrite(right_pins_.enable, HIGH);
        return;
    }
    // Ramp the rate down through the pulse generator instead of zeroing it, so
    // the motor decelerates rather than losing steps.  The caller is expected to
    // keep calling servicePulseGenerator() until idle().
    if (!kStepperHoldWhenIdle) {
        setEnabled(false);
    }
}

void StepperMotorDriver::setRpm(float left_rpm, float right_rpm) {
    drive_.left().setContinuousMode();
    drive_.right().setContinuousMode();
    drive_.left().setTargetRpm(left_rpm);
    drive_.right().setTargetRpm(right_rpm);
    setEnabled(true);
}

void StepperMotorDriver::moveRevolutions(float left_rev, float right_rev) {
    drive_.left().moveRevolutions(left_rev);
    drive_.right().moveRevolutions(right_rev);
    setEnabled(true);
}

void StepperMotorDriver::zeroPosition() {
    drive_.left().zeroPosition();
    drive_.right().zeroPosition();
}

void StepperMotorDriver::resumeContinuous() {
    drive_.left().setContinuousMode();
    drive_.right().setContinuousMode();
}

uint64_t StepperMotorDriver::totalPulses() const {
    return drive_.left().emittedSteps() + drive_.right().emittedSteps();
}

bool StepperMotorDriver::idle() const {
    return drive_.left().currentRateHz() == 0.0f && drive_.right().currentRateHz() == 0.0f &&
           drive_.left().targetRateHz() == 0.0f && drive_.right().targetRateHz() == 0.0f &&
           !drive_.left().moving() && !drive_.right().moving();
}

void StepperMotorDriver::writeAxis(const nav::StepperAxis& axis, const StepperPins& pins,
                                   int32_t steps) {
    if (steps <= 0) {
        return;
    }
    // DIR must be stable before the first pulse of the move.
    digitalWrite(pins.dir, axis.direction() > 0 ? HIGH : LOW);
    for (int32_t i = 0; i < steps; ++i) {
        digitalWrite(pins.step, HIGH);
        delayMicroseconds(kStepPulseWidthUs);
        digitalWrite(pins.step, LOW);
        delayMicroseconds(kStepPulseWidthUs);
    }
}

bool StepperMotorDriver::servicePulseGenerator() {
    if (!enabled_) {
        return false;
    }
    const float dt_s = basePeriodS();
    const int32_t left_steps = drive_.left().advance(dt_s);
    const int32_t right_steps = drive_.right().advance(dt_s);

    if (left_steps <= 0 && right_steps <= 0) {
        return false;
    }
    // Energise before pulsing: a pulse into a disabled driver is lost motion.
    if (!drivers_energised_) {
        setEnabled(true);
    }
    writeAxis(drive_.left(), left_pins_, left_steps);
    writeAxis(drive_.right(), right_pins_, right_steps);
    return true;
}

StepperMotorDriver* stepperDriver() { return g_stepper; }

}  // namespace firmware
