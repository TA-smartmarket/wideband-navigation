#include "stepper_driver.hpp"

#include "navigation/geometry.hpp"
#include "navigation/logging.hpp"

#include <driver/gpio.h>

namespace firmware {

namespace {
constexpr const char* kTag = "STEP";
StepperMotorDriver* g_stepper = nullptr;

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

    // Safe state first: drivers disabled before anything else, so a
    // boot glitch cannot energise the coils.
    pinMode(left_pins_.enable, OUTPUT);
    pinMode(right_pins_.enable, OUTPUT);
    digitalWrite(left_pins_.enable, kStepperEnableActiveLow ? HIGH : LOW);
    digitalWrite(right_pins_.enable, kStepperEnableActiveLow ? HIGH : LOW);

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
    g_stepper = this;
    startPulseTimer();

    NAV_LOG_INFO(kTag, "TB6600 step/dir ready: L(step=%d dir=%d en=%d) R(step=%d dir=%d en=%d)",
                 left_pins_.step, left_pins_.dir, left_pins_.enable,
                 right_pins_.step, right_pins_.dir, right_pins_.enable);
    NAV_LOG_INFO(kTag, "microsteps=%ld, steps/rev=%ld, steps/m=%.1f, max rate=%.0f Hz",
                 static_cast<long>(nav::effectiveMicrosteps(left_config)),
                 static_cast<long>(nav::stepsPerRevolution(left_config)),
                 static_cast<double>(nav::stepsPerMeter(left_config)),
                 static_cast<double>(left_config.max_step_rate_hz));
    return enabled_;
}

void StepperMotorDriver::startPulseTimer() {
    // A STEP is generated over two timer edges.  At the configured 20 kHz
    // maximum rate, the fixed high phase is 25 us: safe for TB6600 optocoupler
    // inputs and independent from FreeRTOS scheduling or serial traffic.
    pulse_timer_ = timerBegin(kStepperTimerBaseHz * 2u);
    if (pulse_timer_ == nullptr) {
        enabled_ = false;
        NAV_LOG_ERROR(kTag, "cannot allocate STEP hardware timer");
        return;
    }
    timerAttachInterrupt(pulse_timer_, &StepperMotorDriver::timerThunk);
    timerAlarm(pulse_timer_, 1, true, 0);
}

void IRAM_ATTR StepperMotorDriver::timerThunk() {
    if (g_stepper != nullptr) {
        g_stepper->onTimerTick();
    }
}

void IRAM_ATTR StepperMotorDriver::onTimerTick() {
    if (!enabled_) return;
    if (pulse_high_) {
        if (left_pulse_active_) gpio_set_level(static_cast<gpio_num_t>(left_pins_.step), 0);
        if (right_pulse_active_) gpio_set_level(static_cast<gpio_num_t>(right_pins_.step), 0);
        pulse_high_ = false;
        left_pulse_active_ = false;
        right_pulse_active_ = false;
        return;
    }
    const int32_t left_steps = drive_.left().advance(basePeriodS());
    const int32_t right_steps = drive_.right().advance(basePeriodS());
    if ((left_steps <= 0 && right_steps <= 0) || !drivers_energised_) return;
    if (left_steps > 0) {
        gpio_set_level(static_cast<gpio_num_t>(left_pins_.dir), drive_.left().direction() > 0 ? 1 : 0);
        gpio_set_level(static_cast<gpio_num_t>(left_pins_.step), 1);
        left_pulse_active_ = true;
    }
    if (right_steps > 0) {
        gpio_set_level(static_cast<gpio_num_t>(right_pins_.dir), drive_.right().direction() > 0 ? 1 : 0);
        gpio_set_level(static_cast<gpio_num_t>(right_pins_.step), 1);
        right_pulse_active_ = true;
    }
    pulse_high_ = left_pulse_active_ || right_pulse_active_;
}

void StepperMotorDriver::setEnabled(bool enable) {
    if (!enabled_) {
        return;
    }
    const int level = enable ? (kStepperEnableActiveLow ? LOW : HIGH)
                             : (kStepperEnableActiveLow ? HIGH : LOW);
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
        digitalWrite(left_pins_.enable, kStepperEnableActiveLow ? HIGH : LOW);
        digitalWrite(right_pins_.enable, kStepperEnableActiveLow ? HIGH : LOW);
        return;
    }
    // Ramp the rate down through the pulse generator instead of zeroing it, so
    // the motor decelerates rather than losing steps.  The caller is expected to
    // keep calling servicePulseGenerator() until idle().
    if (!kStepperHoldWhenIdle) {
        setEnabled(false);
    }
}

void StepperMotorDriver::emergencyStop() {
    drive_.left().setTargetRateHz(0.0f);
    drive_.right().setTargetRateHz(0.0f);
    setEnabled(false);
    digitalWrite(left_pins_.step, LOW);
    digitalWrite(right_pins_.step, LOW);
    pulse_high_ = false;
    left_pulse_active_ = false;
    right_pulse_active_ = false;
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

bool StepperMotorDriver::servicePulseGenerator() {
    return pulse_high_;
}

StepperMotorDriver* stepperDriver() { return g_stepper; }

}  // namespace firmware
