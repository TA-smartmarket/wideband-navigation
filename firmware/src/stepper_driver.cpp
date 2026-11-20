#include "stepper_driver.hpp"

#include "navigation/geometry.hpp"
#include "navigation/logging.hpp"

#include <cmath>
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
#if NAVIGATION_ENABLE_STEPPER_TIMER
    startPulseTimer();
#else
    NAV_LOG_WARN(kTag, "STEP timer disabled: monitor-only build; TB6600 outputs remain inhibited");
#endif

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
    // PlatformIO pins Arduino-ESP32 2.0.x, whose timer API takes a timer
    // number and an APB-clock divider.  A divider of 80 produces a 1 MHz
    // counter from the ESP32's 80 MHz APB clock.  The ISR runs at twice the
    // configured maximum STEP rate because one interrupt raises STEP and the
    // next lowers it.
    constexpr uint32_t kTimerCounterHz = 1000000u;
    constexpr uint32_t kInterruptRateHz = kStepperTimerBaseHz * 2u;
    static_assert(kTimerCounterHz % kInterruptRateHz == 0,
                  "STEP timer period must be an integer number of ticks");
    constexpr uint64_t kAlarmTicks = kTimerCounterHz / kInterruptRateHz;

    pulse_timer_ = timerBegin(/*timer=*/0, /*divider=*/80, /*countUp=*/true);
    if (pulse_timer_ == nullptr) {
        enabled_ = false;
        NAV_LOG_ERROR(kTag, "cannot allocate STEP hardware timer");
        return;
    }
    timerAttachInterrupt(pulse_timer_, &StepperMotorDriver::timerThunk,
                         /*edge=*/true);
    timerAlarmWrite(pulse_timer_, kAlarmTicks, /*autoReload=*/true);
    timerAlarmEnable(pulse_timer_);
}

void IRAM_ATTR StepperMotorDriver::timerThunk() {
    if (g_stepper != nullptr) {
        g_stepper->onTimerTick();
    }
}

void IRAM_ATTR StepperMotorDriver::onTimerTick() {
    if (!enabled_) return;
    // The 40 kHz interrupt alternates between a rising-edge scheduling slot
    // and a falling-edge slot. This guarantees a fixed 25 us HIGH pulse and a
    // 20 kHz maximum pulse rate without emitting 50 ms batches.
    if (!timer_rising_slot_) {
        if (left_pulse_active_) {
            gpio_set_level(static_cast<gpio_num_t>(left_pins_.step), 0);
            left_pulse_active_ = false;
        }
        if (right_pulse_active_) {
            gpio_set_level(static_cast<gpio_num_t>(right_pins_.step), 0);
            right_pulse_active_ = false;
        }
        pulse_high_ = false;
        timer_rising_slot_ = true;
        return;
    }
    timer_rising_slot_ = false;

    bool left_step = false;
    bool right_step = false;
    constexpr uint32_t kPhaseThreshold = kStepperTimerBaseHz * 1000u;
    portENTER_CRITICAL_ISR(&pulse_mux_);
    if (drivers_energised_ && left_rate_millihz_ > 0) {
        if (!left_direction_ready_) {
            gpio_set_level(static_cast<gpio_num_t>(left_pins_.dir),
                           left_pending_direction_ > 0 ? 1 : 0);
            left_direction_ready_ = true;
            left_phase_millihz_ = 0;
        } else {
            left_phase_millihz_ += left_rate_millihz_;
            if (left_phase_millihz_ >= kPhaseThreshold) {
                left_phase_millihz_ -= kPhaseThreshold;
                left_step = true;
            }
        }
    } else {
        left_phase_millihz_ = 0;
        left_direction_ready_ = false;
    }
    if (drivers_energised_ && right_rate_millihz_ > 0) {
        if (!right_direction_ready_) {
            gpio_set_level(static_cast<gpio_num_t>(right_pins_.dir),
                           right_pending_direction_ > 0 ? 1 : 0);
            right_direction_ready_ = true;
            right_phase_millihz_ = 0;
        } else {
            right_phase_millihz_ += right_rate_millihz_;
            if (right_phase_millihz_ >= kPhaseThreshold) {
                right_phase_millihz_ -= kPhaseThreshold;
                right_step = true;
            }
        }
    } else {
        right_phase_millihz_ = 0;
        right_direction_ready_ = false;
    }
    portEXIT_CRITICAL_ISR(&pulse_mux_);
    if (left_step) {
        gpio_set_level(static_cast<gpio_num_t>(left_pins_.step), 1);
        left_pulse_active_ = true;
        ++left_hardware_pulses_;
    }
    if (right_step) {
        gpio_set_level(static_cast<gpio_num_t>(right_pins_.step), 1);
        right_pulse_active_ = true;
        ++right_hardware_pulses_;
    }
    pulse_high_ = left_pulse_active_ || right_pulse_active_;
}

void StepperMotorDriver::planPulseGenerator(float dt_s) {
    if (!enabled_ || !nav::isFinite(dt_s) || dt_s <= 0.0f) return;
    (void)drive_.left().advance(dt_s);
    (void)drive_.right().advance(dt_s);
    const auto rateMilliHz = [](float rate_hz) -> uint32_t {
        const float magnitude = std::fabs(rate_hz);
        if (!nav::isFinite(magnitude) || magnitude <= 0.0f) return 0;
        const float bounded = nav::clampf(
            magnitude, 0.0f, static_cast<float>(kStepperTimerBaseHz));
        return static_cast<uint32_t>(bounded * 1000.0f + 0.5f);
    };
    const int8_t left_direction = static_cast<int8_t>(drive_.left().direction());
    const int8_t right_direction = static_cast<int8_t>(drive_.right().direction());
    const uint32_t left_rate = rateMilliHz(drive_.left().currentRateHz());
    const uint32_t right_rate = rateMilliHz(drive_.right().currentRateHz());
    portENTER_CRITICAL(&pulse_mux_);
    if (left_direction != left_pending_direction_) {
        left_pending_direction_ = left_direction;
        left_direction_ready_ = false;
    }
    if (right_direction != right_pending_direction_) {
        right_pending_direction_ = right_direction;
        right_direction_ready_ = false;
    }
    left_rate_millihz_ = left_rate;
    right_rate_millihz_ = right_rate;
    portEXIT_CRITICAL(&pulse_mux_);
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
    portENTER_CRITICAL(&pulse_mux_);
    left_rate_millihz_ = 0;
    right_rate_millihz_ = 0;
    left_phase_millihz_ = 0;
    right_phase_millihz_ = 0;
    left_direction_ready_ = false;
    right_direction_ready_ = false;
    portEXIT_CRITICAL(&pulse_mux_);
    pulse_high_ = false;
    left_pulse_active_ = false;
    right_pulse_active_ = false;
}

void StepperMotorDriver::setRpm(float left_rpm, float right_rpm) {
    drive_.left().setContinuousMode();
    drive_.right().setContinuousMode();
    drive_.left().setTargetRpm(left_rpm);
    drive_.right().setTargetRpm(right_rpm);
#if NAVIGATION_ENABLE_STEPPER_TIMER
    setEnabled(true);
#else
    // Never energise coils in a monitor-only image: a non-pulsing enabled
    // driver can heat the motors while providing no controllable motion.
    setEnabled(false);
#endif
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
