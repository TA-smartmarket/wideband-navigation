// Stepper motor driver for NEMA 17 motors on A4988-class step/dir drivers.
//
// The A4988 does not accept a PWM duty cycle: it needs a STEP pulse train whose
// *frequency* sets the speed, plus a DIR level for the direction.  The conversion
// from the navigation twist to step rates, the acceleration profile and the step
// counting all live in the portable core (navigation/stepper.hpp); this file only
// owns the electrical interface and the timer ISR.
//
// Position is inherently known: the number of STEP pulses emitted *is* the
// rotation, so `positionSteps()` is a real measurement of commanded motion (not
// a guarantee that the motor did not stall).
#pragma once

#include <Arduino.h>

#include "app_config.hpp"
#include "motor_driver.hpp"
#include "navigation/navigation_config.hpp"
#include "navigation/stepper.hpp"
#include "pin_config.hpp"

namespace firmware {

/// Step/dir driver for the two trolley axes.
///
/// Implements IMotorDriver so the navigation task is unchanged: `setLeft`/
/// `setRight` receive normalised commands in [-1, +1] and are converted into
/// signed STEP frequencies.
class StepperMotorDriver : public IMotorDriver {
public:
    bool begin(const nav::StepperConfig& left_config, const nav::StepperConfig& right_config,
               float wheel_base_m, float max_wheel_speed_mps,
               const StepperPins& left_pins = kStepperPinsLeft,
               const StepperPins& right_pins = kStepperPinsRight);

    void setLeft(float command) override;
    void setRight(float command) override;
    void setBoth(float left, float right) override;
    void stop() override;
    bool drivesHardware() const override { return enabled_; }
    const char* name() const override { return "stepper-a4988"; }

    // ---- manual control (bench / calibration) --------------------------
    /// Set a signed motor RPM directly, bypassing the normalised commands.
    void setRpm(float left_rpm, float right_rpm);

    /// Move each axis by a number of motor revolutions (trapezoidal profile).
    void moveRevolutions(float left_rev, float right_rev);

    /// Declare the current position as zero on both axes.
    void zeroPosition();

    /// Return to velocity mode, cancelling any pending move.
    void resumeContinuous();

    // ---- observation ----------------------------------------------------
    nav::StepperAxis& leftAxis() { return drive_.left(); }
    nav::StepperAxis& rightAxis() { return drive_.right(); }
    const nav::StepperDrive& drive() const { return drive_; }

    /// Total STEP pulses emitted since start-up (both axes).
    uint64_t totalPulses() const;

    /// True when both axes are commanded to zero.
    bool idle() const;

    /// Called from the STEP timer ISR: advance the profiles and emit pulses.
    /// Returns true when at least one pulse was produced.
    bool servicePulseGenerator();

    /// Base period of the pulse generator, in seconds.
    static constexpr float basePeriodS() {
        return 1.0f / static_cast<float>(kStepperTimerBaseHz);
    }

private:
    void writeAxis(const nav::StepperAxis& axis, const StepperPins& pins, int32_t steps);
    void setEnabled(bool enabled);
    void applyCommand(float command, nav::StepperAxis& axis);

    nav::StepperDrive drive_{};
    /// Wheel-speed ceiling the normalised commands are relative to.
    float max_wheel_speed_mps_{0.70f};
    StepperPins left_pins_{};
    StepperPins right_pins_{};
    bool enabled_{false};
    bool drivers_energised_{false};
    uint32_t overrun_count_{0};
};

/// Global stepper driver instance (valid once initialiseMotorDriver() has run).
StepperMotorDriver* stepperDriver();

}  // namespace firmware
