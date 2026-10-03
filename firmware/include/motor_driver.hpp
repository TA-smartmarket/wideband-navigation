// Motor driver abstraction for the firmware.
//
// The navigation core only ever produces normalised commands in [-1, +1]; every
// hardware detail (H-bridge truth table, LEDC channels, dead-time, direction
// inversion) lives behind this interface.  That keeps the control law identical
// between the simulator, a bench rig with no motors, and the final trolley.
#pragma once

#include <Arduino.h>

#include "app_config.hpp"
#include "navigation/differential_drive.hpp"
#include "navigation/geometry.hpp"
#include "navigation/navigation_config.hpp"
#include "navigation/types.hpp"
#include "pin_config.hpp"

namespace firmware {

/// Contract implemented by every motor backend.
class IMotorDriver {
public:
    virtual ~IMotorDriver() = default;

    /// Apply a normalised command in [-1, +1] to each wheel.
    virtual void setLeft(float command) = 0;
    virtual void setRight(float command) = 0;

    /// Apply both commands in one call (avoids a half-updated chassis state).
    virtual void setBoth(float left, float right) {
        setLeft(left);
        setRight(right);
    }

    /// Cut power to both channels.  Must be safe to call at any time.
    virtual void stop() = 0;

    /// True when this backend can actually drive hardware.
    virtual bool drivesHardware() const = 0;

    /// Backend name for telemetry/logging.
    virtual const char* name() const = 0;

    /// Last commanded values, for telemetry.
    nav::MotorCommand lastCommand() const { return last_command_; }

protected:
    nav::MotorCommand last_command_{};
};

/// H-bridge driver using the ESP32 LEDC peripheral.
///
/// Truth table (per channel):
///   command > 0 : IN1 = 1, IN2 = 0, PWM duty = |command|
///   command < 0 : IN1 = 0, IN2 = 1, PWM duty = |command|
///   command = 0 : IN1 = 0, IN2 = 0, PWM duty = 0   (coast)
class HBridgeMotorDriver : public IMotorDriver {
public:
    HBridgeMotorDriver() = default;

    /// Configure pins, LEDC channels and brake the motors.  Returns false when
    /// the LEDC peripheral could not be configured.
    bool begin(const MotorPins& pins = kMotorPins);

    void setLeft(float command) override;
    void setRight(float command) override;
    void setBoth(float left, float right) override;
    void stop() override;
    bool drivesHardware() const override { return enabled_; }
    const char* name() const override { return "h-bridge-ledc"; }

private:
    /// Write one channel.  Returns the normalised magnitude actually applied.
    float writeChannel(int pwm_channel, int in1, int in2, float command);

    MotorPins pins_{};
    bool enabled_{false};
};

/// Backend that discards every command (bench tests without motors).
///
/// The navigation state machine runs unmodified, so motor *logic* is still
/// exercised; only the electrical output is absent.
class NullMotorDriver : public IMotorDriver {
public:
    void setLeft(float command) override {
        last_command_.left = clampCommand(command);
    }
    void setRight(float command) override {
        last_command_.right = clampCommand(command);
    }
    void stop() override {
        last_command_.left = 0.0f;
        last_command_.right = 0.0f;
    }
    bool drivesHardware() const override { return false; }
    const char* name() const override { return "null"; }

private:
    static float clampCommand(float value) {
        if (!nav::isFinite(value)) {
            return 0.0f;
        }
        return value < -1.0f ? -1.0f : (value > 1.0f ? 1.0f : value);
    }
};

/// Records commands and can replay a scripted response (hardware-in-the-loop
/// tests where the real motor must not spin).
class SimulatedMotorDriver : public IMotorDriver {
public:
    void setLeft(float command) override { last_command_.left = clampCommand(command); }
    void setRight(float command) override { last_command_.right = clampCommand(command); }
    void stop() override {
        last_command_.left = 0.0f;
        last_command_.right = 0.0f;
    }
    bool drivesHardware() const override { return false; }
    const char* name() const override { return "simulated"; }

    /// Peak absolute command seen since the last reset (safety assertions).
    float peakCommand() const {
        const float left = last_command_.left < 0 ? -last_command_.left : last_command_.left;
        const float right = last_command_.right < 0 ? -last_command_.right : last_command_.right;
        return left > right ? left : right;
    }

private:
    static float clampCommand(float value) {
        if (!nav::isFinite(value)) {
            return 0.0f;
        }
        return value < -1.0f ? -1.0f : (value > 1.0f ? 1.0f : value);
    }
};

/// Global motor driver instance used by the navigation task.
IMotorDriver& motorDriver();

/// Navigation configuration that selects and parameterises the actuator.  Set by
/// the firmware once the configuration has been loaded, so the driver factory
/// does not need to re-parse it.
void setMotorDriverConfig(const nav::NavigationConfig& config);
const nav::NavigationConfig& motorDriverConfig();

/// Select the backend at start-up according to app_config.hpp.
void initialiseMotorDriver();

}  // namespace firmware
