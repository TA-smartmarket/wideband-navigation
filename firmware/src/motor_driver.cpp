#include "motor_driver.hpp"

#include "navigation/geometry.hpp"
#include "stepper_driver.hpp"
#include "navigation/logging.hpp"

namespace firmware {

namespace {
constexpr const char* kTag = "MOTOR";

/// Static instances: no heap use, and the navigation task always finds a valid
/// driver even if initialisation failed (in which case it is the null driver,
/// which cannot move the trolley).
HBridgeMotorDriver g_hbridge;
NullMotorDriver g_null;
SimulatedMotorDriver g_simulated;
StepperMotorDriver g_stepper;

IMotorDriver* g_active = &g_null;

/// Actuator selection/parameters, published by setup() before the driver is built.
nav::NavigationConfig g_config{};

void setMotorDriverConfigImpl(const nav::NavigationConfig& config) { g_config = config; }

float clampCommand(float value) {
    if (!nav::isFinite(value)) {
        return 0.0f;
    }
    return value < -1.0f ? -1.0f : (value > 1.0f ? 1.0f : value);
}
}  // namespace

bool HBridgeMotorDriver::begin(const MotorPins& pins) {
    pins_ = pins;

    pinMode(pins_.left_in1, OUTPUT);
    pinMode(pins_.left_in2, OUTPUT);
    pinMode(pins_.right_in1, OUTPUT);
    pinMode(pins_.right_in2, OUTPUT);

    // Safe state first: both channels braked before PWM is enabled, so a boot
    // glitch can never produce motion.
    digitalWrite(pins_.left_in1, LOW);
    digitalWrite(pins_.left_in2, LOW);
    digitalWrite(pins_.right_in1, LOW);
    digitalWrite(pins_.right_in2, LOW);

    // Arduino-ESP32 3.x replaced ledcSetup/ledcAttachPin with ledcAttach().
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    const bool left_ok = ledcAttach(pins_.left_pwm, kPwmFrequencyHz, kPwmResolutionBits);
    const bool right_ok = ledcAttach(pins_.right_pwm, kPwmFrequencyHz, kPwmResolutionBits);
#else
    ledcSetup(kLeftPwmChannel, kPwmFrequencyHz, kPwmResolutionBits);
    ledcSetup(kRightPwmChannel, kPwmFrequencyHz, kPwmResolutionBits);
    ledcAttachPin(pins_.left_pwm, kLeftPwmChannel);
    ledcAttachPin(pins_.right_pwm, kRightPwmChannel);
    const bool left_ok = true;
    const bool right_ok = true;
#endif
    if (!left_ok || !right_ok) {
        NAV_LOG_ERROR(kTag, "LEDC PWM configuration failed; motors stay disabled");
        enabled_ = false;
        stop();
        return false;
    }

    enabled_ = true;
    stop();
    NAV_LOG_INFO(kTag, "H-bridge ready: L(pwm=%d in=%d/%d) R(pwm=%d in=%d/%d)",
                 pins_.left_pwm, pins_.left_in1, pins_.left_in2, pins_.right_pwm,
                 pins_.right_in1, pins_.right_in2);
    return true;
}

float HBridgeMotorDriver::writeChannel(int pwm_channel, int in1, int in2, float command) {
    if (!enabled_) {
        return 0.0f;
    }
    const float clamped = clampCommand(command);
    const int duty = static_cast<int>(fabsf(clamped) * 1023.0f);

    if (clamped > 0.0f) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    } else if (clamped < 0.0f) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    } else {
        // Coast: both low.  Stopping by duty alone would leave the bridge in a
        // braked state that draws current and heats the driver.
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
    }
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(pwm_channel, duty);
#else
    ledcWrite(pwm_channel, duty);
#endif
    return clamped;
}

void HBridgeMotorDriver::setLeft(float command) {
    last_command_.left = writeChannel(kLeftPwmChannel, pins_.left_in1, pins_.left_in2, command);
}

void HBridgeMotorDriver::setRight(float command) {
    last_command_.right = writeChannel(kRightPwmChannel, pins_.right_in1, pins_.right_in2, command);
}

void HBridgeMotorDriver::setBoth(float left, float right) {
    // Update both channels before the PWM duty of either is meaningful: the
    // direction pins are written first so the bridge never sees a duty pulse
    // while its direction inputs are still changing.
    setLeft(left);
    setRight(right);
}

void HBridgeMotorDriver::stop() {
    last_command_.left = 0.0f;
    last_command_.right = 0.0f;
    if (!enabled_) {
        // Still drive the direction pins low when PWM failed to initialise.
        digitalWrite(pins_.left_in1, LOW);
        digitalWrite(pins_.left_in2, LOW);
        digitalWrite(pins_.right_in1, LOW);
        digitalWrite(pins_.right_in2, LOW);
        return;
    }
    writeChannel(kLeftPwmChannel, pins_.left_in1, pins_.left_in2, 0.0f);
    writeChannel(kRightPwmChannel, pins_.right_in1, pins_.right_in2, 0.0f);
}

IMotorDriver& motorDriver() { return *g_active; }

void setMotorDriverConfig(const nav::NavigationConfig& config) { setMotorDriverConfigImpl(config); }

const nav::NavigationConfig& motorDriverConfig() { return g_config; }

void initialiseMotorDriver() {
#if NAVIGATION_ENABLE_MOTOR_OUTPUT
    // The actuator is selected by `drive_kind` in the navigation configuration,
    // so a stepper build and an H-bridge build share one code path here.
    const nav::NavigationConfig& config = motorDriverConfig();
    if (config.drive_kind == nav::DriveKind::STEPPER_STEP_DIR) {
        if (g_stepper.begin(config.stepper_left, config.stepper_right,
                            config.drive.wheel_base_m, config.drive.max_wheel_speed_mps)) {
            g_active = &g_stepper;
            return;
        }
        NAV_LOG_ERROR(kTag, "stepper init failed: falling back to the null driver");
        g_active = &g_null;
        g_null.stop();
        return;
    }
    if (g_hbridge.begin(kMotorPins)) {
        g_active = &g_hbridge;
        return;
    }
    NAV_LOG_ERROR(kTag, "H-bridge init failed: falling back to the null driver");
    g_active = &g_null;
    g_null.stop();
#else
    // No hardware output requested: keep the state machine running against a
    // backend that cannot move anything.
    g_active = &g_simulated;
    g_simulated.stop();
    NAV_LOG_WARN(kTag, "motor output disabled at build time (%s driver active)",
                 g_active->name());
#endif
}

}  // namespace firmware
