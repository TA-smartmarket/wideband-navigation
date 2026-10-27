#include "navigation/wheel_velocity_controller.hpp"

#include <cmath>

#include "navigation/geometry.hpp"

namespace nav {

bool WheelVelocityController::configure(const WheelVelocityControlConfig& config) {
    if (!isFinite(config.max_abs_rpm) || config.max_abs_rpm <= 0.0f ||
        !isFinite(config.zero_deadband_rpm) || config.zero_deadband_rpm < 0.0f ||
        !isFinite(config.tracking_error_rpm) || config.tracking_error_rpm <= 0.0f ||
        config.tracking_error_timeout_ms == 0 ||
        !isFinite(config.pid.kp) || !isFinite(config.pid.ki) || !isFinite(config.pid.kd)) {
        return false;
    }
    config_ = config;
    PIDConfig pid_config = config.pid;
    pid_config.circular = false;
    pid_config.output_min = -config.max_abs_rpm;
    pid_config.output_max = config.max_abs_rpm;
    pid_.setConfig(pid_config);
    reset();
    return true;
}

void WheelVelocityController::reset() {
    pid_.reset();
    status_ = WheelVelocityControlStatus{};
    excessive_error_since_ms_ = 0;
}

float WheelVelocityController::update(float target_rpm, float measured_rpm,
                                      float dt_s, uint64_t now_ms,
                                      bool feedback_healthy) {
    if (!feedback_healthy || !isFinite(measured_rpm)) {
        reset();
        status_.tracking_fault = true;
        return 0.0f;
    }
    target_rpm = isFinite(target_rpm)
                     ? clampf(target_rpm, -config_.max_abs_rpm, config_.max_abs_rpm)
                     : 0.0f;
    if (std::fabs(target_rpm) <= config_.zero_deadband_rpm) {
        reset();
        status_.measured_rpm = measured_rpm;
        return 0.0f;
    }

    // Feed-forward is the requested RPM; PID contributes a bounded correction.
    const float correction = pid_.update(target_rpm, measured_rpm, dt_s);
    const float unsaturated = target_rpm + correction;
    const float output = clampf(unsaturated, -config_.max_abs_rpm, config_.max_abs_rpm);
    status_.target_rpm = target_rpm;
    status_.measured_rpm = measured_rpm;
    status_.error_rpm = target_rpm - measured_rpm;
    status_.output_rpm = output;
    status_.p_term = pid_.proportionalTerm();
    status_.i_term = pid_.integralTerm();
    status_.d_term = pid_.derivativeTerm();
    status_.saturated = output != unsaturated;

    if (std::fabs(status_.error_rpm) > config_.tracking_error_rpm) {
        if (excessive_error_since_ms_ == 0) excessive_error_since_ms_ = now_ms;
        status_.tracking_fault = now_ms >= excessive_error_since_ms_ &&
            now_ms - excessive_error_since_ms_ >= config_.tracking_error_timeout_ms;
    } else {
        excessive_error_since_ms_ = 0;
        status_.tracking_fault = false;
    }
    return output;
}

}  // namespace nav
