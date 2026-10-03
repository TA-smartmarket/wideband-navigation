#include "navigation/pid.hpp"

#include "navigation/geometry.hpp"

namespace nav {

namespace {
/// Guard band around dt: anything outside is treated as a scheduling glitch.
constexpr float kMinDt = 1.0e-4f;
constexpr float kMaxDt = 1.0f;
}  // namespace

PIDController::PIDController(float kp, float ki, float kd) {
    config_.kp = kp;
    config_.ki = ki;
    config_.kd = kd;
}

float PIDController::sanitizeDt(float dt_s) const {
    if (!isFinite(dt_s) || dt_s < kMinDt || dt_s > kMaxDt) {
        // A zero/negative/absurd dt would divide by ~0 and produce a huge
        // derivative; fall back to the nominal sample time instead.
        return config_.nominal_dt_s > kMinDt ? config_.nominal_dt_s : kMinDt;
    }
    return dt_s;
}

float PIDController::update(float setpoint, float measurement, float dt_s) {
    if (!isFinite(setpoint)) {
        setpoint = 0.0f;
    }
    if (!isFinite(measurement)) {
        // A non-finite measurement must not corrupt the state; reuse the last
        // known measurement and keep the integral unchanged.
        measurement = previous_measurement_;
    }

    const float dt = sanitizeDt(dt_s);
    const float error = setpoint - measurement;

    p_term_ = config_.kp * error;

    // Derivative on measurement (negative sign because it opposes the error
    // growth) with a first-order low-pass filter.
    float derivative_raw = 0.0f;
    if (has_previous_) {
        derivative_raw = -(measurement - previous_measurement_) / dt;
    }
    const float alpha = clampf(config_.derivative_filter_alpha, 0.0f, 1.0f);
    filtered_derivative_ = alpha * derivative_raw + (1.0f - alpha) * filtered_derivative_;
    d_term_ = config_.kd * filtered_derivative_;

    // Integral candidate, clamped to the configured integral window.
    const float integral_candidate =
        clampf(integral_ + config_.ki * error * dt, config_.integral_min, config_.integral_max);
    const float unsaturated = p_term_ + integral_candidate + d_term_;

    // Anti-windup by integrator clamping (back-calculation): when the sum
    // saturates, the integrator is rewritten so that p + i + d equals the
    // saturated output exactly.  This keeps the first response to a large error
    // immediate (unlike conditional integration, which would refuse to
    // accumulate at all and could output zero) while still preventing the
    // integral from growing past the actuator limit.
    float output = clampf(unsaturated, config_.output_min, config_.output_max);
    if (output != unsaturated) {
        integral_ = clampf(output - p_term_ - d_term_, config_.integral_min, config_.integral_max);
    } else {
        integral_ = integral_candidate;
    }
    i_term_ = integral_;

    if (!isFinite(output)) {
        output = 0.0f;
        reset();
    }

    previous_measurement_ = measurement;
    has_previous_ = true;
    last_output_ = output;
    return output;
}

void PIDController::reset() {
    integral_ = 0.0f;
    filtered_derivative_ = 0.0f;
    previous_measurement_ = 0.0f;
    has_previous_ = false;
    p_term_ = 0.0f;
    i_term_ = 0.0f;
    d_term_ = 0.0f;
    last_output_ = 0.0f;
}

}  // namespace nav
