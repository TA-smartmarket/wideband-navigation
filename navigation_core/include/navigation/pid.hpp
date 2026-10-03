// Reusable PID controller with anti-windup and a filtered derivative.
//
// Used for the heading loop of the trolley.  The derivative is computed on the
// measurement (not on the error) and low-pass filtered, which avoids the
// "derivative kick" caused by a step change of the setpoint and limits noise
// amplification from UWB-derived headings.
#pragma once

#include <cstdint>

namespace nav {

struct PIDConfig {
    float kp{1.0f};
    float ki{0.0f};
    float kd{0.0f};

    /// Symmetric output saturation applied after the sum of the terms.
    float output_min{-1.0f};
    float output_max{1.0f};

    /// Symmetric clamp on the integral accumulator (anti-windup).
    float integral_min{-1.0f};
    float integral_max{1.0f};

    /// Derivative low-pass coefficient in [0, 1]; 1.0 disables filtering.
    float derivative_filter_alpha{0.25f};

    /// Nominal sample time; used to reject absurd dt values.
    float nominal_dt_s{0.05f};
};

class PIDController {
public:
    PIDController() = default;
    explicit PIDController(const PIDConfig& config) : config_(config) {}
    PIDController(float kp, float ki, float kd);

    /// Feed one sample.  `dt_s` is the elapsed time since the previous call.
    /// Non-finite or out-of-range dt is clamped to the nominal sample time.
    float update(float setpoint, float measurement, float dt_s);

    /// Zero the integrator and the derivative history.
    void reset();

    void setConfig(const PIDConfig& config) { config_ = config; }
    const PIDConfig& config() const { return config_; }

    /// Last computed terms, exposed for telemetry and tests.
    float proportionalTerm() const { return p_term_; }
    float integralTerm() const { return i_term_; }
    float derivativeTerm() const { return d_term_; }
    float lastOutput() const { return last_output_; }

private:
    float sanitizeDt(float dt_s) const;

    PIDConfig config_{};
    float integral_{0.0f};
    float previous_measurement_{0.0f};
    bool has_previous_{false};
    float filtered_derivative_{0.0f};

    float p_term_{0.0f};
    float i_term_{0.0f};
    float d_term_{0.0f};
    float last_output_{0.0f};
};

}  // namespace nav
