// Closed-loop wheel velocity controller for a STEP/DIR actuator.
#pragma once

#include <cstdint>

#include "navigation/pid.hpp"

namespace nav {

struct WheelVelocityControlConfig {
    PIDConfig pid{};
    float max_abs_rpm{200.0f};
    float zero_deadband_rpm{0.25f};
    float tracking_error_rpm{25.0f};
    uint32_t tracking_error_timeout_ms{500};
};

struct WheelVelocityControlStatus {
    float target_rpm{0.0f};
    float measured_rpm{0.0f};
    float error_rpm{0.0f};
    float output_rpm{0.0f};
    float p_term{0.0f};
    float i_term{0.0f};
    float d_term{0.0f};
    bool saturated{false};
    bool tracking_fault{false};
};

class WheelVelocityController {
public:
    bool configure(const WheelVelocityControlConfig& config);
    void reset();
    float update(float target_rpm, float measured_rpm, float dt_s,
                 uint64_t now_ms, bool feedback_healthy);
    const WheelVelocityControlStatus& status() const { return status_; }

private:
    WheelVelocityControlConfig config_{};
    PIDController pid_{};
    WheelVelocityControlStatus status_{};
    uint64_t excessive_error_since_ms_{0};
};

}  // namespace nav
