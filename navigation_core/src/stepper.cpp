#include "navigation/stepper.hpp"

#include <cmath>

#include "navigation/differential_drive.hpp"

namespace nav {

namespace {
/// Guard against a zero/negative/absurd time step.
float sanitizeDt(float dt_s) {
    if (!isFinite(dt_s) || dt_s <= 0.0f) {
        return 0.0f;
    }
    // 1 s is already far beyond any sane control period; clamping keeps a
    // scheduling stall from producing a huge single burst of steps.
    return dt_s > 1.0f ? 1.0f : dt_s;
}
}  // namespace

StepperValidation validateStepperConfig(const StepperConfig& config) {
    StepperValidation result;
    if (config.steps_per_revolution <= 0) {
        result.fail("stepper.steps_per_revolution must be > 0");
        return result;
    }
    if (config.microsteps < 1) {
        result.fail("stepper.microsteps must be >= 1");
        return result;
    }
    if (!isFinite(config.wheel_radius_m) || config.wheel_radius_m <= 0.0f) {
        result.fail("stepper.wheel_radius_m must be > 0");
        return result;
    }
    if (!isFinite(config.max_step_rate_hz) || config.max_step_rate_hz <= 0.0f) {
        result.fail("stepper.max_step_rate_hz must be > 0");
        return result;
    }
    if (!isFinite(config.min_step_rate_hz) || config.min_step_rate_hz < 0.0f) {
        result.fail("stepper.min_step_rate_hz must be >= 0");
        return result;
    }
    if (config.min_step_rate_hz > config.max_step_rate_hz) {
        result.fail("stepper.min_step_rate_hz must be <= max_step_rate_hz");
        return result;
    }
    if (!isFinite(config.max_step_accel_hz_per_s) || config.max_step_accel_hz_per_s <= 0.0f) {
        result.fail("stepper.max_step_accel_hz_per_s must be > 0");
        return result;
    }
    return result;
}

float stepsPerMeter(const StepperConfig& config) {
    if (!isFinite(config.wheel_radius_m) || config.wheel_radius_m <= 0.0f) {
        return 0.0f;
    }
    const float circumference = kTwoPi * config.wheel_radius_m;
    if (circumference <= 1.0e-6f) {
        return 0.0f;
    }
    return static_cast<float>(stepsPerRevolution(config)) / circumference;
}

float stepRateToRpm(float step_rate_hz, const StepperConfig& config) {
    const int32_t per_rev = stepsPerRevolution(config);
    if (per_rev <= 0 || !isFinite(step_rate_hz)) {
        return 0.0f;
    }
    return step_rate_hz * 60.0f / static_cast<float>(per_rev);
}

float rpmToStepRate(float rpm, const StepperConfig& config) {
    const int32_t per_rev = stepsPerRevolution(config);
    if (per_rev <= 0 || !isFinite(rpm)) {
        return 0.0f;
    }
    return rpm * static_cast<float>(per_rev) / 60.0f;
}

float wheelVelocityToStepRate(float velocity_mps, const StepperConfig& config) {
    if (!isFinite(velocity_mps)) {
        return 0.0f;
    }
    return velocity_mps * stepsPerMeter(config);
}

float stepRateToWheelVelocity(float step_rate_hz, const StepperConfig& config) {
    const float per_meter = stepsPerMeter(config);
    if (per_meter <= 1.0e-6f || !isFinite(step_rate_hz)) {
        return 0.0f;
    }
    return step_rate_hz / per_meter;
}

float stepsToRevolutions(int64_t steps, const StepperConfig& config) {
    const int32_t per_rev = stepsPerRevolution(config);
    if (per_rev <= 0) {
        return 0.0f;
    }
    return static_cast<float>(steps) / static_cast<float>(per_rev);
}

float stepsToDistanceM(int64_t steps, const StepperConfig& config) {
    const float per_meter = stepsPerMeter(config);
    if (per_meter <= 1.0e-6f) {
        return 0.0f;
    }
    return static_cast<float>(steps) / per_meter;
}

// ---------------------------------------------------------------------------
// StepperAxis
// ---------------------------------------------------------------------------

void StepperAxis::configure(const StepperConfig& config) {
    config_ = config;
    if (!isFinite(config_.max_step_rate_hz) || config_.max_step_rate_hz <= 0.0f) {
        config_.max_step_rate_hz = 20000.0f;
    }
    if (!isFinite(config_.max_step_accel_hz_per_s) || config_.max_step_accel_hz_per_s <= 0.0f) {
        config_.max_step_accel_hz_per_s = config_.max_step_rate_hz;
    }
    reset();
}

void StepperAxis::reset() {
    position_steps_ = 0;
    target_position_steps_ = 0;
    current_rate_hz_ = 0.0f;
    target_rate_hz_ = 0.0f;
    position_rate_hz_ = 0.0f;
    direction_ = 0;
    phase_ = 0.0f;
    emitted_steps_ = 0;
    mode_ = StepperMode::CONTINUOUS;
}

void StepperAxis::setTargetRateHz(float rate_hz) {
    if (!isFinite(rate_hz)) {
        rate_hz = 0.0f;
    }
    const float limit = config_.max_step_rate_hz;
    target_rate_hz_ = clampf(rate_hz, -limit, limit);
    // A magnitude below the minimum is treated as "stop" so a nearly-zero
    // command cannot emit isolated stray pulses.
    if (std::fabs(target_rate_hz_) < config_.min_step_rate_hz) {
        target_rate_hz_ = 0.0f;
    }
}

void StepperAxis::setTargetRpm(float rpm) { setTargetRateHz(rpmToStepRate(rpm, config_)); }

void StepperAxis::setTargetPositionSteps(int64_t steps) {
    target_position_steps_ = steps;
    mode_ = StepperMode::POSITION;
    position_rate_hz_ = config_.max_step_rate_hz;
}

void StepperAxis::moveRevolutions(float revolutions) {
    if (!isFinite(revolutions)) {
        return;
    }
    const int32_t per_rev = stepsPerRevolution(config_);
    const float delta = revolutions * static_cast<float>(per_rev);
    setTargetPositionSteps(position_steps_ + static_cast<int64_t>(std::lround(delta)));
}

void StepperAxis::setContinuousMode() {
    mode_ = StepperMode::CONTINUOUS;
    position_rate_hz_ = 0.0f;
}

bool StepperAxis::positionReached() const {
    return mode_ == StepperMode::POSITION && position_steps_ == target_position_steps_ &&
           current_rate_hz_ == 0.0f;
}

float StepperAxis::stoppingRateLimit() const {
    // v_max such that the axis can still decelerate to rest within the remaining
    // distance: v = sqrt(2 * a * s).  This is what makes the profile stop exactly
    // on target instead of overshooting and hunting.
    const int64_t remaining = target_position_steps_ - position_steps_;
    const float distance = std::fabs(static_cast<float>(remaining));
    const float accel = config_.max_step_accel_hz_per_s;
    return std::sqrt(2.0f * accel * distance);
}

void StepperAxis::rampTowards(float desired, float dt_s) {
    const float max_delta = config_.max_step_accel_hz_per_s * dt_s;
    const float delta = desired - current_rate_hz_;
    if (delta > max_delta) {
        current_rate_hz_ += max_delta;
    } else if (delta < -max_delta) {
        current_rate_hz_ -= max_delta;
    } else {
        current_rate_hz_ = desired;
    }
}

int32_t StepperAxis::advance(float dt_s) {
    dt_s = sanitizeDt(dt_s);
    if (dt_s <= 0.0f) {
        return 0;
    }

    float desired = 0.0f;
    if (mode_ == StepperMode::CONTINUOUS) {
        desired = target_rate_hz_;
    } else {
        const int64_t remaining = target_position_steps_ - position_steps_;
        if (remaining == 0) {
            position_rate_hz_ = 0.0f;
            desired = 0.0f;
        } else {
            // Drive at the profile rate, but never faster than the axis can still
            // brake from, so the stop lands on the target step.
            const float sign = remaining > 0 ? 1.0f : -1.0f;
            const float limit = stoppingRateLimit();
            if (position_rate_hz_ > limit) {
                position_rate_hz_ = limit;
            }
            desired = sign * position_rate_hz_;
            // Guard against a single cycle covering more than the whole
            // remaining move (very coarse dt or a tiny move): cap the rate so the
            // axis cannot overshoot within one step interval.
            const float max_per_cycle =
                std::fabs(static_cast<float>(remaining)) / dt_s;
            if (std::fabs(desired) > max_per_cycle) {
                desired = sign * max_per_cycle;
            }
        }
    }

    rampTowards(desired, dt_s);

    if (mode_ == StepperMode::POSITION) {
        // Cap the *emitted* rate by the braking limit as well.  Ramping alone is
        // not enough: the limit falls as sqrt(remaining distance), so it can drop
        // faster than the acceleration ramp can follow, and a stepper that is
        // asked to decelerate harder than its torque allows simply loses steps.
        const float limit = stoppingRateLimit();
        if (std::fabs(current_rate_hz_) > limit) {
            current_rate_hz_ = current_rate_hz_ > 0.0f ? limit : -limit;
        }
    }

    const float magnitude = std::fabs(current_rate_hz_);
    if (magnitude < config_.min_step_rate_hz) {
        if (mode_ == StepperMode::POSITION) {
            // A position move must still terminate.  The braking limit falls as
            // sqrt(remaining distance), so the rate eventually drops below the
            // minimum step rate while a few steps are still outstanding; without
            // this the axis would creep towards the target forever.  Emit the
            // remainder and declare the move complete.
            const int64_t remaining = target_position_steps_ - position_steps_;
            const int64_t magnitude_remaining = remaining < 0 ? -remaining : remaining;
            if (magnitude_remaining > 0) {
                current_rate_hz_ = 0.0f;
                position_steps_ = target_position_steps_;
                emitted_steps_ += static_cast<uint64_t>(magnitude_remaining);
                direction_ = 0;
                phase_ = 0.0f;
                return static_cast<int32_t>(magnitude_remaining);
            }
        }
        // Below the floor the axis is idle: no steps, no direction.
        direction_ = 0;
        phase_ = 0.0f;
        return 0;
    }

    const int sign = current_rate_hz_ > 0.0f ? 1 : -1;
    const int dir_level = config_.invert_direction ? -sign : sign;
    direction_ = dir_level;

    // DDS phase accumulator: the average rate is exactly `magnitude` even when it
    // is not an integer multiple of 1/dt, because the fractional remainder is
    // carried to the next call.
    //
    // The remainder MUST be preserved when it does not yet amount to a whole
    // step.  Zeroing it here (an earlier revision did) breaks the accumulator for
    // every rate below 1/dt - which on the firmware, with a 20 kHz pulse base,
    // is the entire usable range: no pulse would ever be emitted.
    phase_ += magnitude * dt_s;
    int32_t steps = static_cast<int32_t>(phase_);
    if (steps > 0) {
        phase_ -= static_cast<float>(steps);
    }
    if (phase_ >= 1.0f) {
        phase_ = 0.0f;  // defensive: cannot happen after the subtraction above
    }

    if (mode_ == StepperMode::POSITION) {
        const int64_t remaining = target_position_steps_ - position_steps_;
        const int64_t magnitude_remaining = remaining < 0 ? -remaining : remaining;
        if (static_cast<int64_t>(steps) >= magnitude_remaining) {
            // Final cycle: cover exactly the rest and stop.
            //
            // This explicit termination matters.  The braking limit falls as
            // sqrt(remaining), so the rate would asymptotically approach zero
            // and the axis would creep towards the target forever without ever
            // arriving.  Emitting the remaining steps and zeroing the rate
            // guarantees arrival, with no overshoot and no hunting.
            steps = static_cast<int32_t>(magnitude_remaining);
            current_rate_hz_ = 0.0f;
            position_steps_ = target_position_steps_;
        } else if (steps > 0) {
            const int travel_sign = remaining > 0 ? 1 : -1;
            position_steps_ += travel_sign * steps;
        }
        if (steps <= 0) {
            direction_ = 0;
        }
    } else {
        if (steps > 0) {
            position_steps_ += sign * steps;
        }
    }

    emitted_steps_ += static_cast<uint64_t>(steps);
    return steps;
}

// ---------------------------------------------------------------------------
// StepperDrive
// ---------------------------------------------------------------------------

void StepperDrive::configure(const StepperConfig& left, const StepperConfig& right,
                             float wheel_base_m, float max_wheel_speed_mps) {
    left_.configure(left);
    right_.configure(right);
    wheel_base_m_ = (isFinite(wheel_base_m) && wheel_base_m > 1.0e-6f) ? wheel_base_m : 0.32f;
    max_wheel_speed_mps_ =
        (isFinite(max_wheel_speed_mps) && max_wheel_speed_mps > 0.0f) ? max_wheel_speed_mps : 0.70f;
}

void StepperDrive::reset() {
    left_.reset();
    right_.reset();
}

void StepperDrive::commandTwist(float linear_mps, float angular_radps, float dt_s) {
    // Same inverse kinematics as the PWM drive; only the actuator differs.
    const WheelVelocities wheels = twistToWheelVelocities(Twist{linear_mps, angular_radps},
                                                          wheel_base_m_);
    const float left_speed = clampf(wheels.left_mps, -max_wheel_speed_mps_, max_wheel_speed_mps_);
    const float right_speed = clampf(wheels.right_mps, -max_wheel_speed_mps_, max_wheel_speed_mps_);

    left_.setTargetRateHz(wheelVelocityToStepRate(left_speed, left_.config()));
    right_.setTargetRateHz(wheelVelocityToStepRate(right_speed, right_.config()));
    left_.advance(dt_s);
    right_.advance(dt_s);
}

float StepperDrive::odometryDistanceM() const {
    return 0.5f * (left_.distanceM() + right_.distanceM());
}

float StepperDrive::odometryYawRad() const {
    if (wheel_base_m_ <= 1.0e-6f) {
        return 0.0f;
    }
    return (right_.distanceM() - left_.distanceM()) / wheel_base_m_;
}

}  // namespace nav
