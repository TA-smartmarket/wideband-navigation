#include "navigation/wheel_encoder.hpp"

#include <cmath>

#include "navigation/geometry.hpp"

namespace nav {

bool WheelEncoderTracker::configure(const WheelEncoderConfig& config) {
    if (config.counts_per_revolution < 2 ||
        !isFinite(config.sensor_to_wheel_ratio) || config.sensor_to_wheel_ratio <= 0.0f ||
        !isFinite(config.velocity_filter_alpha) || config.velocity_filter_alpha <= 0.0f ||
        config.velocity_filter_alpha > 1.0f ||
        !isFinite(config.max_abs_sensor_rpm) || config.max_abs_sensor_rpm <= 0.0f ||
        config.stale_timeout_ms == 0) {
        return false;
    }
    config_ = config;
    reset();
    return true;
}

void WheelEncoderTracker::reset() {
    sample_ = WheelEncoderSample{};
    previous_raw_ = 0;
    initialized_ = false;
    communication_failures_ = 0;
    rejected_samples_ = 0;
}

bool WheelEncoderTracker::update(uint16_t raw_count, uint64_t timestamp_us) {
    const int32_t counts = config_.counts_per_revolution;
    if (timestamp_us == 0 || static_cast<int32_t>(raw_count) >= counts) {
        ++rejected_samples_;
        return false;
    }

    if (!initialized_) {
        initialized_ = true;
        previous_raw_ = raw_count;
        sample_.raw_count = raw_count;
        sample_.raw_angle_rad = kTwoPi * static_cast<float>(raw_count) /
                                static_cast<float>(counts);
        sample_.timestamp_us = timestamp_us;
        sample_.valid = true;
        return true;
    }

    if (timestamp_us <= sample_.timestamp_us) {
        ++rejected_samples_;
        return false;
    }

    int32_t delta = static_cast<int32_t>(raw_count) - static_cast<int32_t>(previous_raw_);
    const int32_t half = counts / 2;
    if (delta > half) delta -= counts;
    if (delta < -half) delta += counts;
    if (config_.invert_direction) delta = -delta;

    const float dt_s = static_cast<float>(timestamp_us - sample_.timestamp_us) * 1.0e-6f;
    const float sensor_rpm = static_cast<float>(delta) * 60.0f /
                             (static_cast<float>(counts) * dt_s);
    if (!isFinite(sensor_rpm) || std::fabs(sensor_rpm) > config_.max_abs_sensor_rpm) {
        ++rejected_samples_;
        return false;
    }

    sample_.unwrapped_counts += delta;
    sample_.sensor_revolutions = static_cast<float>(sample_.unwrapped_counts) /
                                 static_cast<float>(counts);
    sample_.wheel_revolutions = sample_.sensor_revolutions /
                                config_.sensor_to_wheel_ratio;
    sample_.raw_wheel_rpm = sensor_rpm / config_.sensor_to_wheel_ratio;
    const float alpha = config_.velocity_filter_alpha;
    sample_.filtered_wheel_rpm = alpha * sample_.raw_wheel_rpm +
                                 (1.0f - alpha) * sample_.filtered_wheel_rpm;
    sample_.raw_count = raw_count;
    sample_.raw_angle_rad = kTwoPi * static_cast<float>(raw_count) /
                            static_cast<float>(counts);
    sample_.timestamp_us = timestamp_us;
    sample_.valid = true;
    previous_raw_ = raw_count;
    return true;
}

void WheelEncoderTracker::markCommunicationFailure() {
    ++communication_failures_;
    sample_.valid = false;
}

bool WheelEncoderTracker::healthy(uint64_t now_us) const {
    if (!sample_.valid || !initialized_ || now_us < sample_.timestamp_us) return false;
    const uint64_t timeout_us = static_cast<uint64_t>(config_.stale_timeout_ms) * 1000u;
    return now_us - sample_.timestamp_us <= timeout_us;
}

}  // namespace nav
