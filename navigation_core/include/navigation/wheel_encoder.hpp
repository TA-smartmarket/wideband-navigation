// Portable AS5600 angle processing. Hardware I2C access belongs in firmware;
// this class owns wrap detection, multi-turn tracking, velocity filtering and
// health/fault state so the exact same maths can be unit tested on the host.
#pragma once

#include <cstdint>

namespace nav {

struct WheelEncoderConfig {
    int32_t counts_per_revolution{4096};
    bool invert_direction{false};
    /// Sensor revolutions per wheel revolution. 1.0 when mounted on the wheel.
    float sensor_to_wheel_ratio{1.0f};
    /// First-order velocity filter coefficient in (0, 1].
    float velocity_filter_alpha{0.25f};
    /// Reject samples whose implied sensor speed exceeds this bound.
    float max_abs_sensor_rpm{600.0f};
    uint32_t stale_timeout_ms{100};
};

struct WheelEncoderSample {
    uint16_t raw_count{0};
    float raw_angle_rad{0.0f};
    int64_t unwrapped_counts{0};
    float sensor_revolutions{0.0f};
    float wheel_revolutions{0.0f};
    float raw_wheel_rpm{0.0f};
    float filtered_wheel_rpm{0.0f};
    uint64_t timestamp_us{0};
    bool valid{false};
};

class WheelEncoderTracker {
public:
    bool configure(const WheelEncoderConfig& config);
    void reset();

    /// Add a successful AS5600 raw-angle read. Returns false when the input or
    /// implied speed is unsafe; rejected samples do not alter position.
    bool update(uint16_t raw_count, uint64_t timestamp_us);
    void markCommunicationFailure();

    bool healthy(uint64_t now_us) const;
    const WheelEncoderSample& sample() const { return sample_; }
    const WheelEncoderConfig& config() const { return config_; }
    uint32_t communicationFailures() const { return communication_failures_; }
    uint32_t rejectedSamples() const { return rejected_samples_; }

private:
    WheelEncoderConfig config_{};
    WheelEncoderSample sample_{};
    uint16_t previous_raw_{0};
    bool initialized_{false};
    uint32_t communication_failures_{0};
    uint32_t rejected_samples_{0};
};

}  // namespace nav
