#pragma once

#include <Arduino.h>
#include <Preferences.h>

#include "navigation/navigation_config.hpp"

namespace firmware {

class ConfigStore {
public:
    /// Load a CRC-protected persisted overlay into the validated repository
    /// defaults. Returns true when a stored configuration was applied.
    bool begin(nav::NavigationConfig& config);
    bool save(const nav::NavigationConfig& config);
    bool restoreDefaults();
    uint32_t revision() const { return revision_; }
    void formatJson(const nav::NavigationConfig& config, char* output,
                    size_t output_size) const;

private:
    struct PersistedData {
        uint32_t schema_version{1};
        uint32_t revision{0};
        float left_kp{0.0f}, left_ki{0.0f}, left_kd{0.0f};
        float right_kp{0.0f}, right_ki{0.0f}, right_kd{0.0f};
        float heading_kp{0.0f}, heading_ki{0.0f}, heading_kd{0.0f};
        float wheel_radius_m{0.0f}, wheel_track_m{0.0f};
        float left_sensor_ratio{1.0f}, right_sensor_ratio{1.0f};
        int32_t microsteps{16};
        uint8_t position_source{0};
        uint8_t left_encoder_invert{0};
        uint8_t right_encoder_invert{0};
        uint8_t left_motor_invert{0};
        uint8_t right_motor_invert{0};
        uint8_t reserved[3]{};
        uint32_t crc32{0};
    };

    static uint32_t calculateCrc(const PersistedData& data);
    static bool valid(const PersistedData& data);
    static void apply(const PersistedData& data, nav::NavigationConfig& config);
    static PersistedData capture(const nav::NavigationConfig& config, uint32_t revision);

    Preferences preferences_{};
    uint32_t revision_{0};
    bool open_{false};
};

ConfigStore& configStore();

}  // namespace firmware
