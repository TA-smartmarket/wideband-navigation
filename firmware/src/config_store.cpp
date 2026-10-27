#include "config_store.hpp"

#include <cstddef>
#include <cstring>

namespace firmware {

namespace {
constexpr uint32_t kSchemaVersion = 1;
constexpr const char* kNamespace = "trolley-nav";
constexpr const char* kActiveKey = "active";
constexpr const char* kBackupKey = "last-good";
ConfigStore g_store;
}

uint32_t ConfigStore::calculateCrc(const PersistedData& data) {
    const auto* bytes = reinterpret_cast<const uint8_t*>(&data);
    const size_t length = offsetof(PersistedData, crc32);
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < length; ++i) {
        crc ^= bytes[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1u) ^ (0xEDB88320u & (0u - (crc & 1u)));
        }
    }
    return ~crc;
}

bool ConfigStore::valid(const PersistedData& data) {
    return data.schema_version == kSchemaVersion && data.crc32 == calculateCrc(data);
}

void ConfigStore::apply(const PersistedData& data, nav::NavigationConfig& config) {
    config.wheel_pid_left.pid.kp = data.left_kp;
    config.wheel_pid_left.pid.ki = data.left_ki;
    config.wheel_pid_left.pid.kd = data.left_kd;
    config.wheel_pid_right.pid.kp = data.right_kp;
    config.wheel_pid_right.pid.ki = data.right_ki;
    config.wheel_pid_right.pid.kd = data.right_kd;
    config.motion.heading_kp = data.heading_kp;
    config.heading_pid.kp = data.heading_kp;
    config.heading_pid.ki = data.heading_ki;
    config.heading_pid.kd = data.heading_kd;
    config.drive.wheel_radius_m = data.wheel_radius_m;
    config.drive.wheel_base_m = data.wheel_track_m;
    config.odometry.wheel_radius_m = data.wheel_radius_m;
    config.odometry.wheel_track_m = data.wheel_track_m;
    config.stepper_left.wheel_radius_m = data.wheel_radius_m;
    config.stepper_right.wheel_radius_m = data.wheel_radius_m;
    config.encoder_left.sensor_to_wheel_ratio = data.left_sensor_ratio;
    config.encoder_right.sensor_to_wheel_ratio = data.right_sensor_ratio;
    config.stepper_left.microsteps = data.microsteps;
    config.stepper_right.microsteps = data.microsteps;
    config.position_source = static_cast<nav::PositionSourceMode>(data.position_source);
    config.encoder_left.invert_direction = data.left_encoder_invert != 0;
    config.encoder_right.invert_direction = data.right_encoder_invert != 0;
    config.stepper_left.invert_direction = data.left_motor_invert != 0;
    config.stepper_right.invert_direction = data.right_motor_invert != 0;
}

ConfigStore::PersistedData ConfigStore::capture(const nav::NavigationConfig& config,
                                                uint32_t revision) {
    PersistedData data{};
    data.schema_version = kSchemaVersion;
    data.revision = revision;
    data.left_kp = config.wheel_pid_left.pid.kp;
    data.left_ki = config.wheel_pid_left.pid.ki;
    data.left_kd = config.wheel_pid_left.pid.kd;
    data.right_kp = config.wheel_pid_right.pid.kp;
    data.right_ki = config.wheel_pid_right.pid.ki;
    data.right_kd = config.wheel_pid_right.pid.kd;
    data.heading_kp = config.heading_pid.kp;
    data.heading_ki = config.heading_pid.ki;
    data.heading_kd = config.heading_pid.kd;
    data.wheel_radius_m = config.drive.wheel_radius_m;
    data.wheel_track_m = config.drive.wheel_base_m;
    data.left_sensor_ratio = config.encoder_left.sensor_to_wheel_ratio;
    data.right_sensor_ratio = config.encoder_right.sensor_to_wheel_ratio;
    data.microsteps = config.stepper_left.microsteps;
    data.position_source = static_cast<uint8_t>(config.position_source);
    data.left_encoder_invert = config.encoder_left.invert_direction;
    data.right_encoder_invert = config.encoder_right.invert_direction;
    data.left_motor_invert = config.stepper_left.invert_direction;
    data.right_motor_invert = config.stepper_right.invert_direction;
    data.crc32 = calculateCrc(data);
    return data;
}

bool ConfigStore::begin(nav::NavigationConfig& config) {
    open_ = preferences_.begin(kNamespace, false);
    if (!open_) return false;
    PersistedData data{};
    if (preferences_.getBytesLength(kActiveKey) == sizeof(data) &&
        preferences_.getBytes(kActiveKey, &data, sizeof(data)) == sizeof(data) && valid(data)) {
        nav::NavigationConfig candidate = config;
        apply(data, candidate);
        if (nav::validateConfig(candidate).valid) {
            config = candidate;
            revision_ = data.revision;
            return true;
        }
    }
    if (preferences_.getBytesLength(kBackupKey) == sizeof(data) &&
        preferences_.getBytes(kBackupKey, &data, sizeof(data)) == sizeof(data) && valid(data)) {
        nav::NavigationConfig candidate = config;
        apply(data, candidate);
        if (nav::validateConfig(candidate).valid) {
            config = candidate;
            revision_ = data.revision;
            return true;
        }
    }
    revision_ = 0;
    return false;
}

bool ConfigStore::save(const nav::NavigationConfig& config) {
    if (!open_ || !nav::validateConfig(config).valid) return false;
    PersistedData previous{};
    if (preferences_.getBytesLength(kActiveKey) == sizeof(previous) &&
        preferences_.getBytes(kActiveKey, &previous, sizeof(previous)) == sizeof(previous) &&
        valid(previous)) {
        preferences_.putBytes(kBackupKey, &previous, sizeof(previous));
    }
    const PersistedData candidate = capture(config, revision_ + 1u);
    if (preferences_.putBytes(kActiveKey, &candidate, sizeof(candidate)) != sizeof(candidate)) {
        return false;
    }
    PersistedData verify{};
    if (preferences_.getBytes(kActiveKey, &verify, sizeof(verify)) != sizeof(verify) ||
        !valid(verify)) return false;
    revision_ = candidate.revision;
    return true;
}

bool ConfigStore::restoreDefaults() {
    if (!open_) return false;
    const bool active_removed = preferences_.remove(kActiveKey) ||
                                preferences_.getBytesLength(kActiveKey) == 0;
    preferences_.remove(kBackupKey);
    revision_ = 0;
    return active_removed;
}

void ConfigStore::formatJson(const nav::NavigationConfig& config, char* output,
                             size_t output_size) const {
    if (output == nullptr || output_size == 0) return;
    std::snprintf(output, output_size,
        "{\"type\":\"config\",\"protocol_version\":1,\"schema_version\":1,"
        "\"revision\":%lu,\"left\":{\"kp\":%.5f,\"ki\":%.5f,\"kd\":%.5f},"
        "\"right\":{\"kp\":%.5f,\"ki\":%.5f,\"kd\":%.5f},"
        "\"heading\":{\"kp\":%.5f,\"ki\":%.5f,\"kd\":%.5f},"
        "\"wheel_radius_m\":%.5f,\"wheel_track_m\":%.5f,\"position_source\":%u}\n",
        static_cast<unsigned long>(revision_),
        static_cast<double>(config.wheel_pid_left.pid.kp),
        static_cast<double>(config.wheel_pid_left.pid.ki),
        static_cast<double>(config.wheel_pid_left.pid.kd),
        static_cast<double>(config.wheel_pid_right.pid.kp),
        static_cast<double>(config.wheel_pid_right.pid.ki),
        static_cast<double>(config.wheel_pid_right.pid.kd),
        static_cast<double>(config.heading_pid.kp),
        static_cast<double>(config.heading_pid.ki),
        static_cast<double>(config.heading_pid.kd),
        static_cast<double>(config.drive.wheel_radius_m),
        static_cast<double>(config.drive.wheel_base_m),
        static_cast<unsigned>(config.position_source));
}

ConfigStore& configStore() { return g_store; }

}  // namespace firmware
