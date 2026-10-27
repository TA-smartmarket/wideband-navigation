#include "serial_protocol.hpp"

#include <ArduinoJson.h>

#include <cmath>
#include <cstdio>
#include <cstring>

namespace firmware {
namespace {

void setError(char* error, std::size_t size, const char* message) {
    if (error != nullptr && size > 0) {
        std::snprintf(error, size, "%s", message);
    }
}

bool finiteNumber(JsonVariantConst value, float& result) {
    if (!value.is<float>()) {
        return false;
    }
    result = value.as<float>();
    return std::isfinite(result);
}

}  // namespace

bool parseDashboardCommand(const char* line, DashboardCommand& command, char* error,
                           std::size_t error_size) {
    command = DashboardCommand{};
    if (line == nullptr) {
        setError(error, error_size, "empty command");
        return false;
    }
    JsonDocument document;
    const DeserializationError json_error = deserializeJson(document, line);
    if (json_error || !document.is<JsonObject>()) {
        setError(error, error_size, "invalid JSON command");
        return false;
    }
    const JsonObjectConst root = document.as<JsonObjectConst>();
    const unsigned version = root["protocol_version"] | kDashboardProtocolVersion;
    if (version != kDashboardProtocolVersion) {
        setError(error, error_size, "unsupported protocol_version");
        return false;
    }
    const char* type = root["type"] | "";
    if (std::strcmp(type, "hello") == 0) command.kind = DashboardCommandKind::HELLO;
    else if (std::strcmp(type, "estop") == 0) command.kind = DashboardCommandKind::ESTOP;
    else if (std::strcmp(type, "estop.reset") == 0) command.kind = DashboardCommandKind::RESET_ESTOP;
    else if (std::strcmp(type, "navigation.stop") == 0) command.kind = DashboardCommandKind::NAVIGATION_STOP;
    else if (std::strcmp(type, "config.get") == 0) command.kind = DashboardCommandKind::CONFIG_GET;
    else if (std::strcmp(type, "config.save") == 0) command.kind = DashboardCommandKind::CONFIG_SAVE;
    else if (std::strcmp(type, "config.defaults") == 0) command.kind = DashboardCommandKind::CONFIG_DEFAULTS;
    else if (std::strcmp(type, "encoder.zero") == 0) command.kind = DashboardCommandKind::ENCODER_ZERO;
    else if (std::strcmp(type, "odometry.set_pose") == 0) {
        if (!finiteNumber(root["x_m"], command.pose_x_m) ||
            !finiteNumber(root["y_m"], command.pose_y_m) ||
            !finiteNumber(root["heading_rad"], command.pose_heading_rad) ||
            std::fabs(command.pose_x_m) > 100.0f ||
            std::fabs(command.pose_y_m) > 100.0f ||
            std::fabs(command.pose_heading_rad) > 6.2831854f) {
            setError(error, error_size, "initial pose is invalid or out of range");
            return false;
        }
        command.kind = DashboardCommandKind::ODOMETRY_SET_POSE;
    }
    else if (std::strcmp(type, "pid.set") == 0) {
        const char* motor = root["motor"] | "";
        if (std::strcmp(motor, "left") != 0 && std::strcmp(motor, "right") != 0 &&
            std::strcmp(motor, "heading") != 0) {
            setError(error, error_size, "pid.set target must be left, right or heading");
            return false;
        }
        if (!finiteNumber(root["kp"], command.kp) ||
            !finiteNumber(root["ki"], command.ki) ||
            !finiteNumber(root["kd"], command.kd) ||
            command.kp < 0.0f || command.kp > 10.0f ||
            command.ki < 0.0f || command.ki > 10.0f ||
            command.kd < 0.0f || command.kd > 10.0f) {
            setError(error, error_size, "PID gains must be finite within [0, 10]");
            return false;
        }
        command.kind = DashboardCommandKind::PID_SET;
        command.right_wheel = std::strcmp(motor, "right") == 0;
        command.heading_pid = std::strcmp(motor, "heading") == 0;
    } else if (std::strcmp(type, "position.source") == 0) {
        const char* source = root["source"] | "";
        if (std::strcmp(source, "encoder_odometry") == 0) command.position_source = 0;
        else if (std::strcmp(source, "external_mock_uwb") == 0) command.position_source = 1;
        else if (std::strcmp(source, "real_uwb") == 0) command.position_source = 2;
        else {
            setError(error, error_size, "invalid position source");
            return false;
        }
        command.kind = DashboardCommandKind::POSITION_SOURCE;
    }
    else if (std::strcmp(type, "navigation.destination") == 0) {
        const int node_id = root["node_id"] | -1;
        if (node_id <= 0) {
            setError(error, error_size, "node_id must be positive");
            return false;
        }
        command.kind = DashboardCommandKind::NAVIGATION_DESTINATION;
        command.node_id = node_id;
    } else if (std::strcmp(type, "motor.manual") == 0) {
        float linear = 0.0f;
        float angular = 0.0f;
        if (!finiteNumber(root["linear_mps"], linear) || !finiteNumber(root["angular_radps"], angular)) {
            setError(error, error_size, "manual velocities must be finite numbers");
            return false;
        }
        // Limits match the compiled navigation configuration.  They are an
        // independent serial safety boundary, not merely UI validation.
        if (std::fabs(linear) > 0.45f || std::fabs(angular) > 2.2f) {
            setError(error, error_size, "manual velocity exceeds safety limit");
            return false;
        }
        command.kind = DashboardCommandKind::MANUAL;
        command.linear_mps = linear;
        command.angular_radps = angular;
    } else {
        setError(error, error_size, "unsupported command type");
        return false;
    }
    return true;
}

}  // namespace firmware
