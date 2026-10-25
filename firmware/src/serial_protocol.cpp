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
    StaticJsonDocument<384> document;
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
