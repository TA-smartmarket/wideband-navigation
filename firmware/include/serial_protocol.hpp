// Versioned newline-delimited JSON control protocol for the laptop dashboard.
//
// This parser accepts only bounded, explicitly supported commands.  It is kept
// separate from navigation_core's human-oriented CMD: parser because the USB
// control interface is an integration boundary and malformed host input must
// never affect the motor task.
#pragma once

#include <cstddef>

namespace firmware {

constexpr unsigned kDashboardProtocolVersion = 1;

enum class DashboardCommandKind {
    NONE,
    HELLO,
    MANUAL,
    ESTOP,
    RESET_ESTOP,
    NAVIGATION_STOP,
    NAVIGATION_DESTINATION,
    CONFIG_GET,
    CONFIG_SAVE,
    CONFIG_DEFAULTS,
};

struct DashboardCommand {
    DashboardCommandKind kind{DashboardCommandKind::NONE};
    float linear_mps{0.0f};
    float angular_radps{0.0f};
    int node_id{-1};
};

/// Parse one NDJSON object.  Returns false for invalid/unsupported input and
/// writes a short reason into error when it is provided.
bool parseDashboardCommand(const char* line, DashboardCommand& command, char* error,
                           std::size_t error_size);

}  // namespace firmware
