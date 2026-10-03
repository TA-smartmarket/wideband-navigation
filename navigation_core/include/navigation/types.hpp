// Core value types shared by the desktop simulator and the ESP32-S3 firmware.
//
// Everything in this header is portable C++17: no Arduino, no filesystem, no
// dynamic-exception-heavy code.  Fixed size arrays are used instead of
// std::string so the ESP32 build does not churn the heap while parsing a
// position line at 10 Hz.
#pragma once

#include <cstdint>
#include <cstring>
#include <string>

namespace nav {

/// Coordinate frame mandated by the project contract (see docs/coordinate_system.md).
inline constexpr const char* kFrameId = "smart_market_map";
inline constexpr uint32_t kSchemaVersion = 1;
inline constexpr const char* kDefaultTrolleyId = "TROLLEY_01";

/// Bounded string storage sizes (bytes, including the terminating NUL).
inline constexpr std::size_t kIdFieldSize = 24;
inline constexpr std::size_t kFrameFieldSize = 24;

/// Hard upper bounds so embedded builds can size their containers statically.
inline constexpr int kMaxGraphNodes = 96;
inline constexpr int kMaxGraphEdges = 384;
inline constexpr int kMaxRouteNodes = 96;

/// Copy a C string into a fixed-size buffer, always NUL terminating.
inline void copyBounded(char* dst, std::size_t dst_size, const char* src) {
    if (dst == nullptr || dst_size == 0) {
        return;
    }
    if (src == nullptr) {
        dst[0] = '\0';
        return;
    }
    std::size_t i = 0;
    for (; i + 1 < dst_size && src[i] != '\0'; ++i) {
        dst[i] = src[i];
    }
    dst[i] = '\0';
}

/// True when the fixed-size buffer equals the given C string.
inline bool boundedEquals(const char* a, const char* b) {
    if (a == nullptr || b == nullptr) {
        return false;
    }
    return std::strcmp(a, b) == 0;
}

struct Position2D {
    float x_m{0.0f};
    float y_m{0.0f};
};

/// A processed trolley position produced by the external UWB/EKF subsystem.
///
/// The navigation project never consumes raw ranging values; it consumes this
/// structure only (see docs/data_contract.md).
struct PositionMeasurement {
    uint32_t schema_version{kSchemaVersion};
    char trolley_id[kIdFieldSize]{};
    char frame_id[kFrameFieldSize]{};
    uint64_t timestamp_ms{0};
    Position2D position{};
    float quality{0.0f};
    bool valid{false};

    PositionMeasurement() {
        copyBounded(trolley_id, sizeof(trolley_id), kDefaultTrolleyId);
        copyBounded(frame_id, sizeof(frame_id), kFrameId);
    }

    void setTrolleyId(const char* id) { copyBounded(trolley_id, sizeof(trolley_id), id); }
    void setFrameId(const char* id) { copyBounded(frame_id, sizeof(frame_id), id); }
    std::string trolleyIdString() const { return std::string(trolley_id); }
    std::string frameIdString() const { return std::string(frame_id); }
};

/// Outcome of validating a raw measurement.  Kept as an enum (not a string) so
/// the control loop can branch cheaply and tests can assert on it.
enum class MeasurementStatus {
    OK = 0,
    NO_DATA,
    BAD_SCHEMA_VERSION,
    WRONG_FRAME_ID,
    WRONG_TROLLEY_ID,
    NON_FINITE_COORDINATE,
    INVALID_FLAG,
    LOW_QUALITY,
    OUT_OF_MAP_BOUNDS,
    STALE_TIMESTAMP,
};

const char* toString(MeasurementStatus status);

/// Result of snapping a position onto the navigation graph.
struct NearestNodeResult {
    bool found{false};
    int node_id{-1};
    float distance_m{0.0f};
};

/// Waypoint derived from a route node; motion control needs coordinates.
struct Waypoint {
    int node_id{-1};
    float x_m{0.0f};
    float y_m{0.0f};
};

/// High level navigation errors (see docs/navigation_algorithm.md).
enum class NavigationError {
    NONE = 0,
    INVALID_POSITION,
    POSITION_TIMEOUT,
    INVALID_DESTINATION,
    START_NODE_NOT_FOUND,
    ROUTE_NOT_FOUND,
    GRAPH_INVALID,
    CONFIG_INVALID,
    NOT_READY,
    CANCELLED,
    EMERGENCY_STOP_ACTIVE,
};

const char* toString(NavigationError error);

/// Path planning failure reasons.
enum class PathError {
    NONE = 0,
    INVALID_START,
    INVALID_DESTINATION,
    START_EQUALS_DESTINATION,
    NO_ROUTE,
    EMPTY_GRAPH,
};

const char* toString(PathError error);

/// Result of a Dijkstra query.
struct PathResult {
    bool success{false};
    int node_ids[kMaxRouteNodes]{};
    int node_count{0};
    float total_distance_m{0.0f};
    PathError error{PathError::NO_ROUTE};
    uint32_t expanded_nodes{0};

    void clear() {
        success = false;
        node_count = 0;
        total_distance_m = 0.0f;
        error = PathError::NO_ROUTE;
        expanded_nodes = 0;
    }
};

/// Plan result in the shape requested by the specification (vector based, used
/// by the simulator / tests where heap allocation is free).
struct PlanResult {
    bool success{false};
    NavigationError error{NavigationError::NONE};
    float total_distance_m{0.0f};
    int route[kMaxRouteNodes]{};
    int route_length{0};
    uint32_t expanded_nodes{0};
};

/// Navigation finite state machine states (see docs/architecture.md).
enum class NavState {
    BOOT = 0,
    IDLE,
    WAITING_FOR_POSITION,
    READY,
    PLANNING,
    NAVIGATING,
    REPLANNING,
    POSITION_LOST,
    ARRIVED,
    ERROR,
    EMERGENCY_STOP,
};

const char* toString(NavState state);

/// Human readable state name used by logs, telemetry and the status command.
const char* describeState(NavState state);

/// Numeric code used by the C API / telemetry consumers.
inline int stateCode(NavState state) { return static_cast<int>(state); }

/// Operational mode of the motor layer.
enum class MotorControlMode {
    OPEN_LOOP_SIMULATION = 0,
    CLOSED_LOOP_ENCODER,
};

/// Normalised wheel command in [-1, +1].
struct MotorCommand {
    float left{0.0f};
    float right{0.0f};
};

/// Everything the telemetry layer needs, produced once per control cycle.
struct NavigationStatus {
    NavState state{NavState::BOOT};
    NavigationError error{NavigationError::NONE};
    Position2D position{};
    float heading_rad{0.0f};
    bool heading_valid{false};
    float position_quality{0.0f};
    bool position_valid{false};
    bool position_fresh{false};

    int start_node{-1};
    int destination_node{-1};
    int active_waypoint_node{-1};
    int waypoint_index{0};
    int waypoint_count{0};
    float distance_to_waypoint_m{0.0f};
    float distance_to_destination_m{0.0f};
    /// Length of the route as first planned for the current destination.  Kept
    /// stable across replans so experiment metrics have a fixed reference.
    float planned_distance_m{0.0f};
    /// Length still to be driven on the *current* route (changes on every replan).
    float remaining_distance_m{0.0f};
    float travelled_distance_m{0.0f};

    float target_bearing_rad{0.0f};
    float heading_error_rad{0.0f};
    float cross_track_error_m{0.0f};
    bool rotate_in_place{false};

    float linear_velocity_mps{0.0f};
    float angular_velocity_radps{0.0f};
    MotorCommand motor{};

    uint32_t replan_count{0};
    uint32_t position_loss_events{0};
    uint32_t invalid_sample_count{0};
    uint32_t plan_time_us{0};
    int route[kMaxRouteNodes]{};
    int route_length{0};
};

/// Convert a floating point value to a JSON-safe finite number.
inline float sanitizeFinite(float value, float fallback = 0.0f) {
    if (value != value) {  // NaN
        return fallback;
    }
    if (value > 3.0e38f || value < -3.0e38f) {  // +/- infinity
        return fallback;
    }
    return value;
}

}  // namespace nav
