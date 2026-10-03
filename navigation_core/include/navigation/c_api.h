// Flat C ABI over the navigation core.
//
// Purpose: let the Python desktop simulator drive the *identical* C++
// navigation core (graph, Dijkstra, waypoint manager, follower, PID, FSM)
// through ctypes instead of reimplementing the algorithms in Python.  This
// satisfies the "single source of truth" requirement: the Python code owns only
// the physics, the UWB sensor model and the rendering.
//
// The ABI is deliberately narrow and POD-only: no STL types cross the boundary.
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32)
#define NAV_API __declspec(dllexport)
#else
#define NAV_API __attribute__((visibility("default")))
#endif

/// Maximum sizes mirrored from navigation/types.hpp.
#define NAV_MAX_ROUTE_NODES 96
#define NAV_MAX_ID_LEN 24
#define NAV_MAX_ERROR_LEN 160

/// Handle to one navigation session (opaque to the caller).
typedef void* nav_handle;

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

/// Create a session from JSON config text.  Returns NULL on failure.
/// `graph_json` is the content of config/graph.json, `config_json` that of
/// config/navigation.json (NULL means "use defaults").
NAV_API nav_handle nav_create(const char* graph_json, const char* config_json);

/// Destroy a session created by nav_create().
NAV_API void nav_destroy(nav_handle handle);

/// Last error message produced by nav_create() (empty string when fine).
NAV_API const char* nav_last_error(void);

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------

/// Set the destination node.  Returns 1 on success, 0 on rejection.
NAV_API int nav_request_destination(nav_handle handle, int node_id);

/// Re-arm the destination held across an emergency stop (the `start` command).
NAV_API int nav_start(nav_handle handle);

/// Cancel navigation (the `cancel_navigation` command).
NAV_API void nav_cancel(nav_handle handle);

/// Stop and keep the destination.
NAV_API void nav_stop(nav_handle handle);

/// Software emergency stop.
NAV_API void nav_emergency_stop(nav_handle handle);

/// Clear the emergency stop (does not resume motion).
NAV_API void nav_clear_emergency_stop(nav_handle handle);

/// Request a replan on the next cycle.
NAV_API void nav_request_replan(nav_handle handle);

/// Reset all runtime state (keeps the graph and configuration).
NAV_API void nav_reset(nav_handle handle);

// ---------------------------------------------------------------------------
// Inputs
// ---------------------------------------------------------------------------

/// Submit a raw UWB sample.  Returns the MeasurementStatus code
/// (0 == OK, see navigation/types.hpp).
NAV_API int nav_submit_position(nav_handle handle, double x_m, double y_m, double quality,
                                uint64_t timestamp_ms, int valid);

/// Advance the state machine by one control cycle.  `now_ms` must be a
/// monotonic millisecond timestamp owned by the caller.
NAV_API void nav_update(nav_handle handle, uint64_t now_ms);

// ---------------------------------------------------------------------------
// Observation
// ---------------------------------------------------------------------------

/// Snapshot of the navigation status, filled by nav_get_status().
typedef struct nav_status_s {
    int state;              ///< NavState enum value
    int error;              ///< NavigationError enum value
    double x_m;
    double y_m;
    double heading_rad;
    int heading_valid;
    double position_quality;
    int position_valid;
    int position_fresh;

    int start_node;
    int destination_node;
    int active_waypoint_node;
    int waypoint_index;
    int waypoint_count;
    double distance_to_waypoint_m;
    double distance_to_destination_m;
    double planned_distance_m;
    double remaining_distance_m;
    double travelled_distance_m;

    double target_bearing_rad;
    double heading_error_rad;
    double cross_track_error_m;

    double linear_velocity_mps;
    double angular_velocity_radps;
    double left_motor;
    double right_motor;
    int rotate_in_place;

    uint32_t replan_count;
    uint32_t position_loss_events;
    uint32_t invalid_sample_count;
    uint32_t plan_time_us;
    int route_length;
    int route[NAV_MAX_ROUTE_NODES];

    /// Milliseconds since the last accepted sample, or UINT64_MAX when none.
    uint64_t position_age_ms;
} nav_status;

/// Fill `out` with the current navigation status.  Returns 1 on success.
NAV_API int nav_get_status(nav_handle handle, nav_status* out);

/// Serialise the status as the documented telemetry JSON line.
/// Returns the number of bytes written (excluding the NUL terminator).
NAV_API int nav_get_telemetry_json(nav_handle handle, const char* trolley_id, uint64_t timestamp_ms,
                                   char* out, int out_size);

/// Human readable status block (the `status` command output).
NAV_API int nav_get_status_text(nav_handle handle, const char* trolley_id, char* out, int out_size);

// ---------------------------------------------------------------------------
// Stepper (NEMA 17 + A4988 class) - standalone, no navigation session needed
// ---------------------------------------------------------------------------
//
// Exposed so the simulator can integrate genuine step-based wheel odometry and
// so the conversion maths can be cross-checked from Python.  One handle is one
// axis.

/// Create a stepper axis.  `invert_direction` mirrors the DIR level.
NAV_API nav_handle nav_stepper_create(int steps_per_revolution, int microsteps,
                                      double wheel_radius_m, double max_step_rate_hz,
                                      double max_step_accel_hz_per_s, int invert_direction);

NAV_API void nav_stepper_destroy(nav_handle handle);

/// Reset position, rate and profile (keeps the configuration).
NAV_API void nav_stepper_reset(nav_handle handle);

/// Declare the current physical position as step 0.
NAV_API void nav_stepper_zero_position(nav_handle handle);

/// Continuous mode: follow a wheel linear velocity (m/s, signed).
NAV_API void nav_stepper_set_velocity(nav_handle handle, double velocity_mps);

/// Continuous mode: follow a motor RPM (signed).
NAV_API void nav_stepper_set_rpm(nav_handle handle, double rpm);

/// Position mode: drive to an absolute step position (trapezoidal profile).
NAV_API void nav_stepper_move_to(nav_handle handle, long long target_steps);

/// Position mode: move by a number of motor revolutions.
NAV_API void nav_stepper_move_revolutions(nav_handle handle, double revolutions);

/// Advance by `dt_s`; returns the STEP pulses to emit (>= 0, see direction).
NAV_API int nav_stepper_advance(nav_handle handle, double dt_s);

NAV_API long long nav_stepper_position_steps(nav_handle handle);
NAV_API long long nav_stepper_target_steps(nav_handle handle);
NAV_API double nav_stepper_distance_m(nav_handle handle);
NAV_API double nav_stepper_revolutions(nav_handle handle);
NAV_API double nav_stepper_current_rate_hz(nav_handle handle);
NAV_API double nav_stepper_current_rpm(nav_handle handle);
NAV_API int nav_stepper_direction(nav_handle handle);
NAV_API int nav_stepper_position_reached(nav_handle handle);
NAV_API unsigned long long nav_stepper_emitted_steps(nav_handle handle);

/// Pure conversion helpers.
NAV_API int nav_stepper_steps_per_revolution(int steps_per_revolution, int microsteps);
NAV_API double nav_stepper_steps_per_meter(int steps_per_revolution, int microsteps,
                                           double wheel_radius_m);
NAV_API double nav_stepper_velocity_to_rate(double velocity_mps, int steps_per_revolution,
                                            int microsteps, double wheel_radius_m);
NAV_API double nav_stepper_rate_to_rpm(double rate_hz, int steps_per_revolution, int microsteps);
NAV_API double nav_stepper_rpm_to_rate(double rpm, int steps_per_revolution, int microsteps);

// ---------------------------------------------------------------------------
// Standalone queries (no session required)
// ---------------------------------------------------------------------------

/// Plan a route with Dijkstra on the given graph JSON.  Writes up to
/// `max_nodes` node ids into `out_nodes` and returns the node count (0 on
/// failure).  `out_distance_m` receives the total distance.
NAV_API int nav_plan_route(const char* graph_json, int start_node, int destination_node,
                           int* out_nodes, int max_nodes, double* out_distance_m);

/// Nearest graph node to (x_m, y_m).  Returns the node id or -1.
NAV_API int nav_find_nearest_node(const char* graph_json, double x_m, double y_m,
                                  double max_distance_m);

/// Number of nodes/edges in the graph JSON; returns 0 when parsing fails.
NAV_API int nav_graph_node_count(const char* graph_json);
NAV_API int nav_graph_edge_count(const char* graph_json);

/// Human readable name of a NavState value (for logging from Python).
NAV_API const char* nav_state_name(int state);

#ifdef __cplusplus
}  // extern "C"
#endif
