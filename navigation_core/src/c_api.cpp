// Implementation of the flat C ABI declared in c_api.h.
//
// This file is the only place where the C++ core is wrapped for foreign
// callers; it contains no navigation logic of its own.
#include "navigation/c_api.h"

#include <cstdio>
#include <cstring>
#include <new>

#include "navigation/config_loader.hpp"
#include "navigation/dijkstra.hpp"
#include "navigation/navigation_fsm.hpp"
#include "navigation/stepper.hpp"
#include "navigation/telemetry.hpp"
#include "navigation/time_utils.hpp"

namespace {

/// Session owned by one nav_handle.
struct NavSession {
    nav::Graph graph;
    nav::NavigationConfig config;
    nav::NavigationFsm fsm;
    uint64_t last_accepted_ms{0};
    bool has_accepted{false};
};

char g_last_error[NAV_MAX_ERROR_LEN] = {};

void setLastError(const char* message) {
    nav::copyBounded(g_last_error, sizeof(g_last_error), message == nullptr ? "" : message);
}

NavSession* asSession(nav_handle handle) { return static_cast<NavSession*>(handle); }

/// Parse graph JSON into `graph`.  Returns false and fills `error` on failure.
bool parseGraph(const char* graph_json, nav::Graph& graph, char* error, std::size_t error_size) {
    if (graph_json == nullptr) {
        std::snprintf(error, error_size, "graph JSON is required");
        return false;
    }
    return nav::loadGraph(graph_json, graph, error, error_size);
}

}  // namespace

extern "C" {

nav_handle nav_create(const char* graph_json, const char* config_json) {
    setLastError("");
    auto* session = new (std::nothrow) NavSession();
    if (session == nullptr) {
        setLastError("out of memory");
        return nullptr;
    }

    char error[NAV_MAX_ERROR_LEN] = {};
    if (!parseGraph(graph_json, session->graph, error, sizeof(error))) {
        setLastError(error);
        delete session;
        return nullptr;
    }

    nav::NavigationConfig config;
    if (config_json != nullptr && config_json[0] != '\0') {
        if (!nav::loadNavigationConfig(config_json, config, error, sizeof(error))) {
            setLastError(error);
            delete session;
            return nullptr;
        }
    }
    session->config = config;

    const nav::ConfigValidation validation = session->fsm.begin(&session->graph, config);
    if (!validation.valid) {
        setLastError(validation.message);
        delete session;
        return nullptr;
    }
    return session;
}

void nav_destroy(nav_handle handle) { delete asSession(handle); }

const char* nav_last_error(void) { return g_last_error; }

int nav_request_destination(nav_handle handle, int node_id) {
    NavSession* session = asSession(handle);
    if (session == nullptr) {
        return 0;
    }
    return session->fsm.requestDestination(node_id) ? 1 : 0;
}

int nav_start(nav_handle handle) {
    NavSession* session = asSession(handle);
    if (session == nullptr) {
        return 0;
    }
    return session->fsm.startNavigation() ? 1 : 0;
}

void nav_cancel(nav_handle handle) {
    NavSession* session = asSession(handle);
    if (session != nullptr) {
        session->fsm.cancelNavigation();
    }
}

void nav_stop(nav_handle handle) {
    NavSession* session = asSession(handle);
    if (session != nullptr) {
        session->fsm.stopNavigation();
    }
}

void nav_emergency_stop(nav_handle handle) {
    NavSession* session = asSession(handle);
    if (session != nullptr) {
        session->fsm.emergencyStop();
    }
}

void nav_clear_emergency_stop(nav_handle handle) {
    NavSession* session = asSession(handle);
    if (session != nullptr) {
        session->fsm.clearEmergencyStop();
    }
}

void nav_request_replan(nav_handle handle) {
    NavSession* session = asSession(handle);
    if (session != nullptr) {
        session->fsm.requestReplan();
    }
}

void nav_reset(nav_handle handle) {
    NavSession* session = asSession(handle);
    if (session != nullptr) {
        session->fsm.reset();
        session->has_accepted = false;
        session->last_accepted_ms = 0;
    }
}

int nav_submit_position(nav_handle handle, double x_m, double y_m, double quality,
                        uint64_t timestamp_ms, int valid) {
    NavSession* session = asSession(handle);
    if (session == nullptr) {
        return static_cast<int>(nav::MeasurementStatus::NO_DATA);
    }
    nav::PositionMeasurement measurement;
    measurement.schema_version = nav::kSchemaVersion;
    measurement.setTrolleyId(nav::kDefaultTrolleyId);
    measurement.setFrameId(nav::kFrameId);
    measurement.timestamp_ms = timestamp_ms;
    measurement.position.x_m = static_cast<float>(x_m);
    measurement.position.y_m = static_cast<float>(y_m);
    measurement.quality = static_cast<float>(quality);
    measurement.valid = valid != 0;

    const uint64_t now_ms = timestamp_ms;
    const nav::MeasurementStatus status = session->fsm.submitPosition(measurement, now_ms);
    if (status == nav::MeasurementStatus::OK) {
        session->last_accepted_ms = now_ms;
        session->has_accepted = true;
    }
    return static_cast<int>(status);
}

void nav_update(nav_handle handle, uint64_t now_ms) {
    NavSession* session = asSession(handle);
    if (session != nullptr) {
        session->fsm.update(now_ms);
    }
}

int nav_get_status(nav_handle handle, nav_status* out) {
    NavSession* session = asSession(handle);
    if (session == nullptr || out == nullptr) {
        return 0;
    }
    const nav::NavigationStatus& status = session->fsm.status();

    std::memset(out, 0, sizeof(nav_status));
    out->state = static_cast<int>(status.state);
    out->error = static_cast<int>(status.error);
    out->x_m = status.position.x_m;
    out->y_m = status.position.y_m;
    out->heading_rad = status.heading_rad;
    out->heading_valid = status.heading_valid ? 1 : 0;
    out->position_quality = status.position_quality;
    out->position_valid = status.position_valid ? 1 : 0;
    out->position_fresh = status.position_fresh ? 1 : 0;

    out->start_node = status.start_node;
    out->destination_node = status.destination_node;
    out->active_waypoint_node = status.active_waypoint_node;
    out->waypoint_index = status.waypoint_index;
    out->waypoint_count = status.waypoint_count;
    out->distance_to_waypoint_m = status.distance_to_waypoint_m;
    out->distance_to_destination_m = status.distance_to_destination_m;
    out->planned_distance_m = status.planned_distance_m;
    out->remaining_distance_m = status.remaining_distance_m;
    out->travelled_distance_m = status.travelled_distance_m;

    out->target_bearing_rad = status.target_bearing_rad;
    out->heading_error_rad = status.heading_error_rad;
    out->cross_track_error_m = status.cross_track_error_m;

    out->linear_velocity_mps = status.linear_velocity_mps;
    out->angular_velocity_radps = status.angular_velocity_radps;
    out->left_motor = status.motor.left;
    out->right_motor = status.motor.right;
    out->rotate_in_place = status.rotate_in_place ? 1 : 0;

    out->replan_count = status.replan_count;
    out->position_loss_events = status.position_loss_events;
    out->invalid_sample_count = status.invalid_sample_count;
    out->plan_time_us = status.plan_time_us;
    out->route_length = status.route_length;
    for (int i = 0; i < status.route_length && i < NAV_MAX_ROUTE_NODES; ++i) {
        out->route[i] = status.route[i];
    }

    // Position age is computed from the monotonic clock owned by the host.
    if (session->has_accepted) {
        const uint64_t now = nav::monotonicMillis();
        out->position_age_ms = now >= session->last_accepted_ms ? now - session->last_accepted_ms : 0;
    } else {
        out->position_age_ms = UINT64_MAX;
    }
    return 1;
}

int nav_get_telemetry_json(nav_handle handle, const char* trolley_id, uint64_t timestamp_ms,
                           char* out, int out_size) {
    NavSession* session = asSession(handle);
    if (session == nullptr || out == nullptr || out_size <= 0) {
        return 0;
    }
    return nav::statusToJson(session->fsm.status(), trolley_id, timestamp_ms, out,
                             static_cast<std::size_t>(out_size));
}

int nav_get_status_text(nav_handle handle, const char* trolley_id, char* out, int out_size) {
    NavSession* session = asSession(handle);
    if (session == nullptr || out == nullptr || out_size <= 0) {
        return 0;
    }
    return nav::statusToText(session->fsm.status(), trolley_id, out,
                             static_cast<std::size_t>(out_size));
}

int nav_plan_route(const char* graph_json, int start_node, int destination_node, int* out_nodes,
                   int max_nodes, double* out_distance_m) {
    nav::Graph graph;
    char error[NAV_MAX_ERROR_LEN] = {};
    if (!parseGraph(graph_json, graph, error, sizeof(error))) {
        setLastError(error);
        return 0;
    }
    const nav::PathResult result = nav::dijkstra(graph, start_node, destination_node);
    if (!result.success) {
        setLastError(nav::toString(result.error));
        return 0;
    }
    const int count = result.node_count < max_nodes ? result.node_count : max_nodes;
    for (int i = 0; i < count; ++i) {
        out_nodes[i] = result.node_ids[i];
    }
    if (out_distance_m != nullptr) {
        *out_distance_m = result.total_distance_m;
    }
    return count;
}

int nav_find_nearest_node(const char* graph_json, double x_m, double y_m, double max_distance_m) {
    nav::Graph graph;
    char error[NAV_MAX_ERROR_LEN] = {};
    if (!parseGraph(graph_json, graph, error, sizeof(error))) {
        setLastError(error);
        return -1;
    }
    nav::Position2D position;
    position.x_m = static_cast<float>(x_m);
    position.y_m = static_cast<float>(y_m);
    const nav::NearestNodeResult result =
        graph.findNearestNode(position, static_cast<float>(max_distance_m));
    return result.found ? result.node_id : -1;
}

int nav_graph_node_count(const char* graph_json) {
    nav::Graph graph;
    char error[NAV_MAX_ERROR_LEN] = {};
    if (!parseGraph(graph_json, graph, error, sizeof(error))) {
        setLastError(error);
        return 0;
    }
    return graph.nodeCount();
}

int nav_graph_edge_count(const char* graph_json) {
    nav::Graph graph;
    char error[NAV_MAX_ERROR_LEN] = {};
    if (!parseGraph(graph_json, graph, error, sizeof(error))) {
        setLastError(error);
        return 0;
    }
    return graph.edgeCount();
}

// ---------------------------------------------------------------------------
// Stepper API
// ---------------------------------------------------------------------------

namespace {
nav::StepperAxis* asAxis(nav_handle handle) { return static_cast<nav::StepperAxis*>(handle); }

nav::StepperConfig makeStepperConfig(int steps_per_revolution, int microsteps,
                                     double wheel_radius_m, double max_step_rate_hz,
                                     double max_step_accel_hz_per_s, int invert_direction) {
    nav::StepperConfig config;
    config.steps_per_revolution = steps_per_revolution > 0 ? steps_per_revolution : 200;
    config.microsteps = microsteps >= 1 ? microsteps : 16;
    config.wheel_radius_m = static_cast<float>(wheel_radius_m > 0.0 ? wheel_radius_m : 0.05);
    config.max_step_rate_hz = static_cast<float>(max_step_rate_hz > 0.0 ? max_step_rate_hz : 20000.0);
    config.max_step_accel_hz_per_s = static_cast<float>(
        max_step_accel_hz_per_s > 0.0 ? max_step_accel_hz_per_s : config.max_step_rate_hz);
    config.invert_direction = invert_direction != 0;
    return config;
}
}  // namespace

nav_handle nav_stepper_create(int steps_per_revolution, int microsteps, double wheel_radius_m,
                              double max_step_rate_hz, double max_step_accel_hz_per_s,
                              int invert_direction) {
    setLastError("");
    auto* axis = new (std::nothrow) nav::StepperAxis();
    if (axis == nullptr) {
        setLastError("out of memory");
        return nullptr;
    }
    const nav::StepperConfig config = makeStepperConfig(steps_per_revolution, microsteps,
                                                       wheel_radius_m, max_step_rate_hz,
                                                       max_step_accel_hz_per_s, invert_direction);
    const nav::StepperValidation validation = nav::validateStepperConfig(config);
    if (!validation.valid) {
        setLastError(validation.message);
        delete axis;
        return nullptr;
    }
    axis->configure(config);
    return axis;
}

void nav_stepper_destroy(nav_handle handle) { delete asAxis(handle); }

void nav_stepper_reset(nav_handle handle) {
    if (nav::StepperAxis* axis = asAxis(handle)) {
        axis->reset();
    }
}

void nav_stepper_zero_position(nav_handle handle) {
    if (nav::StepperAxis* axis = asAxis(handle)) {
        axis->zeroPosition();
    }
}

void nav_stepper_set_velocity(nav_handle handle, double velocity_mps) {
    if (nav::StepperAxis* axis = asAxis(handle)) {
        axis->setContinuousMode();
        axis->setTargetRateHz(
            nav::wheelVelocityToStepRate(static_cast<float>(velocity_mps), axis->config()));
    }
}

void nav_stepper_set_rpm(nav_handle handle, double rpm) {
    if (nav::StepperAxis* axis = asAxis(handle)) {
        axis->setContinuousMode();
        axis->setTargetRpm(static_cast<float>(rpm));
    }
}

void nav_stepper_move_to(nav_handle handle, long long target_steps) {
    if (nav::StepperAxis* axis = asAxis(handle)) {
        axis->setTargetPositionSteps(static_cast<int64_t>(target_steps));
    }
}

void nav_stepper_move_revolutions(nav_handle handle, double revolutions) {
    if (nav::StepperAxis* axis = asAxis(handle)) {
        axis->moveRevolutions(static_cast<float>(revolutions));
    }
}

int nav_stepper_advance(nav_handle handle, double dt_s) {
    if (nav::StepperAxis* axis = asAxis(handle)) {
        return axis->advance(static_cast<float>(dt_s));
    }
    return 0;
}

long long nav_stepper_position_steps(nav_handle handle) {
    const nav::StepperAxis* axis = asAxis(handle);
    return axis == nullptr ? 0 : static_cast<long long>(axis->positionSteps());
}

long long nav_stepper_target_steps(nav_handle handle) {
    const nav::StepperAxis* axis = asAxis(handle);
    return axis == nullptr ? 0 : static_cast<long long>(axis->targetPositionSteps());
}

double nav_stepper_distance_m(nav_handle handle) {
    const nav::StepperAxis* axis = asAxis(handle);
    return axis == nullptr ? 0.0 : static_cast<double>(axis->distanceM());
}

double nav_stepper_revolutions(nav_handle handle) {
    const nav::StepperAxis* axis = asAxis(handle);
    return axis == nullptr ? 0.0 : static_cast<double>(axis->revolutions());
}

double nav_stepper_current_rate_hz(nav_handle handle) {
    const nav::StepperAxis* axis = asAxis(handle);
    return axis == nullptr ? 0.0 : static_cast<double>(axis->currentRateHz());
}

double nav_stepper_current_rpm(nav_handle handle) {
    const nav::StepperAxis* axis = asAxis(handle);
    return axis == nullptr ? 0.0 : static_cast<double>(axis->currentRpm());
}

int nav_stepper_direction(nav_handle handle) {
    const nav::StepperAxis* axis = asAxis(handle);
    return axis == nullptr ? 0 : axis->direction();
}

int nav_stepper_position_reached(nav_handle handle) {
    const nav::StepperAxis* axis = asAxis(handle);
    return (axis != nullptr && axis->positionReached()) ? 1 : 0;
}

unsigned long long nav_stepper_emitted_steps(nav_handle handle) {
    const nav::StepperAxis* axis = asAxis(handle);
    return axis == nullptr ? 0 : static_cast<unsigned long long>(axis->emittedSteps());
}

int nav_stepper_steps_per_revolution(int steps_per_revolution, int microsteps) {
    const nav::StepperConfig config =
        makeStepperConfig(steps_per_revolution, microsteps, 0.05, 20000.0, 20000.0, 0);
    return nav::stepsPerRevolution(config);
}

double nav_stepper_steps_per_meter(int steps_per_revolution, int microsteps,
                                   double wheel_radius_m) {
    const nav::StepperConfig config =
        makeStepperConfig(steps_per_revolution, microsteps, wheel_radius_m, 20000.0, 20000.0, 0);
    return static_cast<double>(nav::stepsPerMeter(config));
}

double nav_stepper_velocity_to_rate(double velocity_mps, int steps_per_revolution, int microsteps,
                                    double wheel_radius_m) {
    const nav::StepperConfig config =
        makeStepperConfig(steps_per_revolution, microsteps, wheel_radius_m, 20000.0, 20000.0, 0);
    return static_cast<double>(
        nav::wheelVelocityToStepRate(static_cast<float>(velocity_mps), config));
}

double nav_stepper_rate_to_rpm(double rate_hz, int steps_per_revolution, int microsteps) {
    const nav::StepperConfig config =
        makeStepperConfig(steps_per_revolution, microsteps, 0.05, 20000.0, 20000.0, 0);
    return static_cast<double>(nav::stepRateToRpm(static_cast<float>(rate_hz), config));
}

double nav_stepper_rpm_to_rate(double rpm, int steps_per_revolution, int microsteps) {
    const nav::StepperConfig config =
        makeStepperConfig(steps_per_revolution, microsteps, 0.05, 20000.0, 20000.0, 0);
    return static_cast<double>(nav::rpmToStepRate(static_cast<float>(rpm), config));
}

const char* nav_state_name(int state) {
    return nav::describeState(static_cast<nav::NavState>(state));
}

}  // extern "C"
