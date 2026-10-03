// Navigation finite state machine: the orchestrator that ties position input,
// graph snapping, Dijkstra planning, waypoint tracking, path following and
// safety together.
//
// Explicit states (see docs/architecture.md for the diagram):
//   BOOT -> IDLE -> WAITING_FOR_POSITION -> READY -> PLANNING -> NAVIGATING
//   NAVIGATING <-> REPLANNING, NAVIGATING <-> POSITION_LOST,
//   NAVIGATING -> ARRIVED, any -> EMERGENCY_STOP, failure -> ERROR
//
// Safety invariant: `status().motor` is only non-zero in NAVIGATING.  Every
// other state - including a failed plan, a stale position or an active
// emergency stop - commands zero on both wheels.
#pragma once

#include "navigation/graph.hpp"
#include "navigation/navigation_config.hpp"
#include "navigation/path_follower.hpp"
#include "navigation/position_validator.hpp"
#include "navigation/types.hpp"
#include "navigation/waypoint_manager.hpp"

namespace nav {

class NavigationFsm {
public:
    NavigationFsm() = default;

    /// Bind the map and configuration.  Returns the configuration validation
    /// result; on failure the FSM stays in ERROR and never moves the motors.
    ConfigValidation begin(const Graph* graph, const NavigationConfig& config);

    /// Reset all runtime state (keeps graph/config).  Returns to BOOT.
    void reset();

    // ---- commands ----------------------------------------------------------
    /// Request navigation to `node_id` (equivalent to the "navigate" command).
    bool requestDestination(int node_id);
    /// Resume the destination that was active before an emergency stop.
    /// This is the explicit operator action required after clearEmergencyStop();
    /// it re-arms the request so the FSM re-validates position and route.
    bool startNavigation();
    /// Cancel navigation and stop (equivalent to "cancel_navigation").
    void cancelNavigation();
    /// Immediate stop request while keeping the destination (equivalent to "stop").
    void stopNavigation();
    /// Software emergency stop: motors are cut and further commands ignored.
    void emergencyStop();
    /// Clear the emergency stop.  Never resumes motion by itself: the FSM
    /// returns to IDLE with the destination held in abeyance, and the operator
    /// must issue `startNavigation()` before the trolley moves again.
    void clearEmergencyStop();
    /// Force a replan on the next cycle (operator "replan" command).
    void requestReplan();

    /// Destination that was active when the emergency stop was engaged (-1 when
    /// none).  Exposed so the status/telemetry layer can report it.
    int heldDestination() const { return held_destination_node_; }

    // ---- inputs ------------------------------------------------------------
    /// Submit a raw UWB sample.  Validation happens here; `now_ms` must be a
    /// local monotonic timestamp.  Returns the validation outcome.
    MeasurementStatus submitPosition(const PositionMeasurement& measurement, uint64_t now_ms);

    /// Mark the position stream as interrupted (e.g. transport error).
    void notifyPositionStreamLost();

    // ---- control loop ------------------------------------------------------
    /// Run one control cycle.  Call at the configured navigation rate.
    void update(uint64_t now_ms);

    // ---- observation -------------------------------------------------------
    const NavigationStatus& status() const { return status_; }
    NavState state() const { return status_.state; }
    NavigationError lastError() const { return status_.error; }
    bool emergencyStopActive() const { return emergency_stop_; }
    const Graph* graph() const { return graph_; }
    const NavigationConfig& config() const { return config_; }
    const WaypointManager& waypoints() const { return waypoints_; }
    const PathFollower& follower() const { return follower_; }

    /// True when the trolley is allowed to actuate its motors.
    bool motionAllowed() const { return status_.state == NavState::NAVIGATING; }

private:
    /// One state-machine step.  Called repeatedly by update() until a holding
    /// state is reached.
    void step(uint64_t now_ms, bool position_fresh, float dt_s);
    void transitionTo(NavState next, uint64_t now_ms);
    void setError(NavigationError error, uint64_t now_ms);

    bool planRoute(uint64_t now_ms, bool is_replan);
    bool positionIsFresh(uint64_t now_ms) const;
    void applyPosition(const PositionMeasurement& measurement, uint64_t now_ms);
    void updatePositionFreshness(uint64_t now_ms);
    void zeroMotion();
    /// Zero the controller *tracking* telemetry when no tracking is happening
    /// (arrival, stop, E-stop) while preserving the final pose.
    void clearTrackingTelemetry();

    const Graph* graph_{nullptr};
    NavigationConfig config_{};
    bool config_valid_{false};

    NavState state_{NavState::BOOT};
    NavigationError error_{NavigationError::NONE};
    bool emergency_stop_{false};
    bool destination_pending_{false};
    bool stop_requested_{false};
    bool replan_requested_{false};
    /// Destination remembered across an emergency stop, waiting for an explicit
    /// startNavigation() before it becomes pending again.
    int held_destination_node_{-1};

    /// Latest filtered position (what navigation is currently using).
    Position2D filtered_position_{};
    bool has_position_{false};
    bool has_last_accepted_{false};
    uint64_t last_accepted_ms_{0};
    /// Timestamp of the previous update() call, used to measure the real dt.
    uint64_t last_update_ms_{0};
    bool has_updated_{false};
    MeasurementStatus last_measurement_status_{MeasurementStatus::NO_DATA};

    PathFollower follower_{};
    WaypointManager waypoints_{};
    DeviationMonitor deviation_{};
    PositionFilter filter_{};

    NavigationStatus status_{};
};

/// Human readable state description used by the status command / logs.
const char* describeState(NavState state);

}  // namespace nav
