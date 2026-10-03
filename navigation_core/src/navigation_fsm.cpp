#include "navigation/navigation_fsm.hpp"

#include <cmath>

#include "navigation/dijkstra.hpp"
#include "navigation/geometry.hpp"
#include "navigation/logging.hpp"
#include "navigation/time_utils.hpp"

namespace nav {

namespace {
constexpr const char* kTag = "NAV";
/// Deviation soft limit: linear speed is scaled by this factor while the
/// trolley is outside the corridor but a replan has not been confirmed yet.
constexpr float kDeviationSpeedScale = 0.5f;
}  // namespace

const char* describeState(NavState state) {
    switch (state) {
        case NavState::BOOT: return "BOOT";
        case NavState::IDLE: return "IDLE";
        case NavState::WAITING_FOR_POSITION: return "WAITING_FOR_POSITION";
        case NavState::READY: return "READY";
        case NavState::PLANNING: return "PLANNING";
        case NavState::NAVIGATING: return "NAVIGATING";
        case NavState::REPLANNING: return "REPLANNING";
        case NavState::POSITION_LOST: return "POSITION_LOST";
        case NavState::ARRIVED: return "ARRIVED";
        case NavState::ERROR: return "ERROR";
        case NavState::EMERGENCY_STOP: return "EMERGENCY_STOP";
        default: return "UNKNOWN";
    }
}

ConfigValidation NavigationFsm::begin(const Graph* graph, const NavigationConfig& config) {
    graph_ = graph;
    config_ = config;

    const ConfigValidation validation = validateConfig(config_);
    config_valid_ = validation.valid;

    // Push the configuration into the collaborators so they never read stale
    // parameters after a reload.
    follower_.setConfig(config_);
    waypoints_.setConfig(config_.path);
    deviation_.setConfig(config_.path);
    filter_.configure(config_.position.enable_filter, config_.position.filter_alpha);

    const uint64_t now = monotonicMillis();
    reset();

    if (!validation.valid) {
        NAV_LOG_ERROR(kTag, "configuration invalid: %s", validation.message);
        setError(NavigationError::CONFIG_INVALID, now);
        state_ = NavState::ERROR;
    } else if (graph_ == nullptr) {
        setError(NavigationError::GRAPH_INVALID, now);
        state_ = NavState::ERROR;
    } else {
        const GraphValidation graph_validation = graph_->validate();
        if (!graph_validation.valid) {
            NAV_LOG_ERROR(kTag, "graph invalid: %s", graph_validation.message());
            setError(NavigationError::GRAPH_INVALID, now);
            state_ = NavState::ERROR;
        } else {
            NAV_LOG_INFO(kTag, "navigation core ready: %d nodes, %d edges",
                         graph_->nodeCount(), graph_->edgeCount());
        }
    }
    status_.state = state_;
    return validation;
}

void NavigationFsm::reset() {
    // The emergency stop is intentionally NOT cleared here: resetting the
    // runtime state must never re-enable the motors.  clearEmergencyStop()
    // is the only way out.
    state_ = emergency_stop_ ? NavState::EMERGENCY_STOP : NavState::BOOT;
    error_ = NavigationError::NONE;
    destination_pending_ = false;
    stop_requested_ = false;
    replan_requested_ = false;

    filtered_position_ = Position2D{};
    has_position_ = false;
    has_last_accepted_ = false;
    last_accepted_ms_ = 0;
    last_measurement_status_ = MeasurementStatus::NO_DATA;

    follower_.reset();
    waypoints_.reset();
    deviation_.reset();
    filter_.reset();

    status_ = NavigationStatus{};
    status_.state = state_;
    zeroMotion();
}

// ---------------------------------------------------------------------------
// commands
// ---------------------------------------------------------------------------

bool NavigationFsm::requestDestination(int node_id) {
    if (emergency_stop_) {
        // Safety requirement: navigation commands are ignored while the
        // emergency stop is active.
        error_ = NavigationError::EMERGENCY_STOP_ACTIVE;
        status_.error = error_;
        NAV_LOG_WARN(kTag, "destination ignored: emergency stop is active");
        return false;
    }
    if (graph_ == nullptr) {
        error_ = NavigationError::GRAPH_INVALID;
        return false;
    }
    if (graph_->findNode(node_id) == nullptr) {
        // A rejected command is not a system fault: report it and stay operable.
        error_ = NavigationError::INVALID_DESTINATION;
        status_.error = error_;
        NAV_LOG_WARN(kTag, "destination rejected: node=%d does not exist", node_id);
        return false;
    }
    const bool was_navigating =
        state_ == NavState::NAVIGATING || state_ == NavState::POSITION_LOST ||
        state_ == NavState::REPLANNING;
    status_.destination_node = node_id;
    destination_pending_ = true;
    stop_requested_ = false;
    error_ = NavigationError::NONE;
    status_.error = NavigationError::NONE;
    waypoints_.reset();
    deviation_.reset();
    NAV_LOG_INFO(kTag, "destination set: node=%d", node_id);

    if (was_navigating) {
        // Changing the destination mid-route must invalidate the current plan
        // immediately; otherwise the trolley would keep driving the old route.
        replan_requested_ = true;
    } else {
        replan_requested_ = false;
        transitionTo(NavState::IDLE, monotonicMillis());
    }
    return true;
}

bool NavigationFsm::startNavigation() {
    if (emergency_stop_) {
        error_ = NavigationError::EMERGENCY_STOP_ACTIVE;
        status_.error = error_;
        NAV_LOG_WARN(kTag, "start ignored: emergency stop is active");
        return false;
    }
    // Prefer the destination held across the emergency stop, otherwise the one
    // that is already requested.
    const int node_id = held_destination_node_ >= 0 ? held_destination_node_
                                                    : status_.destination_node;
    if (node_id < 0) {
        error_ = NavigationError::INVALID_DESTINATION;
        status_.error = error_;
        return false;
    }
    return requestDestination(node_id);
}

void NavigationFsm::cancelNavigation() {
    destination_pending_ = false;
    held_destination_node_ = -1;
    status_.destination_node = -1;
    stop_requested_ = false;
    replan_requested_ = false;
    waypoints_.reset();
    deviation_.reset();
    zeroMotion();
    NAV_LOG_INFO(kTag, "navigation cancelled");
    if (state_ != NavState::EMERGENCY_STOP) {
        transitionTo(NavState::IDLE, monotonicMillis());
    }
}

void NavigationFsm::stopNavigation() {
    stop_requested_ = true;
    zeroMotion();
    waypoints_.reset();
    deviation_.reset();
    NAV_LOG_INFO(kTag, "navigation stopped (destination kept)");
    if (state_ != NavState::EMERGENCY_STOP && state_ != NavState::ERROR) {
        transitionTo(NavState::IDLE, monotonicMillis());
    }
}

void NavigationFsm::emergencyStop() {
    emergency_stop_ = true;
    // Remember what we were doing so the operator can resume explicitly.
    if (status_.destination_node >= 0) {
        held_destination_node_ = status_.destination_node;
    }
    zeroMotion();
    NAV_LOG_ERROR(kTag, "EMERGENCY STOP engaged");
    transitionTo(NavState::EMERGENCY_STOP, monotonicMillis());
}

void NavigationFsm::clearEmergencyStop() {
    if (!emergency_stop_) {
        return;
    }
    emergency_stop_ = false;
    NAV_LOG_WARN(kTag, "emergency stop cleared: awaiting explicit start before moving");
    // Never resume motion here.  The route was invalidated; the FSM walks back
    // through IDLE -> WAITING_FOR_POSITION -> READY -> PLANNING once the
    // operator issues startNavigation().
    waypoints_.reset();
    deviation_.reset();
    follower_.reset();
    destination_pending_ = false;
    stop_requested_ = false;
    replan_requested_ = false;
    error_ = NavigationError::NONE;
    status_.error = NavigationError::NONE;
    transitionTo(NavState::IDLE, monotonicMillis());
}

void NavigationFsm::requestReplan() {
    replan_requested_ = true;
    NAV_LOG_INFO(kTag, "replan requested by operator");
}

// ---------------------------------------------------------------------------
// inputs
// ---------------------------------------------------------------------------

MeasurementStatus NavigationFsm::submitPosition(const PositionMeasurement& measurement,
                                                uint64_t now_ms) {
    const MeasurementValidation validation =
        validateMeasurement(measurement, config_.position, config_.map_width_m,
                            config_.map_height_m, now_ms);
    last_measurement_status_ = validation.status;

    if (!validation.accepted) {
        ++status_.invalid_sample_count;
        // Throttle the log: a dropout produces one rejection per sample.
        if (status_.invalid_sample_count <= 3 || (status_.invalid_sample_count % 20) == 0) {
            NAV_LOG_WARN(kTag, "position rejected (%s), invalid samples=%u",
                         toString(validation.status), status_.invalid_sample_count);
        }
        return validation.status;
    }

    const Position2D previous = filtered_position_;
    filtered_position_ = filter_.apply(measurement.position);
    has_position_ = true;
    last_accepted_ms_ = now_ms;
    status_.position_quality = measurement.quality;

    // Accumulate the actually travelled distance while navigating; the ground
    // truth is never used for this, only accepted measurements.
    if (has_last_accepted_ && state_ == NavState::NAVIGATING) {
        status_.travelled_distance_m += euclideanDistance(previous, filtered_position_);
    }
    has_last_accepted_ = true;
    return MeasurementStatus::OK;
}

void NavigationFsm::notifyPositionStreamLost() {
    has_position_ = false;
    status_.position_fresh = false;
    status_.position_valid = false;
}

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------

bool NavigationFsm::positionIsFresh(uint64_t now_ms) const {
    if (!has_position_) {
        return false;
    }
    if (now_ms < last_accepted_ms_) {
        return true;  // clock went backwards: treat as fresh rather than stalling
    }
    return (now_ms - last_accepted_ms_) <= config_.position.timeout_ms;
}

void NavigationFsm::updatePositionFreshness(uint64_t now_ms) {
    const bool fresh = positionIsFresh(now_ms);
    status_.position = filtered_position_;
    status_.position_fresh = fresh;
    status_.position_valid = fresh && last_measurement_status_ == MeasurementStatus::OK;
    if (waypoints_.hasRoute()) {
        status_.distance_to_destination_m = waypoints_.distanceToDestination(filtered_position_);
    } else {
        status_.distance_to_destination_m = 0.0f;
    }
}

void NavigationFsm::clearTrackingTelemetry() {
    // The pose (position/heading) is kept: it is the final result of the run.
    status_.heading_error_rad = 0.0f;
    status_.target_bearing_rad = status_.heading_rad;
    status_.distance_to_waypoint_m = 0.0f;
    status_.cross_track_error_m = 0.0f;
    status_.rotate_in_place = false;
}

void NavigationFsm::zeroMotion() {
    status_.motor = MotorCommand{};
    status_.linear_velocity_mps = 0.0f;
    status_.angular_velocity_radps = 0.0f;
    status_.rotate_in_place = false;
}

void NavigationFsm::transitionTo(NavState next, uint64_t now_ms) {
    (void)now_ms;
    if (state_ == next) {
        return;
    }
    NAV_LOG_INFO(kTag, "State %s -> %s", describeState(state_), describeState(next));
    state_ = next;
    status_.state = next;

    switch (next) {
        case NavState::POSITION_LOST:
            ++status_.position_loss_events;
            zeroMotion();
            break;
        case NavState::ARRIVED:
        case NavState::ERROR:
        case NavState::IDLE:
        case NavState::EMERGENCY_STOP:
            zeroMotion();
            break;
        default:
            break;
    }
}

void NavigationFsm::setError(NavigationError error, uint64_t now_ms) {
    error_ = error;
    status_.error = error;
    (void)now_ms;
    NAV_LOG_ERROR(kTag, "error: %s", toString(error));
}

bool NavigationFsm::planRoute(uint64_t now_ms, bool is_replan) {
    if (graph_ == nullptr) {
        setError(NavigationError::GRAPH_INVALID, now_ms);
        zeroMotion();
        transitionTo(NavState::ERROR, now_ms);
        return false;
    }
    if (status_.destination_node < 0) {
        setError(NavigationError::INVALID_DESTINATION, now_ms);
        zeroMotion();
        transitionTo(NavState::ERROR, now_ms);
        return false;
    }
    if (!has_position_) {
        setError(NavigationError::INVALID_POSITION, now_ms);
        zeroMotion();
        transitionTo(NavState::ERROR, now_ms);
        return false;
    }

    // Snap the measured position onto the graph.  The trolley itself is never
    // teleported to the node; only the planner's start node is derived here.
    // A replan happens because the trolley is already off the corridor, so it
    // gets a wider snap radius than a fresh plan.
    const float snap_limit = is_replan ? config_.path.replan_max_graph_snap_distance_m
                                       : config_.path.max_graph_snap_distance_m;

    NearestNodeResult snap = graph_->findNearestNode(filtered_position_, snap_limit);
    if (is_replan && snap.found) {
        // The global nearest node can be one the trolley has already passed.
        // Re-snapping there would command it to drive *backwards* to a node
        // behind it, which oscillates forever between that node and the
        // destination.  In that case only, re-snap to the nearest node still
        // *ahead* on the route.
        //
        // The substitution is deliberately conditional: always preferring a node
        // ahead lets the planner jump straight to a node further along the route
        // (for example the destination itself), producing a degenerate
        // single-waypoint route that abandons the corridor.  Keeping the global
        // nearest whenever it is not behind preserves normal behaviour.
        const Waypoint* route = waypoints_.waypoints();
        const int first = waypoints_.currentIndex();
        int global_index_in_route = -1;
        for (int i = first; i < waypoints_.waypointCount(); ++i) {
            if (route[i].node_id == snap.node_id) {
                global_index_in_route = i;
                break;
            }
        }
        const bool already_passed =
            global_index_in_route < 0 && waypoints_.hasRoute() && !waypoints_.finished() &&
            waypoints_.destinationNodeId() == status_.destination_node;
        if (already_passed) {
            NearestNodeResult ahead;
            ahead.distance_m = -1.0f;
            for (int i = first; i < waypoints_.waypointCount(); ++i) {
                const Position2D candidate{route[i].x_m, route[i].y_m};
                const float distance = euclideanDistance(filtered_position_, candidate);
                if (ahead.distance_m < 0.0f || distance < ahead.distance_m) {
                    ahead.distance_m = distance;
                    ahead.node_id = route[i].node_id;
                }
            }
            if (ahead.node_id >= 0 && ahead.distance_m <= snap_limit) {
                snap = ahead;
                snap.found = true;
            }
        }
    }
    if (!snap.found) {
        // Too far from every node to be mapped safely onto the graph.
        setError(NavigationError::START_NODE_NOT_FOUND, now_ms);
        zeroMotion();
        transitionTo(NavState::ERROR, now_ms);
        return false;
    }

    // Time the planner with nanosecond resolution: on a desktop host a 12-node
    // Dijkstra completes in well under a microsecond.
    const uint64_t start_ns = monotonicNanos();
    const PathResult path = dijkstra(*graph_, snap.node_id, status_.destination_node);
    const uint64_t elapsed_ns = monotonicNanos() - start_ns;
    status_.plan_time_us = nanosToMicrosRoundedUp(elapsed_ns);

    if (!path.success) {
        setError(path.error == PathError::INVALID_DESTINATION ? NavigationError::INVALID_DESTINATION
                                                             : NavigationError::ROUTE_NOT_FOUND,
                 now_ms);
        NAV_LOG_WARN(kTag, "no route from node %d to node %d (%s)", snap.node_id,
                     status_.destination_node, toString(path.error));
        zeroMotion();
        transitionTo(NavState::ERROR, now_ms);
        return false;
    }

    Waypoint route[kMaxRouteNodes];
    int count = pathToWaypoints(*graph_, path, route, kMaxRouteNodes);
    if (count <= 0) {
        setError(NavigationError::ROUTE_NOT_FOUND, now_ms);
        zeroMotion();
        transitionTo(NavState::ERROR, now_ms);
        return false;
    }

    // Skip the start node when the trolley already sits on it, so it does not
    // drive backwards to a node centre it is standing on.
    int first = 0;
    if (count >= 2) {
        const Position2D start_node{route[0].x_m, route[0].y_m};
        if (euclideanDistance(filtered_position_, start_node) <= config_.path.waypoint_tolerance_m) {
            first = 1;
        }
    }
    const int usable = count - first;
    if (!waypoints_.loadRoute(route + first, usable, status_.destination_node)) {
        setError(NavigationError::ROUTE_NOT_FOUND, now_ms);
        zeroMotion();
        transitionTo(NavState::ERROR, now_ms);
        return false;
    }
    waypoints_.snapToNearestWaypoint(filtered_position_);
    // Remember where the route was planned from: the leg from here to the first
    // waypoint is part of the route for deviation monitoring.
    waypoints_.setEntryPoint(filtered_position_);
    // The absolute heading is deliberately NOT seeded here: with UWB-only
    // positioning the initial yaw is unknown.  The follower starts by driving
    // towards the first waypoint and derives the heading from the resulting
    // displacement (see PathFollower::updateHeading).

    status_.start_node = snap.node_id;
    // Publish the waypoint telemetry immediately: the first NAVIGATING cycle
    // must already report a valid active waypoint, not the previous route's
    // (or an empty) state.
    status_.active_waypoint_node =
        waypoints_.current() != nullptr ? waypoints_.current()->node_id : -1;
    status_.waypoint_index = waypoints_.currentIndex();
    status_.waypoint_count = waypoints_.waypointCount();
    // Measured from the trolley's current position through every waypoint, so the
    // entry leg is included: a route whose first node is dropped because the
    // trolley already stands on it must still report the leg it has to drive.
    status_.remaining_distance_m = waypoints_.remainingDistance(filtered_position_);
    if (!is_replan) {
        // Only the initial plan defines the reference distance for the run.
        status_.planned_distance_m = status_.remaining_distance_m;
    }
    status_.route_length = path.node_count;
    for (int i = 0; i < path.node_count; ++i) {
        status_.route[i] = path.node_ids[i];
    }
    if (is_replan) {
        ++status_.replan_count;
    }

    deviation_.reset();

    // Emit the route in the format used by the log examples in the spec.
    char buffer[160];
    int written = 0;
    for (int i = 0; i < path.node_count && written < static_cast<int>(sizeof(buffer)) - 8; ++i) {
        written += std::snprintf(buffer + written, sizeof(buffer) - static_cast<std::size_t>(written),
                                 i == 0 ? "%d" : " -> %d", path.node_ids[i]);
    }
    NAV_LOG_INFO("PLAN", "Route: %s", buffer);
    NAV_LOG_INFO("PLAN", "Distance: %.2f m, planning time %u us, %d nodes expanded",
                 static_cast<double>(path.total_distance_m), status_.plan_time_us,
                 path.expanded_nodes);

    transitionTo(NavState::NAVIGATING, now_ms);
    return true;
}

// ---------------------------------------------------------------------------
// control loop
// ---------------------------------------------------------------------------

void NavigationFsm::update(uint64_t now_ms) {
    // Measured elapsed time since the previous control cycle.  The control loop
    // must never assume it runs at exactly navigation_rate_hz: the simulator
    // steps the plant faster than the navigation rate, and a scheduler glitch
    // changes dt on hardware too.  dt drives the PID, the speed ramp and the
    // yaw dead reckoning, so it has to be the real elapsed time.
    float dt_s = clampControlDt(static_cast<float>(now_ms - last_update_ms_) * 0.001f, config_.control);
    if (!has_updated_) {
        dt_s = 1.0f / config_.control.navigation_rate_hz;
        has_updated_ = true;
    }
    last_update_ms_ = now_ms;

    if (emergency_stop_) {
        zeroMotion();
        if (state_ != NavState::EMERGENCY_STOP) {
            transitionTo(NavState::EMERGENCY_STOP, now_ms);
        }
        status_.state = state_;
        updatePositionFreshness(now_ms);
        return;
    }
    if (!config_valid_ || graph_ == nullptr) {
        zeroMotion();
        status_.state = state_;
        return;
    }

    const bool fresh = positionIsFresh(now_ms);

    // Pass-through states (BOOT, IDLE, WAITING_FOR_POSITION, READY, PLANNING,
    // REPLANNING) perform their work and hand over immediately, so a navigation
    // request that already has a valid position starts moving within a single
    // control cycle instead of burning one cycle per intermediate state.
    // The iteration cap keeps a pathological cycle bounded.
    constexpr int kMaxTransitionsPerCycle = 8;
    for (int iteration = 0; iteration < kMaxTransitionsPerCycle; ++iteration) {
        const NavState before = state_;
        step(now_ms, fresh, dt_s);
        status_.state = state_;
        // States that hold until the next cycle: their behaviour is time based
        // (motor commands, deviation windows) or terminal.
        switch (state_) {
            case NavState::NAVIGATING:
            case NavState::POSITION_LOST:
            case NavState::ARRIVED:
            case NavState::ERROR:
            case NavState::EMERGENCY_STOP:
                updatePositionFreshness(now_ms);
                return;
            default:
                break;
        }
        if (state_ == before) {
            break;  // no progress: waiting for an external event
        }
    }

    status_.state = state_;
    updatePositionFreshness(now_ms);
}

void NavigationFsm::step(uint64_t now_ms, bool fresh, float dt_s) {
    switch (state_) {
        case NavState::BOOT:
            transitionTo(NavState::IDLE, now_ms);
            break;

        case NavState::IDLE:
            zeroMotion();
            if (destination_pending_) {
                transitionTo(NavState::WAITING_FOR_POSITION, now_ms);
            }
            break;

        case NavState::WAITING_FOR_POSITION:
            zeroMotion();
            if (fresh && has_position_) {
                transitionTo(NavState::READY, now_ms);
            }
            break;

        case NavState::READY:
            zeroMotion();
            transitionTo(NavState::PLANNING, now_ms);
            break;

        case NavState::PLANNING:
            planRoute(now_ms, false);
            break;

        case NavState::NAVIGATING: {
            if (stop_requested_ || replan_requested_) {
                replan_requested_ = false;
                if (stop_requested_) {
                    stop_requested_ = false;
                    zeroMotion();
                    transitionTo(NavState::IDLE, now_ms);
                    break;
                }
                zeroMotion();
                transitionTo(NavState::REPLANNING, now_ms);
                break;
            }
            if (!fresh || !has_position_) {
                NAV_LOG_WARN("UWB", "position timeout (>%u ms): stopping trolley",
                             config_.position.timeout_ms);
                zeroMotion();
                transitionTo(NavState::POSITION_LOST, now_ms);
                break;
            }

            follower_.updateHeading(filtered_position_, status_.position_quality);
            const WaypointProgress progress = waypoints_.update(filtered_position_);
            if (progress.destination_reached) {
                zeroMotion();
                // The trolley has stopped, so no tracking error is being acted on.
                // Leaving the last computed heading/waypoint error in telemetry
                // would report an active-looking 32 deg error on a stopped
                // vehicle; the final pose itself is preserved.
                clearTrackingTelemetry();
                status_.remaining_distance_m = waypoints_.distanceToDestination(filtered_position_);
                NAV_LOG_INFO(kTag, "destination node %d reached (%.2f m from destination centre)",
                             status_.destination_node,
                             static_cast<double>(waypoints_.distanceToDestination(filtered_position_)));
                transitionTo(NavState::ARRIVED, now_ms);
                break;
            }
            if (progress.waypoint_reached) {
                NAV_LOG_DEBUG(kTag, "waypoint reached: node=%d, index=%d/%d",
                              progress.reached_node_id, waypoints_.currentIndex(),
                              waypoints_.waypointCount());
            }

            PathFollowerOutput output = follower_.update(filtered_position_, waypoints_, dt_s);
            if (!output.valid) {
                zeroMotion();
                break;
            }

            const bool replan = deviation_.update(output.cross_track_error_m, now_ms);
            if (replan) {
                NAV_LOG_WARN(kTag, "route deviation %.2f m confirmed for %u ms: replanning",
                             static_cast<double>(output.cross_track_error_m),
                             config_.path.route_deviation_confirm_ms);
                zeroMotion();
                transitionTo(NavState::REPLANNING, now_ms);
                break;
            }

            if (deviation_.exceededSoftLimit()) {
                // Outside the corridor but not yet confirmed: keep moving slowly
                // while the deviation timer accumulates evidence.
                output.twist.linear_mps *= kDeviationSpeedScale;
                output.motor = twistToMotorCommand(output.twist, config_.drive);
            }

            status_.motor = output.motor;
            status_.linear_velocity_mps = output.twist.linear_mps;
            status_.angular_velocity_radps = output.twist.angular_radps;
            status_.heading_rad = follower_.heading();
            status_.heading_valid = follower_.headingValid();
            status_.target_bearing_rad = output.target_bearing_rad;
            status_.heading_error_rad = output.heading_error_rad;
            status_.cross_track_error_m = output.cross_track_error_m;
            status_.distance_to_waypoint_m = output.distance_to_waypoint_m;
            status_.active_waypoint_node =
                waypoints_.current() != nullptr ? waypoints_.current()->node_id : -1;
            status_.waypoint_index = waypoints_.currentIndex();
            status_.waypoint_count = waypoints_.waypointCount();
            // Distance still to drive on the current route: recomputed every cycle
            // so it falls to ~0 on arrival instead of showing the original plan.
            status_.remaining_distance_m = waypoints_.remainingDistance(filtered_position_);
            status_.rotate_in_place = output.rotate_in_place;
            break;
        }

        case NavState::REPLANNING:
            zeroMotion();
            planRoute(now_ms, true);
            break;

        case NavState::POSITION_LOST: {
            zeroMotion();
            if (!fresh || !has_position_) {
                break;
            }
            const NearestNodeResult snap =
                graph_->findNearestNode(filtered_position_, config_.path.max_graph_snap_distance_m);
            if (!snap.found) {
                setError(NavigationError::START_NODE_NOT_FOUND, now_ms);
                transitionTo(NavState::ERROR, now_ms);
                break;
            }
            const bool route_still_valid =
                waypoints_.hasRoute() && !waypoints_.finished() &&
                snap.node_id == status_.start_node &&
                waypoints_.crossTrackError(filtered_position_) <=
                    config_.path.max_cross_track_error_m;
            if (route_still_valid) {
                NAV_LOG_INFO(kTag, "position recovered at node %d: resuming route", snap.node_id);
                transitionTo(NavState::NAVIGATING, now_ms);
            } else {
                NAV_LOG_INFO(kTag, "position recovered at node %d: replanning required",
                             snap.node_id);
                transitionTo(NavState::REPLANNING, now_ms);
            }
            break;
        }

        case NavState::ARRIVED:
        case NavState::ERROR:
        case NavState::EMERGENCY_STOP:
        default:
            zeroMotion();
            break;
    }
}

}  // namespace nav
