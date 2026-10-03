// Unit tests: waypoint tracking, deviation detection, the navigation FSM and
// the JSON position parser.
#include <unity.h>

#include <cmath>

#include "navigation/config_loader.hpp"
#include "navigation/geometry.hpp"
#include "navigation/json_parser.hpp"
#include "navigation/navigation_fsm.hpp"
#include "navigation/path_follower.hpp"
#include "navigation/position_provider.hpp"
#include "navigation/telemetry.hpp"
#include "navigation/waypoint_manager.hpp"

using namespace nav;

namespace {

/// 12-node Smart Market grid, identical to config/graph.json.
Graph buildMarketGraph() {
    Graph graph;
    const float xs[4] = {1.5f, 4.5f, 7.5f, 10.5f};
    const float ys[3] = {2.0f, 4.0f, 6.0f};
    int id = 1;
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 4; ++col) {
            graph.addNode(id++, xs[col], ys[row], NodeType::INTERSECTION);
        }
    }
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            const int base = row * 4 + col + 1;
            graph.addEdge(base, base + 1);
        }
    }
    for (int col = 0; col < 4; ++col) {
        graph.addEdge(col + 1, col + 5);
        graph.addEdge(col + 5, col + 9);
    }
    return graph;
}

WaypointManager buildRoute(const Waypoint* waypoints, int count, int destination_id) {
    WaypointManager manager;
    manager.loadRoute(waypoints, count, destination_id);
    return manager;
}

/// Fixed 3-waypoint L-shaped route used by several tests.
void buildLRoute(Waypoint* out) {
    out[0] = Waypoint{1, 0.0f, 0.0f};
    out[1] = Waypoint{2, 3.0f, 0.0f};
    out[2] = Waypoint{3, 3.0f, 3.0f};
}

PositionMeasurement makeMeasurement(float x_m, float y_m, float quality = 0.95f,
                                    uint64_t timestamp_ms = 0, bool valid = true) {
    PositionMeasurement measurement;
    measurement.position.x_m = x_m;
    measurement.position.y_m = y_m;
    measurement.quality = quality;
    measurement.timestamp_ms = timestamp_ms;
    measurement.valid = valid;
    return measurement;
}

/// Status produced by validating a measurement (keeps the assertions short).
MeasurementStatus validateStatus(float x_m, float y_m, float quality, uint64_t timestamp_ms,
                                 bool valid, const PositionConfig& config, uint64_t now) {
    return validateMeasurement(makeMeasurement(x_m, y_m, quality, timestamp_ms, valid), config,
                               12.0f, 8.0f, now)
        .status;
}


}  // namespace

void setUp() {}
void tearDown() {}

// ---------------------------------------------------------------------------
// Waypoint manager
// ---------------------------------------------------------------------------

static void test_waypoint_progression() {
    Waypoint route[3];
    buildLRoute(route);
    WaypointManager manager = buildRoute(route, 3, 3);

    TEST_ASSERT_EQUAL_INT(3, manager.waypointCount());
    TEST_ASSERT_EQUAL_INT(0, manager.currentIndex());
    TEST_ASSERT_EQUAL_INT(1, manager.current()->node_id);

    // Far from every waypoint: nothing advances.
    WaypointProgress progress = manager.update(Position2D{1.5f, 1.5f});
    TEST_ASSERT_FALSE(progress.waypoint_reached);
    TEST_ASSERT_EQUAL_INT(0, manager.currentIndex());

    // Inside the acceptance radius of waypoint 1 (tolerance 0.20 m).
    progress = manager.update(Position2D{0.05f, 0.0f});
    TEST_ASSERT_TRUE(progress.waypoint_reached);
    TEST_ASSERT_EQUAL_INT(1, progress.reached_node_id);
    TEST_ASSERT_FALSE(progress.destination_reached);
    TEST_ASSERT_EQUAL_INT(1, manager.currentIndex());
    TEST_ASSERT_EQUAL_INT(2, manager.current()->node_id);

    // Waypoint 2.
    manager.update(Position2D{3.0f, 0.1f});
    TEST_ASSERT_EQUAL_INT(2, manager.currentIndex());
    TEST_ASSERT_EQUAL_INT(3, manager.current()->node_id);
    TEST_ASSERT_FALSE(manager.finished());

    // Destination: accepted by radius.
    progress = manager.update(Position2D{3.05f, 2.95f});
    TEST_ASSERT_TRUE(progress.destination_reached);
    TEST_ASSERT_TRUE(manager.finished());
}

static void test_destination_detection_uses_tighter_tolerance() {
    Waypoint route[2] = {Waypoint{1, 0.0f, 0.0f}, Waypoint{2, 2.0f, 0.0f}};
    WaypointManager manager = buildRoute(route, 2, 2);
    // 0.18 m from the destination: outside the 0.15 m destination tolerance.
    WaypointProgress progress = manager.update(Position2D{2.18f, 0.0f});
    TEST_ASSERT_FALSE(progress.destination_reached);
    TEST_ASSERT_FALSE(manager.finished());
    // Inside it.
    progress = manager.update(Position2D{2.10f, 0.0f});
    TEST_ASSERT_TRUE(progress.destination_reached);
}

static void test_single_node_route_is_immediately_completable() {
    Waypoint route[1] = {Waypoint{5, 1.0f, 1.0f}};
    WaypointManager manager = buildRoute(route, 1, 5);
    TEST_ASSERT_TRUE(manager.hasRoute());
    const WaypointProgress progress = manager.update(Position2D{1.05f, 1.05f});
    TEST_ASSERT_TRUE(progress.destination_reached);
}

static void test_snap_to_nearest_waypoint_skips_leading_nodes() {
    Waypoint route[3];
    buildLRoute(route);
    WaypointManager manager = buildRoute(route, 3, 3);
    // The trolley starts exactly on node 1: the follower must not drive back to it.
    manager.snapToNearestWaypoint(Position2D{0.05f, 0.0f});
    TEST_ASSERT_EQUAL_INT(1, manager.currentIndex());
    TEST_ASSERT_EQUAL_INT(2, manager.current()->node_id);

    // The destination itself is never skipped by snapping: only the radius test
    // in update() may complete it.
    manager.snapToNearestWaypoint(Position2D{3.0f, 3.0f});
    TEST_ASSERT_EQUAL_INT(1, manager.currentIndex());
    TEST_ASSERT_FALSE(manager.finished());
    TEST_ASSERT_TRUE(manager.update(Position2D{3.0f, 3.0f}).destination_reached);
}

static void test_waypoint_passed_plane_detection() {
    Waypoint route[3];
    buildLRoute(route);
    WaypointManager manager = buildRoute(route, 3, 3);
    // 0.5 m past waypoint 1 along the corridor, still on the line: the follower
    // must advance rather than turn around.
    const WaypointProgress progress = manager.update(Position2D{0.6f, 0.02f});
    TEST_ASSERT_TRUE(progress.waypoint_reached);
    TEST_ASSERT_EQUAL_INT(1, manager.currentIndex());

    // Noise alone must not advance the route: a point behind the waypoint.
    WaypointManager second = buildRoute(route, 3, 3);
    const WaypointProgress behind = second.update(Position2D{-0.25f, 0.0f});
    TEST_ASSERT_FALSE(behind.waypoint_reached);
    TEST_ASSERT_EQUAL_INT(0, second.currentIndex());
}

static void test_remaining_distance_and_cross_track() {
    Waypoint route[3];
    buildLRoute(route);
    WaypointManager manager = buildRoute(route, 3, 3);
    const Position2D position{1.0f, 0.0f};
    // (1,0) is 1.0 m from waypoint 1 (the start node), i.e. outside the 0.20 m
    // snap radius, so the whole route is measured from the trolley:
    // 1.0 m to waypoint 1 + 3.0 m to waypoint 2 + 3.0 m to waypoint 3 = 7.0 m.
    manager.snapToNearestWaypoint(position);
    TEST_ASSERT_EQUAL_INT(0, manager.currentIndex());
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 7.0f, manager.remainingDistance(position));

    // Snapping from inside the radius of the start node consumes it: the
    // remaining distance is now measured to waypoint 2 only.
    manager.snapToNearestWaypoint(Position2D{0.10f, 0.0f});
    TEST_ASSERT_EQUAL_INT(1, manager.currentIndex());
    // 2.9 m to waypoint 2 + 3.0 m to waypoint 3.
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 5.9f, manager.remainingDistance(Position2D{0.10f, 0.0f}));

    // `activeSegmentIndex()` is the leg *leading into* the current waypoint: it
    // is what the trolley is actually traversing, so after snapping past
    // waypoint 1 the active leg is waypoint 1 -> waypoint 2 (the X axis).
    TEST_ASSERT_EQUAL_INT(0, manager.activeSegmentIndex());

    // Cross-track error is the minimum distance to any leg still to be driven.
    // (1.0, 0.5) lies 0.5 m off the X-axis leg, so that is the reported error
    // even though the later vertical leg is 2.0 m away.
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.5f, manager.crossTrackError(Position2D{1.0f, 0.5f}));

    // A point near the final leg is measured against that leg instead.
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.5f, manager.crossTrackError(Position2D{3.5f, 1.5f}));

    WaypointManager unsnapped = buildRoute(route, 3, 3);
    TEST_ASSERT_EQUAL_INT(0, unsnapped.activeSegmentIndex());
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.5f, unsnapped.crossTrackError(Position2D{1.0f, 0.5f}));

    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 3.6055f, manager.distanceToDestination(position));
}

static void test_lookahead_target_point() {
    Waypoint route[3];
    buildLRoute(route);
    PathConfig config;
    config.enable_lookahead = true;
    config.lookahead_distance_m = 1.0f;
    WaypointManager manager(config);
    manager.loadRoute(route, 3, 3);

    // Disabled by default: the target is the node centre.
    WaypointManager plain = buildRoute(route, 3, 3);
    const Position2D plain_target = plain.targetPoint(Position2D{0.5f, 0.0f});
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, plain_target.x_m);

    // Enabled: the target is 1.0 m ahead of the projection onto the segment.
    const Position2D target = manager.targetPoint(Position2D{0.5f, 0.0f});
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.5f, target.x_m);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, target.y_m);

    // Near the end of the segment the lookahead point clamps to the next node.
    const Position2D clamped = manager.targetPoint(Position2D{2.8f, 0.0f});
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 3.0f, clamped.x_m);
}

// ---------------------------------------------------------------------------
// Deviation detection and replanning trigger
// ---------------------------------------------------------------------------

static void test_deviation_requires_confirmation_window() {
    PathConfig config;
    config.max_cross_track_error_m = 0.40f;
    config.replan_cross_track_error_m = 0.60f;
    config.route_deviation_confirm_ms = 500;
    config.replan_cooldown_ms = 1000;
    DeviationMonitor monitor(config);

    // Below the soft limit.
    TEST_ASSERT_FALSE(monitor.update(0.20f, 1000));
    TEST_ASSERT_FALSE(monitor.exceededSoftLimit());
    TEST_ASSERT_FALSE(monitor.confirmed());

    // Above the soft limit but below the replan limit.
    TEST_ASSERT_FALSE(monitor.update(0.50f, 1100));
    TEST_ASSERT_TRUE(monitor.exceededSoftLimit());
    TEST_ASSERT_FALSE(monitor.confirmed());

    // Above the replan limit: no trigger until the confirmation window elapses.
    TEST_ASSERT_FALSE(monitor.update(0.80f, 1200));
    TEST_ASSERT_FALSE(monitor.update(0.80f, 1500));
    TEST_ASSERT_FALSE(monitor.confirmed());
    TEST_ASSERT_EQUAL_UINT32(300, monitor.pendingDurationMs(1500));

    // 500 ms after the deviation started -> trigger.
    TEST_ASSERT_TRUE(monitor.update(0.80f, 1700));
    TEST_ASSERT_TRUE(monitor.confirmed());
    TEST_ASSERT_EQUAL_UINT32(1, monitor.triggerCount());
}

static void test_deviation_noise_spike_does_not_replan() {
    PathConfig config;
    config.replan_cross_track_error_m = 0.60f;
    config.route_deviation_confirm_ms = 500;
    DeviationMonitor monitor(config);
    // One noisy sample, then back inside the corridor.
    TEST_ASSERT_FALSE(monitor.update(1.50f, 1000));
    TEST_ASSERT_FALSE(monitor.update(0.10f, 1100));
    TEST_ASSERT_FALSE(monitor.update(1.50f, 1200));
    TEST_ASSERT_FALSE(monitor.update(0.10f, 1300));
    TEST_ASSERT_FALSE(monitor.confirmed());
    TEST_ASSERT_EQUAL_UINT32(0, monitor.triggerCount());
}

static void test_deviation_respects_replan_cooldown() {
    PathConfig config;
    config.replan_cross_track_error_m = 0.60f;
    config.route_deviation_confirm_ms = 200;
    config.replan_cooldown_ms = 1000;
    DeviationMonitor monitor(config);
    TEST_ASSERT_FALSE(monitor.update(1.0f, 1000));
    TEST_ASSERT_TRUE(monitor.update(1.0f, 1200));   // first trigger
    TEST_ASSERT_FALSE(monitor.update(1.0f, 1400));  // inside the cooldown
    TEST_ASSERT_FALSE(monitor.update(1.0f, 1600));
    TEST_ASSERT_EQUAL_UINT32(1, monitor.triggerCount());
    // After the cooldown and a fresh confirmation window: trigger again.
    TEST_ASSERT_TRUE(monitor.update(1.0f, 2500));
    TEST_ASSERT_EQUAL_UINT32(2, monitor.triggerCount());
}

// ---------------------------------------------------------------------------
// Path follower
// ---------------------------------------------------------------------------

static void test_heading_estimation_ignores_small_displacement() {
    NavigationConfig config;
    PathFollower follower(config);
    TEST_ASSERT_FALSE(follower.headingValid());

    const float baseline = config.position.heading_min_displacement_m;
    follower.updateHeading(Position2D{0.0f, 0.0f});   // establishes the reference
    TEST_ASSERT_FALSE(follower.headingValid());

    // Half the baseline: heading stays unset AND the reference point is kept, so
    // the displacement accumulates across samples.
    follower.updateHeading(Position2D{0.0f, baseline * 0.5f});
    TEST_ASSERT_FALSE(follower.headingValid());

    // A displacement just past the baseline is trusted: heading = +pi/2.
    follower.updateHeading(Position2D{0.0f, baseline * 1.1f});
    TEST_ASSERT_TRUE(follower.headingValid());
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, kPi / 2.0f, follower.heading());

    // A further pure +Y move keeps pi/2 (the correction is blended, not reset).
    follower.updateHeading(Position2D{0.0f, baseline * 2.2f});
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, kPi / 2.0f, follower.heading());

    // A NaN sample must be ignored entirely.
    follower.updateHeading(Position2D{NAN, NAN});
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, kPi / 2.0f, follower.heading());

    // A -X move is a 180 deg reversal; the blend moves the estimate part way,
    // never snapping instantly to the noisy measurement.
    follower.updateHeading(Position2D{-baseline * 1.2f, baseline * 2.2f});
    TEST_ASSERT_TRUE(std::fabs(shortestAngularDistance(kPi / 2.0f, follower.heading())) > 0.0f);
    TEST_ASSERT_TRUE(std::fabs(shortestAngularDistance(kPi / 2.0f, follower.heading())) <= kPi);
}

static void test_follower_rotates_in_place_for_large_heading_error() {
    NavigationConfig config;
    PathFollower follower(config);
    Waypoint route[2] = {Waypoint{1, 1.0f, 0.0f}, Waypoint{2, 5.0f, 0.0f}};
    WaypointManager manager = buildRoute(route, 2, 2);

    // Establish a heading of -pi/2 (facing -Y) while the target is at +X.
    const float baseline = config.position.heading_min_displacement_m;
    follower.updateHeading(Position2D{0.0f, 0.0f});
    follower.updateHeading(Position2D{0.0f, -baseline * 1.1f});
    TEST_ASSERT_TRUE(follower.headingValid());
    const PathFollowerOutput output = follower.update(Position2D{0.0f, 0.0f}, manager, 0.05f);
    TEST_ASSERT_TRUE(output.valid);
    TEST_ASSERT_TRUE(output.rotate_in_place);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output.twist.linear_mps);  // no forward motion while turning
    TEST_ASSERT_TRUE(output.twist.angular_radps != 0.0f);
    // The first cycle is rate limited, so omega cannot jump to the maximum.
    TEST_ASSERT_TRUE(std::fabs(output.twist.angular_radps) <= config.motion.max_angular_speed_radps);
}

static void test_follower_drives_forward_when_aligned() {
    NavigationConfig config;
    PathFollower follower(config);
    Waypoint route[2] = {Waypoint{1, 1.0f, 0.0f}, Waypoint{2, 5.0f, 0.0f}};
    WaypointManager manager = buildRoute(route, 2, 2);

    follower.updateHeading(Position2D{0.0f, 0.0f});
    follower.updateHeading(Position2D{0.20f, 0.0f});  // heading = 0, aligned with +X
    PathFollowerOutput output = follower.update(Position2D{0.20f, 0.0f}, manager, 0.05f);
    TEST_ASSERT_TRUE(output.valid);
    TEST_ASSERT_FALSE(output.rotate_in_place);
    TEST_ASSERT_TRUE(output.twist.linear_mps > 0.0f);
    TEST_ASSERT_TRUE(std::fabs(output.twist.angular_radps) < 0.2f);
    TEST_ASSERT_TRUE(output.motor.left > 0.0f);
    TEST_ASSERT_TRUE(output.motor.right > 0.0f);

    // Speed ramps up over successive cycles instead of stepping instantly.
    const float first = output.twist.linear_mps;
    for (int i = 0; i < 20; ++i) {
        output = follower.update(Position2D{0.20f, 0.0f}, manager, 0.05f);
    }
    TEST_ASSERT_TRUE(output.twist.linear_mps > first);
    TEST_ASSERT_TRUE(output.twist.linear_mps <= config.motion.max_linear_speed_mps + 1e-5f);
}

static void test_follower_reports_zero_for_invalid_position() {
    NavigationConfig config;
    PathFollower follower(config);
    Waypoint route[2] = {Waypoint{1, 1.0f, 0.0f}, Waypoint{2, 5.0f, 0.0f}};
    WaypointManager manager = buildRoute(route, 2, 2);
    const PathFollowerOutput output = follower.update(Position2D{NAN, 0.0f}, manager, 0.05f);
    TEST_ASSERT_FALSE(output.valid);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output.motor.left);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output.motor.right);
}

// ---------------------------------------------------------------------------
// Position validation
// ---------------------------------------------------------------------------

static void test_position_validation_rules() {
    PositionConfig config;
    config.minimum_quality = 0.60f;
    config.map_margin_m = 1.0f;
    config.stale_sample_ms = 1000;
    const uint64_t now = 10000;

    TEST_ASSERT_TRUE(
        validateMeasurement(makeMeasurement(1.0f, 1.0f, 0.9f, now), config, 12.0f, 8.0f, now)
            .accepted);

    // valid == false
    TEST_ASSERT_EQUAL_INT(static_cast<int>(MeasurementStatus::INVALID_FLAG),
                          static_cast<int>(validateStatus(1, 1, 0.9f, now, false, config, now)));
    // Low quality
    TEST_ASSERT_EQUAL_INT(static_cast<int>(MeasurementStatus::LOW_QUALITY),
                          static_cast<int>(validateStatus(1, 1, 0.5f, now, true, config, now)));
    // Wrong frame
    PositionMeasurement wrong_frame = makeMeasurement(1, 1, 0.9f, now);
    wrong_frame.setFrameId("map");
    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(MeasurementStatus::WRONG_FRAME_ID),
        static_cast<int>(validateMeasurement(wrong_frame, config, 12, 8, now).status));
    // Wrong schema
    PositionMeasurement wrong_schema = makeMeasurement(1, 1, 0.9f, now);
    wrong_schema.schema_version = 2;
    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(MeasurementStatus::BAD_SCHEMA_VERSION),
        static_cast<int>(validateMeasurement(wrong_schema, config, 12, 8, now).status));
    // NaN coordinate
    TEST_ASSERT_EQUAL_INT(static_cast<int>(MeasurementStatus::NON_FINITE_COORDINATE),
                          static_cast<int>(validateStatus(NAN, 1, 0.9f, now, true, config, now)));
    // Infinity coordinate
    TEST_ASSERT_EQUAL_INT(static_cast<int>(MeasurementStatus::NON_FINITE_COORDINATE),
                          static_cast<int>(validateStatus(1, INFINITY, 0.9f, now, true, config, now)));
    // Outside the map + margin (12 + 1 = 13)
    TEST_ASSERT_EQUAL_INT(static_cast<int>(MeasurementStatus::OUT_OF_MAP_BOUNDS),
                          static_cast<int>(validateStatus(14.0f, 1, 0.9f, now, true, config, now)));
    // Stale sample (now - timestamp > 1000)
    TEST_ASSERT_EQUAL_INT(static_cast<int>(MeasurementStatus::STALE_TIMESTAMP),
                          static_cast<int>(validateStatus(1, 1, 0.9f, now - 1500, true, config, now)));
    // Timestamp zero
    TEST_ASSERT_EQUAL_INT(static_cast<int>(MeasurementStatus::STALE_TIMESTAMP),
                          static_cast<int>(validateStatus(1, 1, 0.9f, 0, true, config, now)));
}

static void test_position_filter_is_disabled_by_default() {
    PositionFilter filter;
    filter.configure(false, 0.5f);
    TEST_ASSERT_FALSE(filter.enabled());
    const Position2D out = filter.apply(Position2D{1.0f, 2.0f});
    TEST_ASSERT_EQUAL_FLOAT(1.0f, out.x_m);

    filter.configure(true, 0.5f);
    filter.apply(Position2D{0.0f, 0.0f});
    const Position2D smoothed = filter.apply(Position2D{2.0f, 2.0f});
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, smoothed.x_m);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, smoothed.y_m);
}

// ---------------------------------------------------------------------------
// Navigation FSM
// ---------------------------------------------------------------------------

static void test_fsm_boot_to_planning_sequence() {
    const Graph graph = buildMarketGraph();
    NavigationConfig config;
    NavigationFsm fsm;
    TEST_ASSERT_TRUE(fsm.begin(&graph, config).valid);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(NavState::BOOT), static_cast<int>(fsm.state()));

    uint64_t now = 1000;
    fsm.update(now);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(NavState::IDLE), static_cast<int>(fsm.state()));

    // Destination set but no position yet -> WAITING_FOR_POSITION.
    TEST_ASSERT_TRUE(fsm.requestDestination(11));
    fsm.update(now += 50);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(NavState::WAITING_FOR_POSITION),
                          static_cast<int>(fsm.state()));

    // A valid position is enough for a single update() call to walk
    // WAITING_FOR_POSITION -> READY -> PLANNING -> NAVIGATING.
    TEST_ASSERT_EQUAL_INT(static_cast<int>(MeasurementStatus::OK),
                          static_cast<int>(fsm.submitPosition(makeMeasurement(0.8f, 0.9f, 0.95f, now), now)));
    fsm.update(now += 50);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(NavState::NAVIGATING), static_cast<int>(fsm.state()));

    // Deterministic development route from (0.8, 0.9): the position snaps to
    // node 1 and the shortest route to node 11 is 1 -> 2 -> 3 -> 7 -> 11
    // (3.0 + 3.0 + 2.0 + 2.0 = 10.0 m).  The equally long alternative
    // 1 -> 2 -> 6 -> 7 -> 11 is rejected by the deterministic tie-break in
    // dijkstra() (node 3 < node 6 as the predecessor of node 7).
    const NavigationStatus& status = fsm.status();
    TEST_ASSERT_EQUAL_INT(1, status.start_node);
    TEST_ASSERT_EQUAL_INT(11, status.destination_node);
    TEST_ASSERT_EQUAL_INT(5, status.route_length);
    TEST_ASSERT_EQUAL_INT(1, status.route[0]);
    TEST_ASSERT_EQUAL_INT(2, status.route[1]);
    TEST_ASSERT_EQUAL_INT(3, status.route[2]);
    TEST_ASSERT_EQUAL_INT(7, status.route[3]);
    TEST_ASSERT_EQUAL_INT(11, status.route[4]);
    // 1.304 m entry leg from (0.8, 0.9) to node 1, plus 10.00 m of route.
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 11.304f, status.planned_distance_m);
    TEST_ASSERT_TRUE(status.plan_time_us > 0);
}

static void test_fsm_invalid_destination_is_rejected() {
    const Graph graph = buildMarketGraph();
    NavigationConfig config;
    NavigationFsm fsm;
    fsm.begin(&graph, config);
    TEST_ASSERT_FALSE(fsm.requestDestination(999));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(NavigationError::INVALID_DESTINATION),
                          static_cast<int>(fsm.lastError()));
    // The system stays operable: a valid destination is still accepted.
    TEST_ASSERT_TRUE(fsm.requestDestination(3));
}

static void test_fsm_unreachable_destination_reports_error() {
    Graph graph = buildMarketGraph();
    graph.addNode(90, 40.0f, 40.0f);  // isolated node
    NavigationConfig config;
    NavigationFsm fsm;
    fsm.begin(&graph, config);
    uint64_t now = 1000;
    fsm.update(now);
    TEST_ASSERT_TRUE(fsm.requestDestination(90));
    fsm.submitPosition(makeMeasurement(0.8f, 0.9f, 0.95f, now), now);
    for (int i = 0; i < 4; ++i) {
        fsm.update(now += 50);
    }
    TEST_ASSERT_EQUAL_INT(static_cast<int>(NavState::ERROR), static_cast<int>(fsm.state()));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(NavigationError::ROUTE_NOT_FOUND),
                          static_cast<int>(fsm.lastError()));
    // Safety: no motor output in ERROR.
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fsm.status().motor.left);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fsm.status().motor.right);
}

static void test_fsm_position_timeout_stops_motors() {
    const Graph graph = buildMarketGraph();
    NavigationConfig config;
    config.position.timeout_ms = 750;
    NavigationFsm fsm;
    fsm.begin(&graph, config);

    uint64_t now = 1000;
    fsm.update(now);
    fsm.requestDestination(11);
    fsm.submitPosition(makeMeasurement(0.8f, 0.9f, 0.95f, now), now);
    fsm.update(now += 50);  // READY -> PLANNING -> NAVIGATING in one cycle
    TEST_ASSERT_EQUAL_INT(static_cast<int>(NavState::NAVIGATING), static_cast<int>(fsm.state()));

    // Keep the position fresh long enough for the trolley to start moving.
    for (int i = 0; i < 20; ++i) {
        now += 50;
        fsm.submitPosition(makeMeasurement(0.8f + 0.02f * i, 0.9f, 0.95f, now), now);
        fsm.update(now);
    }
    TEST_ASSERT_TRUE(fsm.status().motor.left != 0.0f || fsm.status().motor.right != 0.0f);

    // Stop feeding positions: after the timeout the FSM must stop the motors.
    for (int i = 0; i < 30; ++i) {
        now += 50;
        fsm.update(now);
    }
    TEST_ASSERT_EQUAL_INT(static_cast<int>(NavState::POSITION_LOST), static_cast<int>(fsm.state()));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fsm.status().motor.left);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fsm.status().motor.right);
    TEST_ASSERT_EQUAL_UINT32(1, fsm.status().position_loss_events);

    // Position recovers on the same route: navigation resumes.
    fsm.submitPosition(makeMeasurement(1.2f, 0.9f, 0.95f, now), now);
    fsm.update(now += 50);
    TEST_ASSERT_TRUE(fsm.state() == NavState::NAVIGATING || fsm.state() == NavState::REPLANNING);
}

static void test_fsm_emergency_stop_and_clear() {
    const Graph graph = buildMarketGraph();
    NavigationConfig config;
    NavigationFsm fsm;
    fsm.begin(&graph, config);
    uint64_t now = 1000;
    fsm.update(now);
    fsm.requestDestination(11);
    fsm.submitPosition(makeMeasurement(0.8f, 0.9f, 0.95f, now), now);
    fsm.update(now += 50);
    fsm.update(now += 50);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(NavState::NAVIGATING), static_cast<int>(fsm.state()));

    fsm.emergencyStop();
    TEST_ASSERT_TRUE(fsm.emergencyStopActive());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(NavState::EMERGENCY_STOP), static_cast<int>(fsm.state()));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fsm.status().motor.left);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fsm.status().motor.right);

    // Commands are ignored while the emergency stop is active.
    TEST_ASSERT_FALSE(fsm.requestDestination(3));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(NavigationError::EMERGENCY_STOP_ACTIVE),
                          static_cast<int>(fsm.lastError()));
    for (int i = 0; i < 10; ++i) {
        fsm.submitPosition(makeMeasurement(0.8f, 0.9f, 0.95f, now), now);
        fsm.update(now += 50);
    }
    TEST_ASSERT_EQUAL_INT(static_cast<int>(NavState::EMERGENCY_STOP), static_cast<int>(fsm.state()));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fsm.status().motor.left);
    TEST_ASSERT_FALSE(fsm.motionAllowed());

    // Clearing must not resume motion without re-validating position and route.
    fsm.clearEmergencyStop();
    TEST_ASSERT_FALSE(fsm.emergencyStopActive());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(NavState::IDLE), static_cast<int>(fsm.state()));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fsm.status().motor.left);
    TEST_ASSERT_EQUAL_INT(11, fsm.heldDestination());  // resumable via startNavigation()

    // startNavigation() re-arms the held destination and re-plans.
    TEST_ASSERT_TRUE(fsm.startNavigation());
    fsm.submitPosition(makeMeasurement(0.8f, 0.9f, 0.95f, now), now);
    fsm.update(now += 50);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(NavState::NAVIGATING), static_cast<int>(fsm.state()));
    TEST_ASSERT_EQUAL_INT(11, fsm.status().destination_node);

    // startNavigation() with no held destination is rejected.
    NavigationFsm fresh;
    fresh.begin(&graph, config);
    TEST_ASSERT_FALSE(fresh.startNavigation());
}

static void test_fsm_cancel_and_stop_commands() {
    const Graph graph = buildMarketGraph();
    NavigationConfig config;
    NavigationFsm fsm;
    fsm.begin(&graph, config);
    uint64_t now = 1000;
    fsm.update(now);
    fsm.requestDestination(11);
    fsm.submitPosition(makeMeasurement(0.8f, 0.9f, 0.95f, now), now);
    fsm.update(now += 50);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(NavState::NAVIGATING), static_cast<int>(fsm.state()));

    fsm.cancelNavigation();
    TEST_ASSERT_EQUAL_INT(static_cast<int>(NavState::IDLE), static_cast<int>(fsm.state()));
    TEST_ASSERT_EQUAL_INT(-1, fsm.status().destination_node);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fsm.status().motor.left);
}

static void test_fsm_invalid_configuration_blocks_motion() {
    const Graph graph = buildMarketGraph();
    NavigationConfig config;
    config.drive.wheel_base_m = 0.0f;  // invalid
    NavigationFsm fsm;
    const ConfigValidation validation = fsm.begin(&graph, config);
    TEST_ASSERT_FALSE(validation.valid);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(NavState::ERROR), static_cast<int>(fsm.state()));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(NavigationError::CONFIG_INVALID),
                          static_cast<int>(fsm.lastError()));
    fsm.requestDestination(11);
    fsm.update(2000);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fsm.status().motor.left);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, fsm.status().motor.right);
}

static void test_fsm_snap_distance_limit_reports_error() {
    const Graph graph = buildMarketGraph();
    NavigationConfig config;
    config.path.max_graph_snap_distance_m = 1.5f;
    NavigationFsm fsm;
    fsm.begin(&graph, config);
    uint64_t now = 1000;
    fsm.update(now);
    fsm.requestDestination(11);
    // 5 m away from every node: cannot be mapped onto the graph.
    fsm.submitPosition(makeMeasurement(1.5f, 0.0f, 0.95f, now), now);
    fsm.update(now += 50);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(NavState::ERROR), static_cast<int>(fsm.state()));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(NavigationError::START_NODE_NOT_FOUND),
                          static_cast<int>(fsm.lastError()));
}

static void test_fsm_replans_after_confirmed_deviation() {
    const Graph graph = buildMarketGraph();
    NavigationConfig config;
    config.path.max_cross_track_error_m = 0.40f;
    config.path.replan_cross_track_error_m = 0.60f;
    config.path.route_deviation_confirm_ms = 200;
    config.path.replan_cooldown_ms = 0;
    NavigationFsm fsm;
    fsm.begin(&graph, config);

    uint64_t now = 1000;
    fsm.update(now);
    fsm.requestDestination(11);
    fsm.submitPosition(makeMeasurement(0.8f, 0.9f, 0.95f, now), now);
    fsm.update(now += 50);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(NavState::NAVIGATING), static_cast<int>(fsm.state()));

    // Push the trolley 1.0 m off the corridor.  (2.5, 1.0) is within the replan
    // snap radius of node 1 (1.41 m < replan_max_graph_snap_distance_m), so the
    // confirmed deviation results in a successful replan rather than a failure.
    //
    // Timing: the deviation starts at i=0, the 200 ms confirmation window
    // elapses at i=4 (4 x 50 ms), so exactly one replan happens in this window.
    for (int i = 0; i < 6; ++i) {
        now += 50;
        fsm.submitPosition(makeMeasurement(2.5f, 1.0f, 0.95f, now), now);
        fsm.update(now);
    }
    TEST_ASSERT_EQUAL_INT(static_cast<int>(NavState::NAVIGATING), static_cast<int>(fsm.state()));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(NavigationError::NONE),
                          static_cast<int>(fsm.lastError()));
    TEST_ASSERT_EQUAL_UINT32(1, fsm.status().replan_count);
    // The replanned route starts at the trolley's current position, so it is
    // back on-corridor by construction: that is exactly the point of replanning.
    TEST_ASSERT_TRUE(fsm.status().cross_track_error_m <= config.path.max_cross_track_error_m);
    // And a single noisy sample must never have triggered a replan.
    TEST_ASSERT_EQUAL_UINT32(0, fsm.status().invalid_sample_count);
}

// ---------------------------------------------------------------------------
// JSON parsing and serialisation
// ---------------------------------------------------------------------------

static void test_json_position_line_parsing() {
    const char* line =
        "{\"schema_version\":1,\"trolley_id\":\"TROLLEY_01\",\"frame_id\":\"smart_market_map\","
        "\"timestamp_ms\":1000,\"position\":{\"x_m\":1.25,\"y_m\":2.70},\"quality\":0.95,"
        "\"valid\":true}";
    const JsonValue root = jsonParse(line);
    TEST_ASSERT_TRUE(root.isObject());
    PositionMeasurement measurement;
    TEST_ASSERT_TRUE(measurementFromJson(root, measurement));
    TEST_ASSERT_EQUAL_FLOAT(1.25f, measurement.position.x_m);
    TEST_ASSERT_EQUAL_FLOAT(2.70f, measurement.position.y_m);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.95f, measurement.quality);
    TEST_ASSERT_EQUAL_UINT64(1000, measurement.timestamp_ms);
    TEST_ASSERT_TRUE(measurement.valid);
    TEST_ASSERT_TRUE(boundedEquals(measurement.trolley_id, "TROLLEY_01"));
    TEST_ASSERT_TRUE(boundedEquals(measurement.frame_id, "smart_market_map"));
    TEST_ASSERT_TRUE(root.member("position").member("x_m").isNumber());
    TEST_ASSERT_FALSE(root.member("missing").valid());
    TEST_ASSERT_FALSE(jsonParse("{not json}").valid());
    TEST_ASSERT_FALSE(jsonParse(line, 5).valid());  // truncated
}

static void test_json_rejects_incomplete_position_object() {
    PositionMeasurement measurement;
    TEST_ASSERT_FALSE(measurementFromJson(jsonParse("{\"schema_version\":1}"), measurement));
    TEST_ASSERT_FALSE(measurementFromJson(
        jsonParse("{\"schema_version\":1,\"position\":{\"x_m\":1}}"), measurement));
    // x_m present but not a number.
    TEST_ASSERT_FALSE(measurementFromJson(
        jsonParse("{\"schema_version\":1,\"position\":{\"x_m\":\"a\",\"y_m\":1}}"), measurement));
}

static void test_measurement_json_round_trip() {
    PositionMeasurement original = makeMeasurement(3.42f, 5.11f, 0.96f, 123456);
    char buffer[256];
    const int written = measurementToJson(original, buffer, sizeof(buffer));
    TEST_ASSERT_TRUE(written > 0);
    PositionMeasurement parsed;
    TEST_ASSERT_TRUE(measurementFromJson(jsonParse(buffer), parsed));
    TEST_ASSERT_EQUAL_FLOAT(3.42f, parsed.position.x_m);
    TEST_ASSERT_EQUAL_FLOAT(5.11f, parsed.position.y_m);
    TEST_ASSERT_EQUAL_UINT64(123456, parsed.timestamp_ms);
    TEST_ASSERT_TRUE(parsed.valid);
}

static void test_serial_provider_reassembles_split_lines() {
    // A tiny byte stream that hands out the JSON one character at a time,
    // simulating a transport that splits lines across reads.
    struct ChunkedStream : IByteStream {
        const char* data;
        std::size_t length;
        std::size_t position{0};
        std::size_t chunk{1};
        std::size_t read(char* buffer, std::size_t max_bytes) override {
            std::size_t written = 0;
            while (written < max_bytes && written < chunk && position < length) {
                buffer[written++] = data[position++];
            }
            return written;
        }
        std::size_t write(const char*, std::size_t) override { return 0; }
    };

    const char* payload =
        "{\"schema_version\":1,\"trolley_id\":\"TROLLEY_01\",\"frame_id\":\"smart_market_map\","
        "\"timestamp_ms\":1000,\"position\":{\"x_m\":1.2,\"y_m\":2.3},\"quality\":0.95,"
        "\"valid\":true}\n"
        "POS:{\"schema_version\":1,\"trolley_id\":\"TROLLEY_01\",\"frame_id\":\"smart_market_map\","
        "\"timestamp_ms\":1050,\"position\":{\"x_m\":1.3,\"y_m\":2.4},\"quality\":0.90,"
        "\"valid\":true}\n"
        "garbage line that is not json\n";

    ChunkedStream stream;
    stream.data = payload;
    stream.length = std::strlen(payload);
    SerialPositionProvider provider(&stream);

    PositionMeasurement measurement;
    TEST_ASSERT_TRUE(provider.poll(measurement, 1000));
    TEST_ASSERT_EQUAL_FLOAT(1.2f, measurement.position.x_m);

    TEST_ASSERT_TRUE(provider.poll(measurement, 1100));
    TEST_ASSERT_EQUAL_FLOAT(1.3f, measurement.position.x_m);

    TEST_ASSERT_FALSE(provider.poll(measurement, 1200));  // only the garbage line is left
    TEST_ASSERT_EQUAL_UINT32(2, provider.parsedLines());
    TEST_ASSERT_EQUAL_UINT32(1, provider.malformedLines());
}

static void test_mock_and_replay_providers() {
    MockPositionProvider mock;
    PositionMeasurement measurement;
    TEST_ASSERT_FALSE(mock.poll(measurement, 1000));
    mock.setPosition(2.0f, 3.0f, 0.9f);
    TEST_ASSERT_TRUE(mock.poll(measurement, 1000));
    TEST_ASSERT_EQUAL_FLOAT(2.0f, measurement.position.x_m);
    TEST_ASSERT_EQUAL_UINT64(1000, measurement.timestamp_ms);  // local clock injected
    TEST_ASSERT_FALSE(mock.poll(measurement, 1050));           // consumed once

    mock.setRepeat(true);
    TEST_ASSERT_TRUE(mock.poll(measurement, 1050));
    TEST_ASSERT_TRUE(mock.poll(measurement, 1100));

    PositionMeasurement samples[3] = {makeMeasurement(1, 1), makeMeasurement(2, 2), makeMeasurement(3, 3)};
    ReplayPositionProvider replay;
    TEST_ASSERT_TRUE(replay.load(samples, 3));
    TEST_ASSERT_TRUE(replay.poll(measurement, 0));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, measurement.position.x_m);
    TEST_ASSERT_TRUE(replay.poll(measurement, 0));
    TEST_ASSERT_TRUE(replay.poll(measurement, 0));
    TEST_ASSERT_FALSE(replay.poll(measurement, 0));
    TEST_ASSERT_TRUE(replay.finished());
}

// ---------------------------------------------------------------------------
// Config loading and telemetry
// ---------------------------------------------------------------------------

/// Load a navigation config from text and report whether it was accepted.
/// The loader's error message is copied into `error` so a test can assert that a
/// rejection comes with a readable reason (pass nullptr to ignore it).
bool navigationConfigAccepted(const char* text, char* error, std::size_t error_size) {
    NavigationConfig config;
    return loadNavigationConfig(text, config, error, error_size);
}

static void test_load_navigation_config_text() {
    const char* text = R"({
      "position": {"minimum_quality": 0.7, "timeout_ms": 500},
      "path": {"waypoint_tolerance_m": 0.25, "replan_cross_track_error_m": 0.8},
      "motion": {"max_linear_speed_mps": 0.5, "max_angular_speed_radps": 1.2},
      "robot": {"wheel_base_m": 0.30},
      "control": {"navigation_rate_hz": 25.0}
    })";
    NavigationConfig config;
    char error[160] = {};
    TEST_ASSERT_TRUE(loadNavigationConfig(text, config, error, sizeof(error)));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.7f, config.position.minimum_quality);
    TEST_ASSERT_EQUAL_UINT32(500, config.position.timeout_ms);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.25f, config.path.waypoint_tolerance_m);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.8f, config.path.replan_cross_track_error_m);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.30f, config.drive.wheel_base_m);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 25.0f, config.control.navigation_rate_hz);

    // Invalid values must be rejected with a readable message.
    char failure[160] = {};
    TEST_ASSERT_FALSE(
        navigationConfigAccepted("{\"robot\":{\"wheel_base_m\":0}}", failure, sizeof(failure)));
    TEST_ASSERT_TRUE(std::strlen(failure) > 0);
    TEST_ASSERT_FALSE(navigationConfigAccepted("{\"position\":{\"minimum_quality\":5}}", nullptr, 0));
    TEST_ASSERT_FALSE(navigationConfigAccepted(
        "{\"path\":{\"max_cross_track_error_m\":0.6,\"replan_cross_track_error_m\":0.4}}",
        nullptr, 0));
    TEST_ASSERT_FALSE(navigationConfigAccepted("not json", nullptr, 0));
}

static void test_load_graph_text() {
    const char* text = R"({
      "nodes": [
        {"id": 1, "x_m": 0.0, "y_m": 0.0, "type": "entry"},
        {"id": 2, "x_m": 3.0, "y_m": 0.0, "type": "aisle"}
      ],
      "edges": [ {"from": 1, "to": 2} ]
    })";
    Graph graph;
    char error[160] = {};
    TEST_ASSERT_TRUE(loadGraph(text, graph, error, sizeof(error)));
    TEST_ASSERT_EQUAL_INT(2, graph.nodeCount());
    TEST_ASSERT_EQUAL_FLOAT(3.0f, graph.edgeAt(0).weight_m);

    // Unknown endpoint.
    TEST_ASSERT_FALSE(loadGraph(R"({"nodes":[{"id":1,"x_m":0,"y_m":0}],"edges":[{"from":1,"to":9}]})",
                                graph, error, sizeof(error)));
    // Negative weight.
    TEST_ASSERT_FALSE(loadGraph(
        R"({"nodes":[{"id":1,"x_m":0,"y_m":0},{"id":2,"x_m":1,"y_m":0}],)"
        R"("edges":[{"from":1,"to":2,"weight_m":-2}]})",
        graph, error, sizeof(error)));
    // Empty node array.
    TEST_ASSERT_FALSE(loadGraph(R"({"nodes":[]})", graph, error, sizeof(error)));
    // Duplicate node id.
    TEST_ASSERT_FALSE(loadGraph(
        R"({"nodes":[{"id":1,"x_m":0,"y_m":0},{"id":1,"x_m":1,"y_m":1}]})", graph, error,
        sizeof(error)));
}

static void test_load_map_metadata_text() {
    const char* text =
        R"({"map_id":"SMART_MARKET_MAIN","map_version":1,"frame_id":"smart_market_map",)"
        R"("width_m":12,"height_m":8})";
    MapMetadata metadata;
    char error[160] = {};
    TEST_ASSERT_TRUE(loadMapMetadata(text, metadata, error, sizeof(error)));
    TEST_ASSERT_EQUAL_INT(1, metadata.map_version);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 12.0f, metadata.width_m);
    TEST_ASSERT_TRUE(boundedEquals(metadata.map_id, "SMART_MARKET_MAIN"));
    // Wrong frame id is a hard error.
    TEST_ASSERT_FALSE(loadMapMetadata(
        R"({"map_id":"X","map_version":1,"frame_id":"map","width_m":1,"height_m":1})", metadata,
        error, sizeof(error)));
    TEST_ASSERT_FALSE(loadMapMetadata(R"({"map_id":"X","width_m":1,"height_m":1})", metadata, error,
                                      sizeof(error)));
}

static void test_telemetry_json_contract() {
    NavigationStatus status;
    status.state = NavState::NAVIGATING;
    status.position = Position2D{2.55f, 3.20f};
    status.heading_rad = 1.12f;
    status.heading_valid = true;
    status.destination_node = 11;
    status.active_waypoint_node = 7;
    status.waypoint_index = 2;
    status.waypoint_count = 5;
    status.distance_to_waypoint_m = 0.72f;
    status.distance_to_destination_m = 4.15f;
    status.linear_velocity_mps = 0.25f;
    status.angular_velocity_radps = 0.42f;
    status.motor = MotorCommand{0.31f, 0.64f};
    status.position_quality = 0.92f;
    status.position_valid = true;

    char buffer[1024];
    const int written = statusToJson(status, "TROLLEY_01", 5000, buffer, sizeof(buffer));
    TEST_ASSERT_TRUE(written > 0);

    const JsonValue root = jsonParse(buffer);
    TEST_ASSERT_TRUE(root.isObject());
    char trolley_id[32];
    TEST_ASSERT_TRUE(root.member("trolley_id").asString(trolley_id, sizeof(trolley_id)));
    TEST_ASSERT_TRUE(boundedEquals(trolley_id, "TROLLEY_01"));
    TEST_ASSERT_EQUAL_UINT64(5000, root.member("timestamp_ms").asUint64(0));
    char frame_id[32];
    TEST_ASSERT_TRUE(root.member("frame_id").asString(frame_id, sizeof(frame_id)));
    TEST_ASSERT_TRUE(boundedEquals(frame_id, "smart_market_map"));
    TEST_ASSERT_EQUAL_UINT32(1, static_cast<uint32_t>(root.member("schema_version").asInt(0)));

    const JsonValue navigation = root.member("navigation");
    TEST_ASSERT_TRUE(navigation.isObject());
    char state[32];
    TEST_ASSERT_TRUE(navigation.member("state").asString(state, sizeof(state)));
    TEST_ASSERT_TRUE(boundedEquals(state, "NAVIGATING"));
    TEST_ASSERT_EQUAL_INT(11, navigation.member("destination_node").asInt(-1));
    TEST_ASSERT_EQUAL_INT(7, navigation.member("active_waypoint_node").asInt(-1));
    TEST_ASSERT_EQUAL_INT(2, navigation.member("waypoint_index").asInt(-1));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.72f, navigation.member("distance_to_waypoint_m").asFloat());
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 4.15f, navigation.member("distance_to_destination_m").asFloat());

    const JsonValue pose = root.member("pose");
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 2.55f, pose.member("x_m").asFloat());
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 3.20f, pose.member("y_m").asFloat());
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.12f, pose.member("heading_rad").asFloat());

    const JsonValue control = root.member("control");
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.25f, control.member("linear_velocity_mps").asFloat());
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.42f, control.member("angular_velocity_radps").asFloat());
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.31f, control.member("left_motor").asFloat());
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.64f, control.member("right_motor").asFloat());
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.92f, root.member("position_quality").asFloat());
    TEST_ASSERT_TRUE(root.member("position_valid").asBool(false));
}

static void test_status_text_output() {
    NavigationStatus status;
    status.state = NavState::NAVIGATING;
    status.position = Position2D{3.21f, 4.08f};
    status.heading_rad = 1.6144f;  // ~92.5 deg
    status.heading_valid = true;
    status.destination_node = 11;
    status.active_waypoint_node = 7;
    status.waypoint_index = 3;
    status.waypoint_count = 5;
    status.motor = MotorCommand{0.43f, 0.56f};
    char buffer[1024];
    const int written = statusToText(status, "TROLLEY_01", buffer, sizeof(buffer));
    TEST_ASSERT_TRUE(written > 0);
    TEST_ASSERT_TRUE(std::strstr(buffer, "State: NAVIGATING") != nullptr);
    TEST_ASSERT_TRUE(std::strstr(buffer, "Destination Node: 11") != nullptr);
    TEST_ASSERT_TRUE(std::strstr(buffer, "Left Motor: 0.43") != nullptr);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_waypoint_progression);
    RUN_TEST(test_destination_detection_uses_tighter_tolerance);
    RUN_TEST(test_single_node_route_is_immediately_completable);
    RUN_TEST(test_snap_to_nearest_waypoint_skips_leading_nodes);
    RUN_TEST(test_waypoint_passed_plane_detection);
    RUN_TEST(test_remaining_distance_and_cross_track);
    RUN_TEST(test_lookahead_target_point);
    RUN_TEST(test_deviation_requires_confirmation_window);
    RUN_TEST(test_deviation_noise_spike_does_not_replan);
    RUN_TEST(test_deviation_respects_replan_cooldown);
    RUN_TEST(test_heading_estimation_ignores_small_displacement);
    RUN_TEST(test_follower_rotates_in_place_for_large_heading_error);
    RUN_TEST(test_follower_drives_forward_when_aligned);
    RUN_TEST(test_follower_reports_zero_for_invalid_position);
    RUN_TEST(test_position_validation_rules);
    RUN_TEST(test_position_filter_is_disabled_by_default);
    RUN_TEST(test_fsm_boot_to_planning_sequence);
    RUN_TEST(test_fsm_invalid_destination_is_rejected);
    RUN_TEST(test_fsm_unreachable_destination_reports_error);
    RUN_TEST(test_fsm_position_timeout_stops_motors);
    RUN_TEST(test_fsm_emergency_stop_and_clear);
    RUN_TEST(test_fsm_cancel_and_stop_commands);
    RUN_TEST(test_fsm_invalid_configuration_blocks_motion);
    RUN_TEST(test_fsm_snap_distance_limit_reports_error);
    RUN_TEST(test_fsm_replans_after_confirmed_deviation);
    RUN_TEST(test_json_position_line_parsing);
    RUN_TEST(test_json_rejects_incomplete_position_object);
    RUN_TEST(test_measurement_json_round_trip);
    RUN_TEST(test_serial_provider_reassembles_split_lines);
    RUN_TEST(test_mock_and_replay_providers);
    RUN_TEST(test_load_navigation_config_text);
    RUN_TEST(test_load_graph_text);
    RUN_TEST(test_load_map_metadata_text);
    RUN_TEST(test_telemetry_json_contract);
    RUN_TEST(test_status_text_output);
    return UNITY_END();
}
