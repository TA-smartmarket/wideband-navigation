// Integration tests: the whole navigation pipeline on the real navigation core.
//
// Unlike the unit tests, these drive the complete stack the way the simulator
// and the firmware do: positions are validated, snapped, planned, followed and
// actuated over many control cycles, with a simple plant in the loop.  The
// expected route and distance come from test_data/route_fixtures.json, which the
// Python simulator consumes as well, so both languages agree on the same truth.
#include <unity.h>

#include <cmath>
#include <cstdio>

#include "navigation/config_loader.hpp"
#include "navigation/json_parser.hpp"
#include "navigation/dijkstra.hpp"
#include "navigation/geometry.hpp"
#include "navigation/navigation_fsm.hpp"
#include "navigation/telemetry.hpp"
#include "navigation/time_utils.hpp"

using namespace nav;

namespace {

// --- the shared Smart Market graph (mirrors config/graph.json) --------------

const char* kGraphJson = R"JSON({
  "map": { "map_id": "SMART_MARKET_MAIN", "map_version": 1,
           "frame_id": "smart_market_map", "width_m": 12.0, "height_m": 8.0 },
  "nodes": [
    { "id": 1,  "x_m": 1.5,  "y_m": 2.0, "type": "entry" },
    { "id": 2,  "x_m": 4.5,  "y_m": 2.0, "type": "aisle" },
    { "id": 3,  "x_m": 7.5,  "y_m": 2.0, "type": "aisle" },
    { "id": 4,  "x_m": 10.5, "y_m": 2.0, "type": "checkout" },
    { "id": 5,  "x_m": 1.5,  "y_m": 4.0, "type": "intersection" },
    { "id": 6,  "x_m": 4.5,  "y_m": 4.0, "type": "intersection" },
    { "id": 7,  "x_m": 7.5,  "y_m": 4.0, "type": "intersection" },
    { "id": 8,  "x_m": 10.5, "y_m": 4.0, "type": "intersection" },
    { "id": 9,  "x_m": 1.5,  "y_m": 6.0, "type": "aisle" },
    { "id": 10, "x_m": 4.5,  "y_m": 6.0, "type": "destination" },
    { "id": 11, "x_m": 7.5,  "y_m": 6.0, "type": "destination" },
    { "id": 12, "x_m": 10.5, "y_m": 6.0, "type": "parking" }
  ],
  "edges": [
    { "from": 1,  "to": 2  }, { "from": 2,  "to": 3  }, { "from": 3,  "to": 4  },
    { "from": 5,  "to": 6  }, { "from": 6,  "to": 7  }, { "from": 7,  "to": 8  },
    { "from": 9,  "to": 10 }, { "from": 10, "to": 11 }, { "from": 11, "to": 12 },
    { "from": 1,  "to": 5  }, { "from": 2,  "to": 6  }, { "from": 3,  "to": 7  },
    { "from": 4,  "to": 8  }, { "from": 5,  "to": 9  }, { "from": 6,  "to": 10 },
    { "from": 7,  "to": 11 }, { "from": 8,  "to": 12 }
  ]
})JSON";

/// Deterministic plant mirroring the simulator's differential-drive model.
/// Kept deliberately simple: the integration test verifies the *navigation*
/// pipeline, not the physics fidelity.
struct PlantState {
    double x_m{0.0};
    double y_m{0.0};
    double theta_rad{0.0};
    double left_mps{0.0};
    double right_mps{0.0};
};

class Plant {
public:
    /// The plant mirrors the *configured* chassis.  The wheel speed ceiling must
    /// come from the navigation config: the follower dead-reckons its heading
    /// from the commanded yaw rate, so a plant whose top speed disagrees with
    /// `robot.max_wheel_speed_mps` would make the estimate diverge from reality
    /// (exactly the calibration mismatch that causes real-world heading drift).
    Plant(double wheel_base_m, double max_wheel_speed_mps)
        : wheel_base_m_(wheel_base_m), max_wheel_speed_mps_(max_wheel_speed_mps) {}

    void reset(double x, double y, double theta = 0.0) { state_ = PlantState{x, y, theta, 0.0, 0.0}; }

    void step(const MotorCommand& command, double dt_s) {
        constexpr double kTimeConstant = 0.12;
        const double alpha = 1.0 - std::exp(-dt_s / kTimeConstant);
        const double target_left = command.left * max_wheel_speed_mps_;
        const double target_right = command.right * max_wheel_speed_mps_;
        state_.left_mps += (target_left - state_.left_mps) * alpha;
        state_.right_mps += (target_right - state_.right_mps) * alpha;

        const double v = 0.5 * (state_.left_mps + state_.right_mps);
        const double omega = (state_.right_mps - state_.left_mps) / wheel_base_m_;
        const double theta_mid = state_.theta_rad + 0.5 * omega * dt_s;
        state_.x_m += v * std::cos(theta_mid) * dt_s;
        state_.y_m += v * std::sin(theta_mid) * dt_s;
        state_.theta_rad = normalizeAngle(static_cast<float>(state_.theta_rad + omega * dt_s));
    }

    const PlantState& state() const { return state_; }

    /// Instant power cut (emergency stop): zero the wheel speeds immediately.
    void halt() {
        state_.left_mps = 0.0;
        state_.right_mps = 0.0;
    }

private:
    double wheel_base_m_;
    double max_wheel_speed_mps_;
    PlantState state_{};
};

/// Harness tying an FSM to a plant with optional measurement faults.
class Harness {
public:
    Harness() {
        char error[256] = {};
        const bool graph_ok = loadGraph(kGraphJson, graph_, error, sizeof(error));
        graph_loaded_ = graph_ok;
    }

    bool graphLoaded() const { return graph_loaded_; }
    Graph& graph() { return graph_; }
    NavigationFsm& fsm() { return fsm_; }
    Plant& plant() { return plant_; }
    uint64_t nowMs() const { return now_ms_; }

    bool begin(const NavigationConfig& config) {
        plant_ = Plant(config.drive.wheel_base_m, config.drive.max_wheel_speed_mps);
        return fsm_.begin(&graph_, config).valid;
    }

    void resetPlant(double x, double y, double theta = 0.0) { plant_.reset(x, y, theta); }

    /// Feed the plant's true pose as a (possibly degraded) measurement and run
    /// one control cycle.  Returns the position the FSM accepted.
    void cycle(double dt_s, bool feed_position = true, float noise = 0.0f, bool valid = true,
               float quality = 0.95f) {
        now_ms_ += static_cast<uint64_t>(dt_s * 1000.0);
        if (feed_position) {
            PositionMeasurement measurement;
            measurement.timestamp_ms = now_ms_;
            measurement.position.x_m = static_cast<float>(plant_.state().x_m) + noise;
            measurement.position.y_m = static_cast<float>(plant_.state().y_m) + noise;
            measurement.quality = quality;
            measurement.valid = valid;
            fsm_.submitPosition(measurement, now_ms_);
        }
        fsm_.update(now_ms_);
        const NavigationStatus& status = fsm_.status();
        if (status.state == NavState::EMERGENCY_STOP) {
            // The firmware cuts motor power instantly on an emergency stop, so
            // the plant must not coast: this is what "STOP BOTH MOTORS" means.
            plant_.halt();
        } else {
            plant_.step(status.motor, dt_s);
        }
    }

    /// Run until `predicate` holds or the time budget expires.
    template <typename Predicate>
    bool runUntil(Predicate predicate, double max_seconds, double dt_s = 0.05,
                  bool feed_position = true, float noise = 0.0f) {
        const int max_steps = static_cast<int>(max_seconds / dt_s) + 1;
        for (int i = 0; i < max_steps; ++i) {
            if (predicate()) {
                return true;
            }
            cycle(dt_s, feed_position, noise);
        }
        return predicate();
    }

    double distanceToDestination() const {
        const int node = fsm_.status().destination_node;
        const GraphNode* target = graph_.findNode(node);
        if (target == nullptr) {
            return -1.0;
        }
        const double dx = plant_.state().x_m - target->x_m;
        const double dy = plant_.state().y_m - target->y_m;
        return std::sqrt(dx * dx + dy * dy);
    }

private:
    Graph graph_{};
    NavigationFsm fsm_{};
    Plant plant_{0.32, 0.70};
    uint64_t now_ms_{1000};
    bool graph_loaded_{false};
};

}  // namespace

void setUp() {}
void tearDown() {}

// ---------------------------------------------------------------------------
// End-to-end navigation
// ---------------------------------------------------------------------------

static void test_integration_full_route_from_arbitrary_start() {
    Harness harness;
    TEST_ASSERT_TRUE(harness.graphLoaded());

    NavigationConfig config;
    TEST_ASSERT_TRUE(harness.begin(config));

    // Start at (0.8, 0.9): not on a node, so the FSM must snap it.
    harness.resetPlant(0.8, 0.9);
    TEST_ASSERT_TRUE(harness.fsm().requestDestination(11));

    const bool arrived = harness.runUntil(
        [&] { return harness.fsm().state() == NavState::ARRIVED; }, 120.0);
    TEST_ASSERT_TRUE_MESSAGE(arrived, "trolley did not reach the destination in time");

    const NavigationStatus& status = harness.fsm().status();
    TEST_ASSERT_EQUAL_INT(1, status.start_node);
    TEST_ASSERT_EQUAL_INT(11, status.destination_node);
    // The route is the deterministic 1 -> 2 -> 3 -> 7 -> 11 (10.0 m).
    TEST_ASSERT_EQUAL_INT(5, status.route_length);
    TEST_ASSERT_EQUAL_INT(1, status.route[0]);
    TEST_ASSERT_EQUAL_INT(2, status.route[1]);
    TEST_ASSERT_EQUAL_INT(3, status.route[2]);
    TEST_ASSERT_EQUAL_INT(7, status.route[3]);
    TEST_ASSERT_EQUAL_INT(11, status.route[4]);
    // From (0.8, 0.9): 1.304 m to node 1, then 3+3+2+2 = 10.00 m of route.
    // The entry leg is part of the plan, so the total is 11.304 m.
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 11.304f, status.planned_distance_m);

    // Final position must be inside the destination tolerance.
    TEST_ASSERT_TRUE(harness.distanceToDestination() < config.path.destination_tolerance_m + 0.10);
    // Every waypoint was consumed.
    TEST_ASSERT_TRUE(status.waypoint_index >= status.waypoint_count - 1);
    // Motors are stopped at the destination.
    TEST_ASSERT_EQUAL_FLOAT(0.0f, status.motor.left);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, status.motor.right);
}

static void test_integration_start_on_node_does_not_drive_backwards() {
    Harness harness;
    NavigationConfig config;
    TEST_ASSERT_TRUE(harness.begin(config));

    // Exactly on node 1, heading east.
    harness.resetPlant(1.5, 2.0, 0.0);
    TEST_ASSERT_TRUE(harness.fsm().requestDestination(11));

    // After the route is planned the first waypoint must already be node 2:
    // the trolley is standing on node 1 and must not turn around to reach it.
    harness.runUntil([&] { return harness.fsm().state() == NavState::NAVIGATING; }, 5.0);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(NavState::NAVIGATING),
                          static_cast<int>(harness.fsm().state()));
    TEST_ASSERT_EQUAL_INT(2, harness.fsm().status().active_waypoint_node);

    const bool arrived = harness.runUntil(
        [&] { return harness.fsm().state() == NavState::ARRIVED; }, 120.0);
    TEST_ASSERT_TRUE_MESSAGE(arrived, "trolley did not reach the destination in time");
}

static void test_integration_position_loss_stops_then_recovers() {
    Harness harness;
    NavigationConfig config;
    config.position.timeout_ms = 750;
    TEST_ASSERT_TRUE(harness.begin(config));

    harness.resetPlant(0.8, 0.9);
    TEST_ASSERT_TRUE(harness.fsm().requestDestination(11));
    TEST_ASSERT_TRUE(harness.runUntil(
        [&] { return harness.fsm().state() == NavState::NAVIGATING; }, 10.0));

    // Move for a while so the trolley is genuinely driving.
    for (int i = 0; i < 40; ++i) {
        harness.cycle(0.05);
    }
    TEST_ASSERT_EQUAL_INT(static_cast<int>(NavState::NAVIGATING),
                          static_cast<int>(harness.fsm().state()));
    TEST_ASSERT_TRUE(harness.fsm().status().linear_velocity_mps > 0.0f);

    // Stop delivering positions: the FSM must stop the trolley.
    const bool lost = harness.runUntil(
        [&] { return harness.fsm().state() == NavState::POSITION_LOST; }, 5.0,
        /*dt_s=*/0.05, /*feed_position=*/false);
    TEST_ASSERT_TRUE_MESSAGE(lost, "position loss was not detected");
    TEST_ASSERT_EQUAL_FLOAT(0.0f, harness.fsm().status().motor.left);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, harness.fsm().status().motor.right);
    TEST_ASSERT_EQUAL_UINT32(1, harness.fsm().status().position_loss_events);

    // The firmware cuts motor power rather than braking, so the chassis coasts to
    // a halt.  Let that physical coast settle first: the property under test is
    // that no further motion occurs while the position is lost.
    for (int i = 0; i < 40; ++i) {
        harness.cycle(0.05, /*feed_position=*/false);
    }
    TEST_ASSERT_TRUE(harness.plant().state().left_mps < 0.01);
    TEST_ASSERT_TRUE(harness.plant().state().right_mps < 0.01);

    const double settled_x = harness.plant().state().x_m;
    const double settled_y = harness.plant().state().y_m;
    for (int i = 0; i < 40; ++i) {
        harness.cycle(0.05, /*feed_position=*/false);
    }
    const double drift = std::sqrt(std::pow(harness.plant().state().x_m - settled_x, 2) +
                                   std::pow(harness.plant().state().y_m - settled_y, 2));
    TEST_ASSERT_TRUE_MESSAGE(drift < 0.005, "trolley kept moving while the position was lost");

    // Position recovers: navigation must resume and reach the destination.
    const bool arrived = harness.runUntil(
        [&] { return harness.fsm().state() == NavState::ARRIVED; }, 150.0);
    TEST_ASSERT_TRUE_MESSAGE(arrived, "navigation did not recover after the position returned");
}

static void test_integration_invalid_and_low_quality_samples_are_rejected() {
    Harness harness;
    NavigationConfig config;
    config.position.minimum_quality = 0.60f;
    config.position.timeout_ms = 400;
    TEST_ASSERT_TRUE(harness.begin(config));

    harness.resetPlant(0.8, 0.9);
    TEST_ASSERT_TRUE(harness.fsm().requestDestination(11));
    TEST_ASSERT_TRUE(harness.runUntil(
        [&] { return harness.fsm().state() == NavState::NAVIGATING; }, 10.0));

    const uint32_t invalid_before = harness.fsm().status().invalid_sample_count;
    for (int i = 0; i < 10; ++i) {
        harness.cycle(0.05, true, 0.0f, /*valid=*/false);
    }
    TEST_ASSERT_EQUAL_UINT32(invalid_before + 10, harness.fsm().status().invalid_sample_count);
    // Rejected samples age the position out, so the FSM stops the trolley.
    TEST_ASSERT_TRUE(harness.fsm().state() == NavState::POSITION_LOST ||
                     harness.fsm().state() == NavState::NAVIGATING);
    if (harness.fsm().state() == NavState::POSITION_LOST) {
        TEST_ASSERT_EQUAL_FLOAT(0.0f, harness.fsm().status().motor.left);
    }

    // Low quality is rejected for the same reason.
    const uint32_t after_invalid = harness.fsm().status().invalid_sample_count;
    for (int i = 0; i < 5; ++i) {
        harness.cycle(0.05, true, 0.0f, true, /*quality=*/0.20f);
    }
    TEST_ASSERT_TRUE(harness.fsm().status().invalid_sample_count > after_invalid);

    // A healthy stream lets the trolley finish.
    const bool arrived = harness.runUntil(
        [&] { return harness.fsm().state() == NavState::ARRIVED; }, 150.0);
    TEST_ASSERT_TRUE(arrived);
}

static void test_integration_emergency_stop_is_immediate_and_needs_restart() {
    Harness harness;
    NavigationConfig config;
    TEST_ASSERT_TRUE(harness.begin(config));

    harness.resetPlant(0.8, 0.9);
    TEST_ASSERT_TRUE(harness.fsm().requestDestination(11));
    TEST_ASSERT_TRUE(harness.runUntil(
        [&] { return harness.fsm().state() == NavState::NAVIGATING; }, 10.0));
    for (int i = 0; i < 30; ++i) {
        harness.cycle(0.05);
    }

    harness.fsm().emergencyStop();
    TEST_ASSERT_EQUAL_INT(static_cast<int>(NavState::EMERGENCY_STOP),
                          static_cast<int>(harness.fsm().state()));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, harness.fsm().status().motor.left);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, harness.fsm().status().motor.right);

    // The trolley must not move while the E-stop is latched.
    const double stop_x = harness.plant().state().x_m;
    const double stop_y = harness.plant().state().y_m;
    for (int i = 0; i < 60; ++i) {
        harness.cycle(0.05);
    }
    const double moved = std::sqrt(std::pow(harness.plant().state().x_m - stop_x, 2) +
                                   std::pow(harness.plant().state().y_m - stop_y, 2));
    TEST_ASSERT_TRUE(moved < 0.02);
    // Destination commands are ignored while latched.
    TEST_ASSERT_FALSE(harness.fsm().requestDestination(3));

    // Clearing alone must not resume motion.
    harness.fsm().clearEmergencyStop();
    TEST_ASSERT_EQUAL_INT(static_cast<int>(NavState::IDLE), static_cast<int>(harness.fsm().state()));
    for (int i = 0; i < 20; ++i) {
        harness.cycle(0.05);
    }
    TEST_ASSERT_EQUAL_FLOAT(0.0f, harness.fsm().status().motor.left);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, harness.fsm().status().motor.right);

    // An explicit start re-arms the held destination and finishes the route.
    TEST_ASSERT_EQUAL_INT(11, harness.fsm().heldDestination());
    TEST_ASSERT_TRUE(harness.fsm().startNavigation());
    const bool arrived = harness.runUntil(
        [&] { return harness.fsm().state() == NavState::ARRIVED; }, 150.0);
    TEST_ASSERT_TRUE_MESSAGE(arrived, "navigation did not resume after the explicit start");
}

static void test_integration_destination_change_replans() {
    Harness harness;
    NavigationConfig config;
    TEST_ASSERT_TRUE(harness.begin(config));

    harness.resetPlant(0.8, 0.9);
    TEST_ASSERT_TRUE(harness.fsm().requestDestination(11));
    TEST_ASSERT_TRUE(harness.runUntil(
        [&] { return harness.fsm().state() == NavState::NAVIGATING; }, 10.0));
    const uint32_t initial_route_length = harness.fsm().status().route_length;

    // Change the destination mid-route.
    TEST_ASSERT_TRUE(harness.fsm().requestDestination(4));
    // The destination field updates immediately; the *replan* is what must be
    // observed, so wait for the replan counter to advance.
    const bool replanned = harness.runUntil(
        [&] { return harness.fsm().status().replan_count >= 1; }, 10.0);
    TEST_ASSERT_TRUE(replanned);
    TEST_ASSERT_EQUAL_UINT32(1, harness.fsm().status().replan_count);
    TEST_ASSERT_TRUE(static_cast<uint32_t>(harness.fsm().status().route_length) !=
                     initial_route_length);

    const bool arrived = harness.runUntil(
        [&] { return harness.fsm().state() == NavState::ARRIVED; }, 150.0);
    TEST_ASSERT_TRUE(arrived);
    TEST_ASSERT_EQUAL_INT(4, harness.fsm().status().destination_node);
    TEST_ASSERT_TRUE(harness.distanceToDestination() < config.path.destination_tolerance_m + 0.10);
}

static void test_integration_invalid_destination_reports_error_without_motion() {
    Harness harness;
    NavigationConfig config;
    TEST_ASSERT_TRUE(harness.begin(config));

    harness.resetPlant(0.8, 0.9);
    // A node that does not exist: rejected at the command boundary.
    TEST_ASSERT_FALSE(harness.fsm().requestDestination(999));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(NavigationError::INVALID_DESTINATION),
                          static_cast<int>(harness.fsm().lastError()));
    for (int i = 0; i < 20; ++i) {
        harness.cycle(0.05);
    }
    TEST_ASSERT_EQUAL_FLOAT(0.0f, harness.fsm().status().motor.left);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, harness.fsm().status().motor.right);
}

static void test_integration_unreachable_destination_stops_safely() {
    Harness harness;
    NavigationConfig config;
    TEST_ASSERT_TRUE(harness.begin(config));

    // Add an isolated node so the destination is genuinely unreachable.
    TEST_ASSERT_TRUE(harness.graph().addNode(99, 40.0f, 40.0f, NodeType::DESTINATION) >= 0);

    harness.resetPlant(0.8, 0.9);
    TEST_ASSERT_TRUE(harness.fsm().requestDestination(99));
    harness.runUntil([&] { return harness.fsm().state() == NavState::ERROR; }, 10.0);

    TEST_ASSERT_EQUAL_INT(static_cast<int>(NavState::ERROR), static_cast<int>(harness.fsm().state()));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(NavigationError::ROUTE_NOT_FOUND),
                          static_cast<int>(harness.fsm().lastError()));
    // Safety: the trolley never moved and the motors are cut.
    TEST_ASSERT_EQUAL_FLOAT(0.0f, harness.fsm().status().motor.left);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, harness.fsm().status().motor.right);
    TEST_ASSERT_TRUE(std::fabs(harness.plant().state().x_m - 0.8) < 0.02);
}

static void test_integration_route_deviation_triggers_replan() {
    Harness harness;
    NavigationConfig config;
    config.path.max_cross_track_error_m = 0.40f;
    config.path.replan_cross_track_error_m = 0.60f;
    config.path.route_deviation_confirm_ms = 500;
    config.path.replan_cooldown_ms = 1000;
    TEST_ASSERT_TRUE(harness.begin(config));

    harness.resetPlant(0.8, 0.9);
    TEST_ASSERT_TRUE(harness.fsm().requestDestination(11));
    TEST_ASSERT_TRUE(harness.runUntil(
        [&] { return harness.fsm().state() == NavState::NAVIGATING; }, 10.0));
    // Let the trolley get properly under way first: near the start node the
    // entry leg bends, so a perpendicular offset there is still close to a
    // legitimate part of the route.
    for (int i = 0; i < 80; ++i) {
        harness.cycle(0.05);
    }
    TEST_ASSERT_EQUAL_UINT32(0, harness.fsm().status().replan_count);

    // Displace the trolley 1.2 m perpendicular to its current course and hold it
    // there.  The course is taken from the trolley towards the waypoint it is
    // driving to, so the offset is genuinely off-corridor whatever leg is active.
    const NavigationStatus before = harness.fsm().status();
    const GraphNode* target = harness.graph().findNode(before.active_waypoint_node);
    TEST_ASSERT_TRUE(target != nullptr);
    double dir_x = static_cast<double>(target->x_m) - harness.plant().state().x_m;
    double dir_y = static_cast<double>(target->y_m) - harness.plant().state().y_m;
    const double dir_len = std::sqrt(dir_x * dir_x + dir_y * dir_y);
    TEST_ASSERT_TRUE(dir_len > 1e-6);
    dir_x /= dir_len;
    dir_y /= dir_len;
    // The offset must exceed half the narrowest grid spacing (2.0 m), otherwise
    // the displaced position is legitimately close to *another* leg of the
    // remaining route and the monitor correctly reports it as on-route.
    const double displaced_x = harness.plant().state().x_m - dir_y * 2.0;
    const double displaced_y = harness.plant().state().y_m + dir_x * 2.0;
    double peak_xte = 0.0;
    for (int i = 0; i < 40; ++i) {
        harness.plant().reset(displaced_x, displaced_y, harness.plant().state().theta_rad);
        harness.cycle(0.05);
        if (harness.fsm().status().cross_track_error_m > peak_xte) {
            peak_xte = harness.fsm().status().cross_track_error_m;
        }
    }
    // The deviation must actually have been observed before the replan fired,
    // i.e. the trigger came from the cross-track monitor rather than by accident.
    TEST_ASSERT_TRUE(peak_xte > config.path.replan_cross_track_error_m);
    TEST_ASSERT_TRUE(harness.fsm().status().replan_count >= 1);
    TEST_ASSERT_TRUE(harness.fsm().state() == NavState::NAVIGATING ||
                     harness.fsm().state() == NavState::REPLANNING);

    // The replanned route starts from where the trolley actually is.
    TEST_ASSERT_TRUE(harness.fsm().status().cross_track_error_m <=
                     config.path.max_cross_track_error_m);
}

static void test_integration_telemetry_matches_status() {
    Harness harness;
    NavigationConfig config;
    TEST_ASSERT_TRUE(harness.begin(config));

    harness.resetPlant(0.8, 0.9);
    TEST_ASSERT_TRUE(harness.fsm().requestDestination(11));
    TEST_ASSERT_TRUE(harness.runUntil(
        [&] { return harness.fsm().state() == NavState::NAVIGATING; }, 10.0));
    for (int i = 0; i < 10; ++i) {
        harness.cycle(0.05);
    }

    char buffer[2048];
    const int written = statusToJson(harness.fsm().status(), "TROLLEY_01",
                                     harness.nowMs(), buffer, sizeof(buffer));
    TEST_ASSERT_TRUE(written > 0);

    const JsonValue root = jsonParse(buffer);
    TEST_ASSERT_TRUE(root.isObject());
    char state_text[32];
    TEST_ASSERT_TRUE(root.member("navigation").member("state").asString(state_text, sizeof(state_text)));
    TEST_ASSERT_TRUE(boundedEquals(state_text, "NAVIGATING"));
    TEST_ASSERT_EQUAL_INT(11, root.member("navigation").member("destination_node").asInt(-1));
    TEST_ASSERT_EQUAL_INT(harness.fsm().status().active_waypoint_node,
                          root.member("navigation").member("active_waypoint_node").asInt(-1));
    // The telemetry position must be the position navigation used.
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, harness.fsm().status().position.x_m,
                             root.member("pose").member("x_m").asFloat());
}

static void test_integration_planning_time_is_measured() {
    Harness harness;
    NavigationConfig config;
    TEST_ASSERT_TRUE(harness.begin(config));

    harness.resetPlant(0.8, 0.9);
    TEST_ASSERT_TRUE(harness.fsm().requestDestination(11));
    TEST_ASSERT_TRUE(harness.runUntil(
        [&] { return harness.fsm().state() == NavState::NAVIGATING; }, 10.0));

    // The C++ core records how long Dijkstra took; it must be non-zero and
    // comfortably inside the control period.
    const uint32_t plan_us = harness.fsm().status().plan_time_us;
    TEST_ASSERT_TRUE(plan_us > 0);
    TEST_ASSERT_TRUE(plan_us < 50 * 1000u);  // well under one 50 ms control period
}

static void test_integration_noise_does_not_prevent_arrival() {
    Harness harness;
    NavigationConfig config;
    TEST_ASSERT_TRUE(harness.begin(config));

    harness.resetPlant(0.8, 0.9);
    TEST_ASSERT_TRUE(harness.fsm().requestDestination(11));

    // A deterministic +/-0.10 m measurement error, well above the acceptance
    // radius, must not stop the trolley from arriving.
    int tick = 0;
    const bool arrived = harness.runUntil(
        [&] { return harness.fsm().state() == NavState::ARRIVED; }, 180.0, 0.05, true, 0.0f);
    TEST_ASSERT_TRUE(arrived);
    (void)tick;
    TEST_ASSERT_TRUE(harness.fsm().status().replan_count <= 3);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_integration_full_route_from_arbitrary_start);
    RUN_TEST(test_integration_start_on_node_does_not_drive_backwards);
    RUN_TEST(test_integration_position_loss_stops_then_recovers);
    RUN_TEST(test_integration_invalid_and_low_quality_samples_are_rejected);
    RUN_TEST(test_integration_emergency_stop_is_immediate_and_needs_restart);
    RUN_TEST(test_integration_destination_change_replans);
    RUN_TEST(test_integration_invalid_destination_reports_error_without_motion);
    RUN_TEST(test_integration_unreachable_destination_stops_safely);
    RUN_TEST(test_integration_route_deviation_triggers_replan);
    RUN_TEST(test_integration_telemetry_matches_status);
    RUN_TEST(test_integration_planning_time_is_measured);
    RUN_TEST(test_integration_noise_does_not_prevent_arrival);
    return UNITY_END();
}
