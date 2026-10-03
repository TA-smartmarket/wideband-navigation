#include "navigation/path_follower.hpp"

#include <cmath>

#include "navigation/geometry.hpp"

namespace nav {

namespace {
/// Heading error (radians) at which the trolley stops translating and turns on
/// the spot; derived from `rotate_in_place_threshold_deg` at run time.
constexpr float kRadToDeg = 57.29577951308232f;
constexpr float kDegToRad = 0.017453292519943295f;
}  // namespace

// ---------------------------------------------------------------------------
// DeviationMonitor
// ---------------------------------------------------------------------------

void DeviationMonitor::reset() {
    over_threshold_ = false;
    soft_exceeded_ = false;
    confirmed_ = false;
    over_since_ms_ = 0;
    last_trigger_ms_ = 0;
    has_triggered_ = false;
    last_error_m_ = 0.0f;
    trigger_count_ = 0;
}

uint32_t DeviationMonitor::pendingDurationMs(uint64_t now_ms) const {
    if (!over_threshold_ || now_ms < over_since_ms_) {
        return 0;
    }
    return static_cast<uint32_t>(now_ms - over_since_ms_);
}

bool DeviationMonitor::update(float cross_track_error_m, uint64_t now_ms) {
    last_error_m_ = isFinite(cross_track_error_m) ? cross_track_error_m : 0.0f;

    soft_exceeded_ = last_error_m_ > config_.max_cross_track_error_m;

    if (last_error_m_ <= config_.replan_cross_track_error_m) {
        // Back inside the corridor: forget the pending deviation.
        over_threshold_ = false;
        confirmed_ = false;
        over_since_ms_ = 0;
        return false;
    }

    if (!over_threshold_) {
        over_threshold_ = true;
        over_since_ms_ = now_ms;
        confirmed_ = false;
        return false;
    }

    // Still over threshold: require the confirmation window before acting.
    if (pendingDurationMs(now_ms) < config_.route_deviation_confirm_ms) {
        confirmed_ = false;
        return false;
    }

    // Cooldown prevents a replan storm while the trolley is recovering.
    if (has_triggered_ && (now_ms - last_trigger_ms_) < config_.replan_cooldown_ms) {
        return false;
    }

    has_triggered_ = true;
    last_trigger_ms_ = now_ms;
    confirmed_ = true;
    ++trigger_count_;
    // Re-arm the confirmation window: a second replan needs another full window.
    over_since_ms_ = now_ms;
    return true;
}

// ---------------------------------------------------------------------------
// PathFollower
// ---------------------------------------------------------------------------

PathFollower::PathFollower(const NavigationConfig& config) { setConfig(config); }

void PathFollower::setConfig(const NavigationConfig& config) {
    config_ = config;
    heading_pid_.setConfig(config_.heading_pid);
    heading_pid_.reset();
}

void PathFollower::reset() {
    has_previous_position_ = false;
    has_heading_ = false;
    heading_rad_ = 0.0f;
    heading_update_count_ = 0;
    seconds_since_correction_ = 0.0f;
    rotate_in_place_active_ = false;
    previous_linear_mps_ = 0.0f;
    previous_angular_radps_ = 0.0f;
    previous_command_ = MotorCommand{};
    heading_pid_.reset();
}

void PathFollower::updateHeading(const Position2D& position, float position_quality) {
    if (!isFinite(position.x_m) || !isFinite(position.y_m)) {
        return;  // never let a corrupt sample poison the heading
    }
    if (!has_previous_position_) {
        previous_position_ = position;
        has_previous_position_ = true;
        return;
    }
    const float dx = position.x_m - previous_position_.x_m;
    const float dy = position.y_m - previous_position_.y_m;
    const float displacement = std::sqrt(dx * dx + dy * dy);

    // While the trolley rotates on the spot, the true heading is only observable
    // through the commanded yaw rate: the displacement it produces is
    // *tangential* to the arc, so `atan2(dy, dx)` would be ~90 deg away from the
    // real heading.  Applying such a correction makes the follower rotate
    // forever (a limit cycle around the waypoint).  In this mode the estimate is
    // therefore left entirely to dead reckoning and the reference point is kept.
    if (rotate_in_place_active_) {
        return;
    }

    // The baseline grows as the reported quality drops: with sigma comparable
    // to the baseline the measured direction is dominated by measurement noise,
    // so a noisier sensor must travel further before its heading is trusted.
    const float quality = isFinite(position_quality) ? clampf(position_quality, 0.0f, 1.0f) : 1.0f;
    const float baseline =
        config_.position.heading_min_displacement_m *
        (1.0f + config_.position.heading_quality_baseline_boost * (1.0f - quality));

    if (displacement >= baseline) {
        // Blend towards the measured direction instead of overwriting it.  A
        // single UWB sample can place the displacement vector tens of degrees
        // off the true course (the noise sigma is comparable to the baseline),
        // so an abrupt assignment makes the follower oscillate.  The circular
        // exponential blend removes that noise while still tracking slow drift.
        const float measured = std::atan2(dy, dx);
        const float alpha = clampf(config_.position.heading_correction_gain, 0.0f, 1.0f);
        if (has_heading_) {
            heading_rad_ = normalizeAngle(
                heading_rad_ + alpha * shortestAngularDistance(heading_rad_, measured));
        } else {
            heading_rad_ = measured;
            has_heading_ = true;
        }
        ++heading_update_count_;
        seconds_since_correction_ = 0.0f;
        previous_position_ = position;
    }
}

float PathFollower::headingDeadbandScale(float heading_error_rad) const {
    const float deadband = config_.motion.heading_deadband_deg * kDegToRad;
    if (deadband <= 0.0f) {
        return 1.0f;
    }
    const float magnitude = std::fabs(heading_error_rad);
    if (magnitude >= deadband) {
        return 1.0f;
    }
    // Smoothstep inside the deadband: the scale and its first derivative both go
    // to zero at the edge, so the angular command is continuous and the trolley
    // settles instead of chattering across a hard threshold.
    const float x = magnitude / deadband;
    return x * x * (3.0f - 2.0f * x);
}

float PathFollower::destinationApproachScale(float distance_to_destination_m,
                                             float desired_linear_mps) const {
    const float slowdown = config_.path.destination_slowdown_distance_m;
    if (slowdown <= 1.0e-3f || distance_to_destination_m >= slowdown) {
        return 1.0f;
    }
    const float t = clampf(distance_to_destination_m / slowdown, 0.0f, 1.0f);
    // Linear taper down to the minimum controllable speed, never below it: the
    // trolley must still be able to creep the last few centimetres instead of
    // stalling outside the destination tolerance.
    const float minimum = config_.motion.min_linear_speed_mps;
    if (desired_linear_mps <= minimum) {
        return 1.0f;
    }
    const float target = minimum + (desired_linear_mps - minimum) * t;
    return clampf(target / desired_linear_mps, 0.0f, 1.0f);
}

MotorCommand PathFollower::limitCommandSlew(const MotorCommand& target, float dt_s) {
    const float limit = config_.motion.max_motor_command_change_per_s;
    if (limit <= 0.0f || dt_s <= 0.0f) {
        previous_command_ = target;
        return target;
    }
    const float max_delta = limit * dt_s;
    MotorCommand limited;
    limited.left = rateLimit(previous_command_.left, target.left, max_delta);
    limited.right = rateLimit(previous_command_.right, target.right, max_delta);
    previous_command_ = limited;
    return limited;
}

float PathFollower::rateLimit(float current, float target, float max_delta) {
    if (!isFinite(target)) {
        return 0.0f;
    }
    if (!isFinite(current)) {
        return target;
    }
    const float delta = target - current;
    if (delta > max_delta) {
        return current + max_delta;
    }
    if (delta < -max_delta) {
        return current - max_delta;
    }
    return target;
}

float PathFollower::turnAngleAtWaypoint(const WaypointManager& waypoints) const {
    const Waypoint* current = waypoints.current();
    const Waypoint* next = waypoints.next();
    if (current == nullptr || next == nullptr) {
        return 0.0f;
    }
    const int index = waypoints.currentIndex();
    const Waypoint* previous = index > 0 ? &waypoints.waypoints()[index - 1] : nullptr;
    if (previous == nullptr) {
        return 0.0f;  // the first waypoint has no incoming leg to compare against
    }
    const float in_x = current->x_m - previous->x_m;
    const float in_y = current->y_m - previous->y_m;
    const float out_x = next->x_m - current->x_m;
    const float out_y = next->y_m - current->y_m;
    const float in_len = std::sqrt(in_x * in_x + in_y * in_y);
    const float out_len = std::sqrt(out_x * out_x + out_y * out_y);
    if (in_len <= 1.0e-6f || out_len <= 1.0e-6f) {
        return 0.0f;
    }
    return std::fabs(shortestAngularDistance(std::atan2(in_y, in_x), std::atan2(out_y, out_x)));
}

float PathFollower::desiredLinearSpeed(float heading_error_rad, float distance_to_waypoint_m,
                                       float turn_angle_at_waypoint_rad) const {
    const MotionConfig& motion = config_.motion;
    const float error_deg = std::fabs(heading_error_rad) * kRadToDeg;
    if (error_deg >= motion.rotate_in_place_threshold_deg) {
        return 0.0f;  // turn on the spot instead of tracing a wide arc
    }

    float speed = motion.max_linear_speed_mps;
    if (error_deg > motion.heading_error_slow_deg) {
        // Linear derating between the "full speed" angle and the rotate-in-place
        // angle: at the threshold the trolley is already at minimum speed.
        const float span = motion.rotate_in_place_threshold_deg - motion.heading_error_slow_deg;
        const float t = span > 1.0e-3f ? (error_deg - motion.heading_error_slow_deg) / span : 1.0f;
        const float scale = clampf(1.0f - t, 0.0f, 1.0f);
        speed = motion.min_linear_speed_mps +
                (motion.max_linear_speed_mps - motion.min_linear_speed_mps) * scale;
    }

    // Corner handling: slow down before a sharp turn so the follower can settle
    // inside the acceptance radius instead of overshooting the node.
    const float slowdown_distance = config_.path.corner_slowdown_distance_m;
    if (turn_angle_at_waypoint_rad > 0.6f && slowdown_distance > 1.0e-3f &&
        distance_to_waypoint_m < slowdown_distance) {
        // Quadratic derating: the trolley must already be slow well before the
        // node, because the braking distance at cruise speed is comparable to
        // the corner radius and a linear ramp leaves it too fast at the node.
        const float t = clampf(distance_to_waypoint_m / slowdown_distance, 0.0f, 1.0f);
        const float corner_speed = motion.min_linear_speed_mps +
                                   (speed - motion.min_linear_speed_mps) * t * t;
        speed = corner_speed < speed ? corner_speed : speed;
    }
    return clampf(speed, 0.0f, motion.max_linear_speed_mps);
}

Twist PathFollower::computeTwist(const Position2D& position, const Position2D& target,
                                 float distance_to_waypoint_m, float dt_s) {
    Twist twist;
    const MotionConfig& motion = config_.motion;
    const float dt = clampControlDt(dt_s, config_.control);

    const float target_bearing = calculateBearing(position, target);
    // The heading may not be valid yet (no displacement since boot): assume the
    // trolley faces the target so the first cycle does not spin wildly.  The
    // caller is responsible for not moving before a heading exists.
    const float heading = has_heading_ ? heading_rad_ : target_bearing;
    const float heading_error = normalizeAngle(target_bearing - heading);

    // PID saturates at +/- max_angular_speed (configured in NavigationConfig).
    const float omega_pid = heading_pid_.update(target_bearing, heading, dt);
    const float omega = clampf(omega_pid, -motion.max_angular_speed_radps,
                               motion.max_angular_speed_radps);

    const float turn_angle = 0.0f;  // no route context in the pure API: no corner slowdown
    float desired_linear = desiredLinearSpeed(heading_error, distance_to_waypoint_m, turn_angle);

    // Rate limiting models the physical inability of the chassis to change speed
    // instantly and keeps the simulator's PID test meaningful.
    const float max_dv = motion.max_linear_accel_mps2 * dt;
    const float max_domega = motion.max_angular_accel_radps2 * dt;
    const float limited_linear = rateLimit(previous_linear_mps_, desired_linear, max_dv);
    const float limited_omega = rateLimit(previous_angular_radps_, omega, max_domega);

    previous_linear_mps_ = limited_linear;
    previous_angular_radps_ = limited_omega;

    twist.linear_mps = limited_linear;
    twist.angular_radps = limited_omega;
    return twist;
}

PathFollowerOutput PathFollower::update(const Position2D& position,
                                        const WaypointManager& waypoints, float dt_s) {
    PathFollowerOutput output;
    if (!isFinite(position.x_m) || !isFinite(position.y_m)) {
        return output;  // invalid input -> zero twist, caller keeps motors stopped
    }
    // Clamp a jittery or missing dt: a zero/absurd value would make the
    // derivative term and the yaw dead reckoning meaningless.
    const float dt = clampControlDt(dt_s, config_.control);
    const Waypoint* waypoint = waypoints.current();
    if (waypoint == nullptr) {
        return output;
    }

    const Position2D target = waypoints.targetPoint(position);
    output.distance_to_waypoint_m = waypoints.distanceToCurrent(position);
    output.cross_track_error_m = waypoints.crossTrackError(position);
    output.target_bearing_rad = calculateBearing(position, target);

    // Always steer with the best available estimate.  Before the first
    // position-derived correction this is the dead-reckoned value (seeded at 0
    // and integrated from the commanded yaw rate), which is strictly better than
    // assuming the trolley already faces the target: that assumption yields a
    // zero error and makes the trolley drive straight at full speed in whatever
    // direction it happens to point, producing a wide initial swoop.
    const float heading = heading_rad_;
    output.heading_error_rad = normalizeAngle(output.target_bearing_rad - heading);

    // Straight-segment stabilisation: a taper factor applied to the *command*,
    // not to the reported error.  Telemetry and metrics must keep seeing the true
    // tracking error, otherwise the deadband would flatter the numbers.
    const float deadband_scale = headingDeadbandScale(output.heading_error_rad);
    const float error_deg = std::fabs(output.heading_error_rad) * kRadToDeg;
    output.rotate_in_place = error_deg >= config_.motion.rotate_in_place_threshold_deg;
    // Remember the mode for the next heading correction (updateHeading runs
    // before this function in the control cycle).
    rotate_in_place_active_ = output.rotate_in_place;

    const float turn_angle = turnAngleAtWaypoint(waypoints);
    output.desired_linear_mps =
        desiredLinearSpeed(output.heading_error_rad, output.distance_to_waypoint_m, turn_angle);

    // Final approach: taper the speed so the trolley does not arrive at cruise
    // speed.  Only the *last* waypoint (the destination) is tapered; intermediate
    // nodes keep their corner-based slowdown so the trolley does not crawl
    // through the whole route.
    if (waypoints.currentIndex() >= waypoints.waypointCount() - 1) {
        output.desired_linear_mps *=
            destinationApproachScale(waypoints.distanceToDestination(position),
                                     output.desired_linear_mps);
    }

    const float omega_pid = heading_pid_.update(output.target_bearing_rad, heading, dt);
    // The taper is applied to the angular command: inside the deadband the
    // residual correction fades out smoothly, so the trolley stops weaving on
    // straights while large errors are still corrected at full authority.
    const float omega = clampf(omega_pid * deadband_scale, -config_.motion.max_angular_speed_radps,
                               config_.motion.max_angular_speed_radps);

    const float limited_linear =
        rateLimit(previous_linear_mps_, output.desired_linear_mps,
                  config_.motion.max_linear_accel_mps2 * dt);
    const float limited_omega =
        rateLimit(previous_angular_radps_, omega, config_.motion.max_angular_accel_radps2 * dt);

    // Predictor half of the heading estimate: integrate the *actually
    // commanded* yaw rate (dead reckoning).  A position difference cannot
    // observe rotation in place, so this is the only source of heading
    // information while the trolley spins on the spot; freezing it (as an
    // earlier revision did) makes the controller believe the error never
    // shrinks and the trolley rotates forever.
    //
    // Drift is self-correcting: the next position-derived correction in
    // updateHeading() overwrites the estimate as soon as the trolley travels
    // `heading_min_displacement_m`.  During a pure spin the drift per second is
    // bounded by the wheel gain mismatch, which is small.
    // Integrated unconditionally: `has_heading_` only records whether a
    // position-derived correction has ever been applied, whereas the yaw
    // estimate itself is valid from the first commanded rotation.  Gating this
    // on `has_heading_` would leave the estimate frozen at its initial value
    // while the trolley spins, so the heading error would never shrink.
    heading_rad_ = normalizeAngle(heading_rad_ + limited_omega * dt);
    seconds_since_correction_ += dt;

    previous_linear_mps_ = limited_linear;
    previous_angular_radps_ = limited_omega;

    output.twist.linear_mps = limited_linear;
    output.twist.angular_radps = limited_omega;
    // Slew-rate limit the wheel commands: a real motor cannot reverse instantly,
    // and without this the command can oscillate at the control rate while the
    // body twist looks smooth.
    output.motor = limitCommandSlew(twistToMotorCommand(output.twist, config_.drive), dt);
    output.valid = true;
    return output;
}

}  // namespace nav
