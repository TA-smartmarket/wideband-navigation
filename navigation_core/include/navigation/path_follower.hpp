// Path follower: converts "where am I / where is the active waypoint" into a
// body twist (v, omega) and then into normalised wheel commands.
//
// Control law (deliberately lightweight, suitable for a 20 Hz ESP32-S3 loop):
//   * heading error  e = wrap(target_bearing - heading)
//   * omega          = PID(target_bearing, heading)  -> saturates at max_angular_speed
//   * v              = f(|e|) with corner slowdown, then rate limited
//   * rotate in place when |e| exceeds the configured threshold
//   * wheel commands = differential drive inverse kinematics
//
// This class contains no timing, no I/O and no FSM logic, which keeps it
// directly unit-testable and reusable by the simulator and the firmware.
#pragma once

#include "navigation/differential_drive.hpp"
#include "navigation/navigation_config.hpp"
#include "navigation/pid.hpp"
#include "navigation/types.hpp"
#include "navigation/waypoint_manager.hpp"

namespace nav {

/// Route-deviation detector with time-based confirmation and a replan cooldown.
///
/// A single noisy sample must not trigger a replan, therefore the cross-track
/// error has to stay above `replan_cross_track_error_m` for
/// `route_deviation_confirm_ms` before `update()` returns true.
class DeviationMonitor {
public:
    DeviationMonitor() = default;
    explicit DeviationMonitor(const PathConfig& config) : config_(config) {}

    void setConfig(const PathConfig& config) { config_ = config; }
    void reset();

    /// Feed one cross-track error sample.  Returns true exactly once per
    /// confirmed deviation (subject to the replan cooldown).
    bool update(float cross_track_error_m, uint64_t now_ms);

    /// Soft threshold crossed (speed reduction / warning telemetry).
    bool exceededSoftLimit() const { return soft_exceeded_; }
    /// The deviation has been confirmed and a replan was requested.
    bool confirmed() const { return confirmed_; }
    float lastError() const { return last_error_m_; }
    uint32_t triggerCount() const { return trigger_count_; }
    /// Milliseconds the current over-threshold condition has been active.
    uint32_t pendingDurationMs(uint64_t now_ms) const;

private:
    PathConfig config_{};
    bool over_threshold_{false};
    bool soft_exceeded_{false};
    bool confirmed_{false};
    uint64_t over_since_ms_{0};
    uint64_t last_trigger_ms_{0};
    bool has_triggered_{false};
    float last_error_m_{0.0f};
    uint32_t trigger_count_{0};
};

/// Output of one follower cycle; everything the FSM/telemetry needs.
struct PathFollowerOutput {
    Twist twist{};
    MotorCommand motor{};
    float target_bearing_rad{0.0f};
    float heading_error_rad{0.0f};
    float distance_to_waypoint_m{0.0f};
    float cross_track_error_m{0.0f};
    float desired_linear_mps{0.0f};
    bool rotate_in_place{false};
    bool deviation_exceeded{false};
    bool replan_requested{false};
    bool valid{false};
};

class PathFollower {
public:
    PathFollower() = default;
    explicit PathFollower(const NavigationConfig& config);

    void setConfig(const NavigationConfig& config);
    const NavigationConfig& config() const { return config_; }

    /// Drop the heading estimate and the speed ramp (used on reset/estop).
    void reset();

    /// Estimate heading from successive positions.  Displacements below
    /// `heading_min_displacement_m` keep the previous heading, which prevents
    /// heading chatter when the trolley is stationary and the UWB output noisy.
    ///
    /// A position difference cannot observe a pure rotation (the trolley spins
    /// about its own centre), so this is only the *correction* half of the
    /// estimate; `update()` contributes the prediction half by integrating the
    /// commanded yaw rate (the same dead reckoning a wheel-odometry robot uses).
    ///
    /// `position_quality` (0..1) scales how long a displacement baseline is
    /// required before a correction is trusted: noisy measurements need a longer
    /// baseline to yield a usable direction.
    void updateHeading(const Position2D& position, float position_quality = 1.0f);

    /// True when a position-derived heading correction has been applied at least
    /// once since the last reset.  Before that the heading is dead reckoned from
    /// the commanded yaw rate alone.
    bool headingValid() const { return has_heading_; }
    /// Seconds since the last position-derived correction (dead-reckoning age).
    float secondsSinceHeadingCorrection() const { return seconds_since_correction_; }

    /// Run one control cycle.  `dt_s` is the *measured* elapsed time since the
    /// previous cycle: the follower must never assume the loop runs at exactly
    /// the configured rate, otherwise dead reckoning and the PID run at the
    /// wrong speed whenever the scheduler jitters or the caller steps faster
    /// (which is exactly what the desktop simulator does).
    PathFollowerOutput update(const Position2D& position, const WaypointManager& waypoints,
                              float dt_s);

    /// Build the twist for an explicit target point (exposed for tests).
    Twist computeTwist(const Position2D& position, const Position2D& target,
                       float distance_to_waypoint_m, float dt_s);

    float heading() const { return heading_rad_; }
    float commandedLinear() const { return previous_linear_mps_; }
    float commandedAngular() const { return previous_angular_radps_; }

private:
    /// Forward speed policy: full speed when aligned, reduced with a growing
    /// heading error, zero when the trolley must rotate in place.
    float desiredLinearSpeed(float heading_error_rad, float distance_to_waypoint_m,
                             float turn_angle_at_waypoint_rad) const;
    float turnAngleAtWaypoint(const WaypointManager& waypoints) const;
    /// Ramp a command towards its target using a maximum acceleration.
    static float rateLimit(float current, float target, float max_delta);
    /// Smoothstep taper applied to the angular command inside the deadband.
    float headingDeadbandScale(float heading_error_rad) const;
    /// Approach taper applied when the active waypoint is the destination.
    float destinationApproachScale(float distance_to_destination_m,
                                   float desired_linear_mps) const;
    /// Rate-limit the final wheel commands to the configured slew rate.
    MotorCommand limitCommandSlew(const MotorCommand& target, float dt_s);

    NavigationConfig config_{};
    PIDController heading_pid_{};

    Position2D previous_position_{};
    bool has_previous_position_{false};
    bool has_heading_{false};
    float heading_rad_{0.0f};
    uint32_t heading_update_count_{0};
    /// Seconds since the last position-derived heading correction (the
    /// dead-reckoning age, reported in telemetry).
    float seconds_since_correction_{0.0f};
    /// True while the follower is commanding a rotate-in-place manoeuvre, which
    /// changes how position-derived heading corrections are validated.
    bool rotate_in_place_active_{false};

    float previous_linear_mps_{0.0f};
    float previous_angular_radps_{0.0f};

    /// Last motor commands actually issued, for the slew-rate limit.
    MotorCommand previous_command_{};
};

}  // namespace nav
