// Navigation tuning parameters.  These are plain structs so they can be filled
// from JSON (simulator / config files) or from firmware constants without the
// core depending on any parser.
#pragma once

#include "navigation/differential_drive.hpp"
#include "navigation/pid.hpp"
#include "navigation/stepper.hpp"
#include "navigation/types.hpp"

namespace nav {

struct PositionConfig {
    float minimum_quality{0.60f};
    uint32_t timeout_ms{750};
    /// Displacement below which the UWB-derived heading estimate is not updated.
    /// Must exceed the UWB noise sigma, otherwise noise alone rotates the
    /// heading estimate (0.03 m is unusable against 0.05-0.20 m noise).
    /// Baseline displacement required before a position-derived heading
    /// correction is applied.  Must be well above the UWB noise sigma: the
    /// direction of a short displacement is dominated by measurement noise.
    float heading_min_displacement_m{0.50f};
    /// Blend gain for position-derived heading corrections (0..1).
    float heading_correction_gain{0.35f};
    /// Extra baseline factor applied as quality falls to 0: the required
    /// displacement becomes heading_min_displacement_m * (1 + boost).
    float heading_quality_baseline_boost{2.0f};
    /// Reject samples older than this (0 disables the check).
    uint32_t stale_sample_ms{1000};
    /// Reject positions further than this outside the map rectangle.
    float map_margin_m{1.0f};
    bool enable_filter{false};
    float filter_alpha{0.5f};
};

struct PathConfig {
    float max_graph_snap_distance_m{1.5f};
    /// Snap radius used when *replanning*.  A replan is triggered precisely
    /// because the trolley left the corridor, so it is expected to be further
    /// from the graph than during a fresh plan; using the initial limit here
    /// turns a recoverable deviation into a fatal "start node not found".
    float replan_max_graph_snap_distance_m{2.5f};
    float waypoint_tolerance_m{0.20f};
    float destination_tolerance_m{0.15f};
    float max_cross_track_error_m{0.40f};
    float replan_cross_track_error_m{0.60f};
    uint32_t route_deviation_confirm_ms{500};
    uint32_t replan_cooldown_ms{1000};
    /// Lookahead (pure pursuit) on the active segment; disabled by default so
    /// the trolley drives to node centres unless explicitly enabled.
    bool enable_lookahead{false};
    float lookahead_distance_m{0.35f};
    /// Distance from the final destination at which the approach speed starts
    /// tapering, so the trolley does not enter the destination tolerance at
    /// cruise speed.
    float destination_slowdown_distance_m{0.60f};
    /// Distance at which the follower starts slowing down for a sharp corner.
    /// Must exceed the braking distance at cruise speed, otherwise the trolley
    /// overshoots the node and leaves the corridor on a 90 deg turn.
    float corner_slowdown_distance_m{0.80f};
};

struct MotionConfig {
    float max_linear_speed_mps{0.45f};
    float min_linear_speed_mps{0.08f};
    float max_angular_speed_radps{1.5f};
    float rotate_in_place_threshold_deg{55.0f};
    /// Heading error below which the trolley may run at full speed.
    float heading_error_slow_deg{25.0f};
    /// Proportional heading gain: omega = -kp * heading_error.
    float heading_kp{1.6f};
    /// Maximum linear acceleration used by the speed ramp.
    float max_linear_accel_mps2{0.35f};
    /// Maximum angular acceleration used by the omega ramp.
    float max_angular_accel_radps2{2.5f};
    /// Heading error below which the angular command is tapered to zero.
    ///
    /// This is a *stabilisation* aid for straight segments: it removes the
    /// residual correction that UWB noise keeps injecting, so the trolley stops
    /// weaving.  The taper uses a smoothstep so the command is C1-continuous -
    /// a hard threshold would produce a step in omega and therefore chatter.
    float heading_deadband_deg{1.5f};
    /// Maximum change of a normalised wheel command per second (0..1 scale).
    /// Models the fact that a real motor cannot reverse instantly and stops the
    /// command from oscillating at the control rate.  0 disables the limit.
    float max_motor_command_change_per_s{6.0f};
};

struct ControlConfig {
    float navigation_rate_hz{20.0f};
    float telemetry_rate_hz{5.0f};
};

/// Motor/actuator selection.  The navigation algorithms are identical either
/// way; only the conversion from a body twist to actuator commands differs.
enum class DriveKind {
    /// H-bridge / DC motor: duty-cycle commands in [-1, +1].
    PWM_H_BRIDGE = 0,
    /// Stepper + step/dir driver (A4988 class): STEP frequency and DIR level.
    STEPPER_STEP_DIR,
};

struct NavigationConfig {
    PositionConfig position{};
    PathConfig path{};
    MotionConfig motion{};
    ControlConfig control{};
    DifferentialDriveConfig drive{};

    /// Actuator type actually fitted to the trolley.
    DriveKind drive_kind{DriveKind::PWM_H_BRIDGE};

    /// Stepper parameters, used when drive_kind == STEPPER_STEP_DIR.  Per-axis so
    /// a mechanical difference between the two motors can be compensated.
    StepperConfig stepper_left{};
    StepperConfig stepper_right{};

    /// Enable the step-based odometry (position feedback from step counting).
    /// Requires `stepper.enable_odometry` and is OFF by default because the
    /// navigation core consumes positions from the UWB subsystem; it exists for
    /// bench calibration and for the position-control mode.
    bool enable_step_odometry{false};

    PIDConfig heading_pid{};

    /// Map limits used for validation (meters, smart_market_map frame).
    float map_width_m{12.0f};
    float map_height_m{8.0f};

    NavigationConfig() {
        // Heading loop defaults tuned for a 0.45 m/s differential drive with a
        // 0.32 m wheel base.  Saturation is +/-(max_angular_speed).
        heading_pid.kp = motion.heading_kp;
        heading_pid.ki = 0.0f;
        heading_pid.kd = 0.05f;
        heading_pid.output_min = -motion.max_angular_speed_radps;
        heading_pid.output_max = motion.max_angular_speed_radps;
        heading_pid.integral_min = -1.0f;
        heading_pid.integral_max = 1.0f;
        heading_pid.nominal_dt_s = 1.0f / control.navigation_rate_hz;
    }
};

/// Validation result for a configuration set.
struct ConfigValidation {
    bool valid{true};
    const char* message{"ok"};

    void fail(const char* reason) {
        valid = false;
        message = reason;
    }
};

/// Validate every parameter; returns the first problem found.
ConfigValidation validateConfig(const NavigationConfig& config);

/// Clamp a control-loop dt into a sane range (protects against scheduler
/// glitches without stalling the loop).
float clampControlDt(float dt_s, const ControlConfig& config);

}  // namespace nav
