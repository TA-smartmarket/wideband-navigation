#include "navigation/navigation_config.hpp"

#include "navigation/geometry.hpp"

namespace nav {

ConfigValidation validateConfig(const NavigationConfig& config) {
    ConfigValidation result;

    // --- position -----------------------------------------------------------
    if (!isFinite(config.position.minimum_quality) || config.position.minimum_quality < 0.0f ||
        config.position.minimum_quality > 1.0f) {
        result.fail("position.minimum_quality must be within [0, 1]");
        return result;
    }
    if (config.position.timeout_ms == 0) {
        result.fail("position.timeout_ms must be > 0");
        return result;
    }
    if (!isFinite(config.position.heading_min_displacement_m) ||
        config.position.heading_min_displacement_m < 0.0f) {
        result.fail("position.heading_min_displacement_m must be >= 0");
        return result;
    }
    if (!isFinite(config.position.filter_alpha) || config.position.filter_alpha <= 0.0f ||
        config.position.filter_alpha > 1.0f) {
        result.fail("position.filter_alpha must be within (0, 1]");
        return result;
    }

    // --- path ---------------------------------------------------------------
    if (!isFinite(config.path.max_graph_snap_distance_m) ||
        config.path.max_graph_snap_distance_m <= 0.0f) {
        result.fail("path.max_graph_snap_distance_m must be > 0");
        return result;
    }
    if (!isFinite(config.path.waypoint_tolerance_m) || config.path.waypoint_tolerance_m <= 0.0f) {
        result.fail("path.waypoint_tolerance_m must be > 0");
        return result;
    }
    if (!isFinite(config.path.destination_tolerance_m) ||
        config.path.destination_tolerance_m <= 0.0f) {
        result.fail("path.destination_tolerance_m must be > 0");
        return result;
    }
    if (!isFinite(config.path.max_cross_track_error_m) ||
        config.path.max_cross_track_error_m <= 0.0f) {
        result.fail("path.max_cross_track_error_m must be > 0");
        return result;
    }
    if (!isFinite(config.path.replan_cross_track_error_m) ||
        config.path.replan_cross_track_error_m < config.path.max_cross_track_error_m) {
        result.fail("path.replan_cross_track_error_m must be >= max_cross_track_error_m");
        return result;
    }
    if (!isFinite(config.path.lookahead_distance_m) || config.path.lookahead_distance_m < 0.0f) {
        result.fail("path.lookahead_distance_m must be >= 0");
        return result;
    }

    // --- motion -------------------------------------------------------------
    if (!isFinite(config.motion.max_linear_speed_mps) || config.motion.max_linear_speed_mps <= 0.0f) {
        result.fail("motion.max_linear_speed_mps must be > 0");
        return result;
    }
    if (!isFinite(config.motion.min_linear_speed_mps) || config.motion.min_linear_speed_mps < 0.0f) {
        result.fail("motion.min_linear_speed_mps must be >= 0");
        return result;
    }
    if (config.motion.min_linear_speed_mps > config.motion.max_linear_speed_mps) {
        result.fail("motion.min_linear_speed_mps must be <= max_linear_speed_mps");
        return result;
    }
    if (!isFinite(config.motion.max_angular_speed_radps) ||
        config.motion.max_angular_speed_radps <= 0.0f) {
        result.fail("motion.max_angular_speed_radps must be > 0");
        return result;
    }
    if (!isFinite(config.motion.rotate_in_place_threshold_deg) ||
        config.motion.rotate_in_place_threshold_deg < 0.0f ||
        config.motion.rotate_in_place_threshold_deg > 180.0f) {
        result.fail("motion.rotate_in_place_threshold_deg must be within [0, 180]");
        return result;
    }
    if (!isFinite(config.motion.heading_error_slow_deg) ||
        config.motion.heading_error_slow_deg <= 0.0f) {
        result.fail("motion.heading_error_slow_deg must be > 0");
        return result;
    }
    if (!isFinite(config.motion.max_linear_accel_mps2) || config.motion.max_linear_accel_mps2 <= 0.0f) {
        result.fail("motion.max_linear_accel_mps2 must be > 0");
        return result;
    }
    if (!isFinite(config.motion.max_angular_accel_radps2) ||
        config.motion.max_angular_accel_radps2 <= 0.0f) {
        result.fail("motion.max_angular_accel_radps2 must be > 0");
        return result;
    }

    if (!isFinite(config.motion.heading_deadband_deg) ||
        config.motion.heading_deadband_deg < 0.0f ||
        config.motion.heading_deadband_deg > 30.0f) {
        result.fail("motion.heading_deadband_deg must be within [0, 30]");
        return result;
    }
    if (!isFinite(config.motion.max_motor_command_change_per_s) ||
        config.motion.max_motor_command_change_per_s < 0.0f) {
        result.fail("motion.max_motor_command_change_per_s must be >= 0");
        return result;
    }
    if (!isFinite(config.path.destination_slowdown_distance_m) ||
        config.path.destination_slowdown_distance_m < 0.0f) {
        result.fail("path.destination_slowdown_distance_m must be >= 0");
        return result;
    }

    // --- control ------------------------------------------------------------
    if (!isFinite(config.control.navigation_rate_hz) || config.control.navigation_rate_hz <= 0.0f ||
        config.control.navigation_rate_hz > 200.0f) {
        result.fail("control.navigation_rate_hz must be within (0, 200]");
        return result;
    }
    if (!isFinite(config.control.telemetry_rate_hz) || config.control.telemetry_rate_hz <= 0.0f ||
        config.control.telemetry_rate_hz > 100.0f) {
        result.fail("control.telemetry_rate_hz must be within (0, 100]");
        return result;
    }

    // --- chassis ------------------------------------------------------------
    const char* drive_problem = validateDifferentialDriveConfig(config.drive);
    if (drive_problem != nullptr) {
        result.fail(drive_problem);
        return result;
    }

    // --- stepper (only when that actuator is selected) ----------------------
    if (config.drive_kind == DriveKind::STEPPER_STEP_DIR) {
        const StepperValidation left = validateStepperConfig(config.stepper_left);
        if (!left.valid) {
            result.fail(left.message);
            return result;
        }
        const StepperValidation right = validateStepperConfig(config.stepper_right);
        if (!right.valid) {
            result.fail(right.message);
            return result;
        }
        // The step generator must be able to reach the fastest wheel the motion
        // profile can demand, otherwise the chassis silently under-runs its
        // commands (the same trap as an undersized max_wheel_speed_mps).
        const float required_wheel =
            config.motion.max_linear_speed_mps +
            0.5f * config.motion.max_angular_speed_radps * config.drive.wheel_base_m;
        const float required_rate =
            wheelVelocityToStepRate(required_wheel, config.stepper_left);
        if (required_rate > config.stepper_left.max_step_rate_hz + 1.0f) {
            result.fail("stepper.max_step_rate_hz is below the rate required by the "
                        "motion limits");
            return result;
        }
    }

    // --- map ----------------------------------------------------------------
    if (!isFinite(config.map_width_m) || config.map_width_m <= 0.0f) {
        result.fail("map_width_m must be > 0");
        return result;
    }
    if (!isFinite(config.map_height_m) || config.map_height_m <= 0.0f) {
        result.fail("map_height_m must be > 0");
        return result;
    }
    if (!isFinite(config.position.map_margin_m) || config.position.map_margin_m < 0.0f) {
        result.fail("position.map_margin_m must be >= 0");
        return result;
    }

    // --- PID ----------------------------------------------------------------
    if (!isFinite(config.heading_pid.kp) || !isFinite(config.heading_pid.ki) ||
        !isFinite(config.heading_pid.kd)) {
        result.fail("heading_pid gains must be finite");
        return result;
    }
    if (config.heading_pid.output_min >= config.heading_pid.output_max) {
        result.fail("heading_pid output_min must be < output_max");
        return result;
    }
    if (config.heading_pid.integral_min >= config.heading_pid.integral_max) {
        result.fail("heading_pid integral_min must be < integral_max");
        return result;
    }
    return result;
}

float clampControlDt(float dt_s, const ControlConfig& config) {
    const float nominal = 1.0f / config.navigation_rate_hz;
    if (!isFinite(dt_s) || dt_s < 1.0e-4f) {
        return nominal;
    }
    const float maximum = nominal * 4.0f;  // tolerate up to 4 missed cycles
    return dt_s > maximum ? maximum : dt_s;
}

}  // namespace nav
