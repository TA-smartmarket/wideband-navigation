#include "navigation/config_loader.hpp"

#include <cstdio>
#include <cstring>

#include "navigation/json_parser.hpp"

namespace nav {

namespace {

void setError(char* error, std::size_t error_size, const char* format, const char* detail = nullptr) {
    if (error == nullptr || error_size == 0) {
        return;
    }
    if (detail != nullptr) {
        std::snprintf(error, error_size, format, detail);
    } else {
        std::snprintf(error, error_size, "%s", format);
    }
}

bool readFloat(const JsonValue& object, const char* key, float& out) {
    const JsonValue value = object.member(key);
    if (!value.isNumber()) {
        return false;
    }
    out = value.asFloat(out);
    return true;
}

}  // namespace

bool loadMapMetadata(const char* text, MapMetadata& out, char* error, std::size_t error_size) {
    const JsonValue root = jsonParse(text);
    if (!root.isObject()) {
        setError(error, error_size, "map.json: invalid JSON object");
        return false;
    }
    MapMetadata parsed;
    if (!root.member("map_id").asString(parsed.map_id, sizeof(parsed.map_id))) {
        setError(error, error_size, "map.json: missing map_id");
        return false;
    }
    parsed.map_version = root.member("map_version").asInt(0);
    if (parsed.map_version <= 0) {
        setError(error, error_size, "map.json: map_version must be >= 1");
        return false;
    }
    if (!root.member("frame_id").asString(parsed.frame_id, sizeof(parsed.frame_id))) {
        setError(error, error_size, "map.json: missing frame_id");
        return false;
    }
    if (!boundedEquals(parsed.frame_id, kFrameId)) {
        setError(error, error_size, "map.json: frame_id must be \"smart_market_map\"");
        return false;
    }
    if (!readFloat(root, "width_m", parsed.width_m) || parsed.width_m <= 0.0f) {
        setError(error, error_size, "map.json: width_m must be > 0");
        return false;
    }
    if (!readFloat(root, "height_m", parsed.height_m) || parsed.height_m <= 0.0f) {
        setError(error, error_size, "map.json: height_m must be > 0");
        return false;
    }
    readFloat(root, "origin_x_m", parsed.origin_x_m);
    readFloat(root, "origin_y_m", parsed.origin_y_m);
    out = parsed;
    return true;
}

bool loadGraph(const char* text, Graph& out, char* error, std::size_t error_size) {
    out.clear();
    const JsonValue root = jsonParse(text);
    if (!root.isObject()) {
        setError(error, error_size, "graph.json: invalid JSON object");
        return false;
    }

    // Optional metadata block.
    const JsonValue metadata = root.member("map");
    if (metadata.isObject()) {
        const JsonValue frame = metadata.member("frame_id");
        char frame_id[kFrameFieldSize];
        if (frame.isString() && frame.asString(frame_id, sizeof(frame_id)) &&
            !boundedEquals(frame_id, kFrameId)) {
            setError(error, error_size, "graph.json: map.frame_id must be \"smart_market_map\"");
            return false;
        }
    }

    const JsonValue nodes = root.member("nodes");
    if (!nodes.isArray() || nodes.size() == 0) {
        setError(error, error_size, "graph.json: \"nodes\" must be a non-empty array");
        return false;
    }
    const int node_count = nodes.size();
    for (int i = 0; i < node_count; ++i) {
        const JsonValue node = nodes.element(i);
        if (!node.isObject()) {
            setError(error, error_size, "graph.json: node entry is not an object");
            return false;
        }
        const int id = node.member("id").asInt(-1);
        float x = 0.0f;
        float y = 0.0f;
        if (id < 0) {
            setError(error, error_size, "graph.json: node without a valid \"id\"");
            return false;
        }
        if (!readFloat(node, "x_m", x) || !readFloat(node, "y_m", y)) {
            setError(error, error_size, "graph.json: node missing x_m/y_m");
            return false;
        }
        char type_text[24] = "intersection";
        node.member("type").asString(type_text, sizeof(type_text));
        const int index = out.addNode(id, x, y, nodeTypeFromString(type_text));
        if (index < 0) {
            char detail[96];
            std::snprintf(detail, sizeof(detail), "graph.json: cannot add node id=%d", id);
            setError(error, error_size, "%s", detail);
            return false;
        }
    }

    const JsonValue edges = root.member("edges");
    if (edges.isArray()) {
        const int edge_count = edges.size();
        for (int i = 0; i < edge_count; ++i) {
            const JsonValue edge = edges.element(i);
            if (!edge.isObject()) {
                setError(error, error_size, "graph.json: edge entry is not an object");
                return false;
            }
            const int from = edge.member("from").asInt(-1);
            const int to = edge.member("to").asInt(-1);
            if (from < 0 || to < 0) {
                setError(error, error_size, "graph.json: edge missing \"from\"/\"to\"");
                return false;
            }
            const JsonValue weight = edge.member("weight_m");
            float weight_m = -1.0f;  // negative => automatic Euclidean weight
            if (weight.isNumber()) {
                weight_m = weight.asFloat(-1.0f);
                if (weight_m < 0.0f) {
                    char detail[96];
                    std::snprintf(detail, sizeof(detail),
                                  "graph.json: edge %d->%d has a negative weight", from, to);
                    setError(error, error_size, "%s", detail);
                    return false;
                }
            }
            bool bidirectional = true;
            const JsonValue bidirectional_value = edge.member("bidirectional");
            if (bidirectional_value.isBoolean()) {
                bidirectional = bidirectional_value.asBool(true);
            }
            if (out.addEdge(from, to, weight_m, bidirectional) < 0) {
                char detail[128];
                std::snprintf(detail, sizeof(detail),
                              "graph.json: cannot add edge %d->%d (self loop, duplicate or unknown "
                              "node)",
                              from, to);
                setError(error, error_size, "%s", detail);
                return false;
            }
        }
    }

    const GraphValidation validation = out.validate();
    if (!validation.valid) {
        setError(error, error_size, "graph.json: %s", validation.message());
        out.clear();
        return false;
    }
    return true;
}

bool loadNavigationConfig(const char* text, NavigationConfig& out, char* error,
                          std::size_t error_size) {
    const JsonValue root = jsonParse(text);
    if (!root.isObject()) {
        setError(error, error_size, "navigation.json: invalid JSON object");
        return false;
    }
    NavigationConfig config;  // start from the tuned defaults

    const JsonValue position = root.member("position");
    if (position.isObject()) {
        readFloat(position, "minimum_quality", config.position.minimum_quality);
        const JsonValue timeout = position.member("timeout_ms");
        if (timeout.isNumber()) {
            config.position.timeout_ms =
                static_cast<uint32_t>(timeout.asInt(static_cast<int>(config.position.timeout_ms)));
        }
        readFloat(position, "heading_min_displacement_m", config.position.heading_min_displacement_m);
        const JsonValue stale = position.member("stale_sample_ms");
        if (stale.isNumber()) {
            config.position.stale_sample_ms = static_cast<uint32_t>(
                stale.asInt(static_cast<int>(config.position.stale_sample_ms)));
        }
        readFloat(position, "map_margin_m", config.position.map_margin_m);
        const JsonValue filter = position.member("filter");
        if (filter.isObject()) {
            config.position.enable_filter = filter.member("enable").asBool(config.position.enable_filter);
            readFloat(filter, "alpha", config.position.filter_alpha);
        }
    }

    const JsonValue path = root.member("path");
    if (path.isObject()) {
        readFloat(path, "max_graph_snap_distance_m", config.path.max_graph_snap_distance_m);
        readFloat(path, "waypoint_tolerance_m", config.path.waypoint_tolerance_m);
        readFloat(path, "destination_tolerance_m", config.path.destination_tolerance_m);
        readFloat(path, "max_cross_track_error_m", config.path.max_cross_track_error_m);
        readFloat(path, "replan_cross_track_error_m", config.path.replan_cross_track_error_m);
        const JsonValue confirm = path.member("route_deviation_confirm_ms");
        if (confirm.isNumber()) {
            config.path.route_deviation_confirm_ms =
                static_cast<uint32_t>(confirm.asInt(static_cast<int>(config.path.route_deviation_confirm_ms)));
        }
        const JsonValue cooldown = path.member("replan_cooldown_ms");
        if (cooldown.isNumber()) {
            config.path.replan_cooldown_ms =
                static_cast<uint32_t>(cooldown.asInt(static_cast<int>(config.path.replan_cooldown_ms)));
        }
        const JsonValue lookahead = path.member("lookahead");
        if (lookahead.isObject()) {
            config.path.enable_lookahead = lookahead.member("enable").asBool(config.path.enable_lookahead);
            readFloat(lookahead, "distance_m", config.path.lookahead_distance_m);
        }
        readFloat(path, "corner_slowdown_distance_m", config.path.corner_slowdown_distance_m);
        readFloat(path, "destination_slowdown_distance_m",
                  config.path.destination_slowdown_distance_m);
    }

    const JsonValue motion = root.member("motion");
    if (motion.isObject()) {
        readFloat(motion, "max_linear_speed_mps", config.motion.max_linear_speed_mps);
        readFloat(motion, "min_linear_speed_mps", config.motion.min_linear_speed_mps);
        readFloat(motion, "max_angular_speed_radps", config.motion.max_angular_speed_radps);
        readFloat(motion, "rotate_in_place_threshold_deg", config.motion.rotate_in_place_threshold_deg);
        readFloat(motion, "heading_error_slow_deg", config.motion.heading_error_slow_deg);
        readFloat(motion, "heading_kp", config.motion.heading_kp);
        readFloat(motion, "max_linear_accel_mps2", config.motion.max_linear_accel_mps2);
        readFloat(motion, "max_angular_accel_radps2", config.motion.max_angular_accel_radps2);
        readFloat(motion, "heading_deadband_deg", config.motion.heading_deadband_deg);
        readFloat(motion, "max_motor_command_change_per_s",
                  config.motion.max_motor_command_change_per_s);
    }

    const JsonValue robot = root.member("robot");
    if (robot.isObject()) {
        readFloat(robot, "wheel_radius_m", config.drive.wheel_radius_m);
        readFloat(robot, "wheel_base_m", config.drive.wheel_base_m);
        readFloat(robot, "max_wheel_speed_mps", config.drive.max_wheel_speed_mps);
        readFloat(robot, "motor_deadband_mps", config.drive.deadband_mps);
        readFloat(robot, "left_gain", config.drive.left_gain);
        readFloat(robot, "right_gain", config.drive.right_gain);
    }

    // Actuator selection and stepper parameters.
    char drive_kind[24] = "pwm_h_bridge";
    root.member("drive_kind").asString(drive_kind, sizeof(drive_kind));
    if (std::strcmp(drive_kind, "stepper") == 0 || std::strcmp(drive_kind, "stepper_step_dir") == 0) {
        config.drive_kind = DriveKind::STEPPER_STEP_DIR;
    } else {
        config.drive_kind = DriveKind::PWM_H_BRIDGE;
    }

    const JsonValue stepper = root.member("stepper");
    if (stepper.isObject()) {
        // Shared defaults for both axes, then per-axis overrides.
        const JsonValue common = stepper.member("common");
        auto readAxis = [](const JsonValue& block, StepperConfig& axis) {
            if (!block.isObject()) {
                return;
            }
            const JsonValue spr = block.member("steps_per_revolution");
            if (spr.isNumber()) {
                axis.steps_per_revolution = spr.asInt(axis.steps_per_revolution);
            }
            const JsonValue micro = block.member("microsteps");
            if (micro.isNumber()) {
                axis.microsteps = micro.asInt(axis.microsteps);
            }
            readFloat(block, "max_step_rate_hz", axis.max_step_rate_hz);
            readFloat(block, "min_step_rate_hz", axis.min_step_rate_hz);
            readFloat(block, "max_step_accel_hz_per_s", axis.max_step_accel_hz_per_s);
            const JsonValue invert = block.member("invert_direction");
            if (invert.isBoolean()) {
                axis.invert_direction = invert.asBool(axis.invert_direction);
            }
        };
        readAxis(common, config.stepper_left);
        readAxis(common, config.stepper_right);
        readAxis(stepper.member("left"), config.stepper_left);
        readAxis(stepper.member("right"), config.stepper_right);
        // The wheel radius is shared with the chassis configuration.
        config.stepper_left.wheel_radius_m = config.drive.wheel_radius_m;
        config.stepper_right.wheel_radius_m = config.drive.wheel_radius_m;
        config.enable_step_odometry =
            stepper.member("enable_odometry").asBool(config.enable_step_odometry);
    }

    const JsonValue control = root.member("control");
    if (control.isObject()) {
        readFloat(control, "navigation_rate_hz", config.control.navigation_rate_hz);
        readFloat(control, "telemetry_rate_hz", config.control.telemetry_rate_hz);
    }

    const JsonValue pid = root.member("heading_pid");
    if (pid.isObject()) {
        // `motion.heading_kp` is the authoritative proportional gain: it is the
        // documented tuning knob and the value the speed policy shares.  Reading
        // `kp` here as well created two sources of truth, so editing
        // motion.heading_kp appeared to have no effect.
        readFloat(pid, "ki", config.heading_pid.ki);
        readFloat(pid, "kd", config.heading_pid.kd);
        readFloat(pid, "derivative_filter_alpha", config.heading_pid.derivative_filter_alpha);
        readFloat(pid, "integral_limit", config.heading_pid.integral_max);
        config.heading_pid.integral_min = -config.heading_pid.integral_max;
    }

    const JsonValue map = root.member("map");
    if (map.isObject()) {
        readFloat(map, "width_m", config.map_width_m);
        readFloat(map, "height_m", config.map_height_m);
    }

    // The proportional gain, the output saturation and the sample time always
    // follow the motion/control sections: one source of truth per parameter.
    config.heading_pid.kp = config.motion.heading_kp;
    config.heading_pid.output_min = -config.motion.max_angular_speed_radps;
    config.heading_pid.output_max = config.motion.max_angular_speed_radps;
    config.heading_pid.nominal_dt_s = 1.0f / config.control.navigation_rate_hz;

    const ConfigValidation validation = validateConfig(config);
    if (!validation.valid) {
        setError(error, error_size, "navigation.json: %s", validation.message);
        return false;
    }
    out = config;
    return true;
}

}  // namespace nav
