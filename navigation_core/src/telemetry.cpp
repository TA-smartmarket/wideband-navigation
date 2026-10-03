#include "navigation/telemetry.hpp"

#include <cstdio>

#include "navigation/geometry.hpp"

namespace nav {

namespace {
constexpr float kRadToDeg = 57.29577951308232f;
}

int statusToJson(const NavigationStatus& status,
                 const char* trolley_id,
                 uint64_t timestamp_ms,
                 char* out,
                 std::size_t out_size) {
    if (out == nullptr || out_size == 0) {
        return 0;
    }
    const int written = std::snprintf(
        out, out_size,
        "{\"schema_version\":%u,\"trolley_id\":\"%s\",\"frame_id\":\"%s\",\"timestamp_ms\":%llu,"
        "\"navigation\":{\"state\":\"%s\",\"error\":\"%s\",\"destination_node\":%d,"
        "\"active_waypoint_node\":%d,\"waypoint_index\":%d,\"waypoint_count\":%d,"
        "\"distance_to_waypoint_m\":%.3f,\"distance_to_destination_m\":%.3f,"
        "\"cross_track_error_m\":%.3f,\"planned_distance_m\":%.3f,"
        "\"remaining_distance_m\":%.3f,\"travelled_distance_m\":%.3f,"
        "\"heading_error_rad\":%.3f,\"replan_count\":%u,\"position_loss_events\":%u,"
        "\"invalid_sample_count\":%u,\"plan_time_us\":%u,\"route_length\":%d},"
        "\"pose\":{\"x_m\":%.3f,\"y_m\":%.3f,\"heading_rad\":%.3f,\"heading_valid\":%s},"
        "\"control\":{\"linear_velocity_mps\":%.3f,\"angular_velocity_radps\":%.3f,"
        "\"left_motor\":%.3f,\"right_motor\":%.3f,\"rotate_in_place\":%s},"
        "\"position_quality\":%.3f,\"position_valid\":%s,\"position_fresh\":%s}",
        static_cast<unsigned>(kSchemaVersion), trolley_id == nullptr ? kDefaultTrolleyId : trolley_id,
        kFrameId, static_cast<unsigned long long>(timestamp_ms), describeState(status.state),
        toString(status.error), status.destination_node, status.active_waypoint_node,
        status.waypoint_index, status.waypoint_count,
        static_cast<double>(sanitizeFinite(status.distance_to_waypoint_m)),
        static_cast<double>(sanitizeFinite(status.distance_to_destination_m)),
        static_cast<double>(sanitizeFinite(status.cross_track_error_m)),
        static_cast<double>(sanitizeFinite(status.planned_distance_m)),
        static_cast<double>(sanitizeFinite(status.remaining_distance_m)),
        static_cast<double>(sanitizeFinite(status.travelled_distance_m)),
        static_cast<double>(sanitizeFinite(status.heading_error_rad)),
        static_cast<unsigned>(status.replan_count),
        static_cast<unsigned>(status.position_loss_events),
        static_cast<unsigned>(status.invalid_sample_count),
        static_cast<unsigned>(status.plan_time_us), status.route_length,
        static_cast<double>(sanitizeFinite(status.position.x_m)),
        static_cast<double>(sanitizeFinite(status.position.y_m)),
        static_cast<double>(sanitizeFinite(status.heading_rad)),
        status.heading_valid ? "true" : "false",
        static_cast<double>(sanitizeFinite(status.linear_velocity_mps)),
        static_cast<double>(sanitizeFinite(status.angular_velocity_radps)),
        static_cast<double>(sanitizeFinite(status.motor.left)),
        static_cast<double>(sanitizeFinite(status.motor.right)),
        status.rotate_in_place ? "true" : "false",
        static_cast<double>(sanitizeFinite(status.position_quality)),
        status.position_valid ? "true" : "false",
        status.position_fresh ? "true" : "false");
    if (written < 0) {
        out[0] = '\0';
        return 0;
    }
    return written;
}

int statusToText(const NavigationStatus& status, const char* trolley_id, char* out,
                 std::size_t out_size) {
    if (out == nullptr || out_size == 0) {
        return 0;
    }
    const int written = std::snprintf(
        out, out_size,
        "Trolley: %s\n"
        "State: %s\n"
        "Error: %s\n"
        "Position: (%.2f, %.2f)\n"
        "Heading: %.1f deg%s\n"
        "Start Node: %d\n"
        "Destination Node: %d\n"
        "Current Waypoint: %d\n"
        "Waypoint: %d/%d\n"
        "Distance to Waypoint: %.2f m\n"
        "Distance to Destination: %.2f m\n"
        "Cross Track Error: %.2f m\n"
        "Planned Distance: %.2f m\n"
        "Remaining Distance: %.2f m\n"
        "Travelled Distance: %.2f m\n"
        "UWB Quality: %.2f\n"
        "Position Valid: %s\n"
        "Left Motor: %.2f\n"
        "Right Motor: %.2f\n"
        "Replans: %u\n"
        "Position Loss Events: %u\n",
        trolley_id == nullptr ? kDefaultTrolleyId : trolley_id, describeState(status.state),
        toString(status.error), static_cast<double>(sanitizeFinite(status.position.x_m)),
        static_cast<double>(sanitizeFinite(status.position.y_m)),
        static_cast<double>(sanitizeFinite(status.heading_rad) * kRadToDeg),
        status.heading_valid ? "" : " (estimated, not yet valid)", status.start_node,
        status.destination_node, status.active_waypoint_node, status.waypoint_index,
        status.waypoint_count, static_cast<double>(sanitizeFinite(status.distance_to_waypoint_m)),
        static_cast<double>(sanitizeFinite(status.distance_to_destination_m)),
        static_cast<double>(sanitizeFinite(status.cross_track_error_m)),
        static_cast<double>(sanitizeFinite(status.planned_distance_m)),
        static_cast<double>(sanitizeFinite(status.remaining_distance_m)),
        static_cast<double>(sanitizeFinite(status.travelled_distance_m)),
        static_cast<double>(sanitizeFinite(status.position_quality)),
        status.position_valid ? "yes" : "no",
        static_cast<double>(sanitizeFinite(status.motor.left)),
        static_cast<double>(sanitizeFinite(status.motor.right)),
        static_cast<unsigned>(status.replan_count),
        static_cast<unsigned>(status.position_loss_events));
    if (written < 0) {
        out[0] = '\0';
        return 0;
    }
    return written;
}

}  // namespace nav
