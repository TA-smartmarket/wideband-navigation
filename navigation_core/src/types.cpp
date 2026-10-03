#include "navigation/types.hpp"

namespace nav {

const char* toString(MeasurementStatus status) {
    switch (status) {
        case MeasurementStatus::OK: return "ok";
        case MeasurementStatus::NO_DATA: return "no data";
        case MeasurementStatus::BAD_SCHEMA_VERSION: return "unsupported schema_version";
        case MeasurementStatus::WRONG_FRAME_ID: return "frame_id is not smart_market_map";
        case MeasurementStatus::WRONG_TROLLEY_ID: return "trolley_id mismatch";
        case MeasurementStatus::NON_FINITE_COORDINATE: return "coordinate is NaN or infinite";
        case MeasurementStatus::INVALID_FLAG: return "valid == false";
        case MeasurementStatus::LOW_QUALITY: return "quality below minimum";
        case MeasurementStatus::OUT_OF_MAP_BOUNDS: return "position outside map limits";
        case MeasurementStatus::STALE_TIMESTAMP: return "timestamp invalid or stale";
        default: return "unknown measurement status";
    }
}

const char* toString(NavigationError error) {
    switch (error) {
        case NavigationError::NONE: return "none";
        case NavigationError::INVALID_POSITION: return "invalid position";
        case NavigationError::POSITION_TIMEOUT: return "position timeout";
        case NavigationError::INVALID_DESTINATION: return "invalid destination";
        case NavigationError::START_NODE_NOT_FOUND: return "start node not found";
        case NavigationError::ROUTE_NOT_FOUND: return "route not found";
        case NavigationError::GRAPH_INVALID: return "graph invalid";
        case NavigationError::CONFIG_INVALID: return "config invalid";
        case NavigationError::NOT_READY: return "not ready";
        case NavigationError::CANCELLED: return "cancelled";
        case NavigationError::EMERGENCY_STOP_ACTIVE: return "emergency stop active";
        default: return "unknown navigation error";
    }
}

const char* toString(PathError error) {
    switch (error) {
        case PathError::NONE: return "ok";
        case PathError::INVALID_START: return "invalid start node";
        case PathError::INVALID_DESTINATION: return "invalid destination node";
        case PathError::START_EQUALS_DESTINATION: return "start equals destination";
        case PathError::NO_ROUTE: return "no route";
        case PathError::EMPTY_GRAPH: return "empty graph";
        default: return "unknown path error";
    }
}

const char* toString(NavState state) { return describeState(state); }

}  // namespace nav
