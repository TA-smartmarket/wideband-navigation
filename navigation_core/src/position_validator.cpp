#include "navigation/position_validator.hpp"

#include "navigation/geometry.hpp"

namespace nav {

MeasurementValidation validateMeasurement(const PositionMeasurement& measurement,
                                          const PositionConfig& config,
                                          float map_width_m,
                                          float map_height_m,
                                          uint64_t now_ms) {
    MeasurementValidation result;

    // Order matters: cheapest and most fundamental checks first.
    if (measurement.schema_version != kSchemaVersion) {
        result.status = MeasurementStatus::BAD_SCHEMA_VERSION;
        return result;
    }
    if (!boundedEquals(measurement.frame_id, kFrameId)) {
        result.status = MeasurementStatus::WRONG_FRAME_ID;
        return result;
    }
    if (measurement.trolley_id[0] == '\0') {
        result.status = MeasurementStatus::WRONG_TROLLEY_ID;
        return result;
    }
    if (!isFinite(measurement.position.x_m) || !isFinite(measurement.position.y_m)) {
        result.status = MeasurementStatus::NON_FINITE_COORDINATE;
        return result;
    }
    if (!measurement.valid) {
        result.status = MeasurementStatus::INVALID_FLAG;
        return result;
    }
    if (!isFinite(measurement.quality) || measurement.quality < config.minimum_quality) {
        result.status = MeasurementStatus::LOW_QUALITY;
        return result;
    }

    // The map contract keeps the trolley inside [0, width] x [0, height]; a
    // generous margin is allowed because a trolley parked just outside the
    // rectangle is still recoverable, whereas a wild jump is not.
    const float margin = config.map_margin_m;
    if (measurement.position.x_m < -margin || measurement.position.x_m > map_width_m + margin ||
        measurement.position.y_m < -margin || measurement.position.y_m > map_height_m + margin) {
        result.status = MeasurementStatus::OUT_OF_MAP_BOUNDS;
        return result;
    }

    // Timestamp sanity: the sample must not come from the future and must not
    // be older than the configured staleness window.
    if (measurement.timestamp_ms == 0) {
        result.status = MeasurementStatus::STALE_TIMESTAMP;
        return result;
    }
    if (config.stale_sample_ms > 0 && now_ms > measurement.timestamp_ms &&
        (now_ms - measurement.timestamp_ms) > config.stale_sample_ms) {
        result.status = MeasurementStatus::STALE_TIMESTAMP;
        return result;
    }

    result.accepted = true;
    result.status = MeasurementStatus::OK;
    return result;
}

}  // namespace nav
