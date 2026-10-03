// Position validation: turns a raw UWB sample into "usable" or "rejected".
//
// Rules implemented here (see docs/data_contract.md):
//   valid == false                      -> INVALID_FLAG
//   frame_id != "smart_market_map"      -> WRONG_FRAME_ID
//   trolley_id mismatch                 -> WRONG_TROLLEY_ID
//   schema_version mismatch             -> BAD_SCHEMA_VERSION
//   NaN / inf coordinates               -> NON_FINITE_COORDINATE
//   quality < minimum_quality           -> LOW_QUALITY
//   outside map rectangle + margin      -> OUT_OF_MAP_BOUNDS
//   timestamp older than stale_sample_ms-> STALE_TIMESTAMP
#pragma once

#include "navigation/navigation_config.hpp"
#include "navigation/types.hpp"

namespace nav {

struct MeasurementValidation {
    bool accepted{false};
    MeasurementStatus status{MeasurementStatus::NO_DATA};
};

/// Validate a measurement using the local monotonic clock for the staleness
/// check (`now_ms` is a local monotonic timestamp, not the remote one).
MeasurementValidation validateMeasurement(const PositionMeasurement& measurement,
                                          const PositionConfig& config,
                                          float map_width_m,
                                          float map_height_m,
                                          uint64_t now_ms);

/// Optional lightweight smoother for the navigation input.
///
/// The real EKF lives in the positioning subsystem; this is disabled by
/// default and only exists to test the navigation stack against unfiltered
/// coordinates.  It is a first-order exponential moving average.
class PositionFilter {
public:
    void configure(bool enabled, float alpha) {
        enabled_ = enabled;
        alpha_ = (alpha > 0.0f && alpha <= 1.0f) ? alpha : 1.0f;
        initialized_ = false;
    }

    void reset() { initialized_ = false; }

    Position2D apply(const Position2D& input) {
        if (!enabled_) {
            return input;
        }
        if (!initialized_) {
            filtered_ = input;
            initialized_ = true;
            return filtered_;
        }
        filtered_.x_m = alpha_ * input.x_m + (1.0f - alpha_) * filtered_.x_m;
        filtered_.y_m = alpha_ * input.y_m + (1.0f - alpha_) * filtered_.y_m;
        return filtered_;
    }

    bool enabled() const { return enabled_; }

private:
    bool enabled_{false};
    bool initialized_{false};
    float alpha_{0.5f};
    Position2D filtered_{};
};

}  // namespace nav
