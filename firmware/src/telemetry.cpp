#include "telemetry.hpp"

#include <cstring>

#include "navigation/geometry.hpp"
#include "navigation/json_parser.hpp"
#include "navigation/logging.hpp"

namespace firmware {

namespace {
constexpr const char* kTag = "TELEM";
}  // namespace

// ---------------------------------------------------------------------------
// TelemetryPublisher
// ---------------------------------------------------------------------------

void TelemetryPublisher::publishJson(const nav::NavigationStatus& status, uint64_t timestamp_ms) {
    if (sink_ == nullptr) {
        return;
    }
    const int written = nav::statusToJson(status, TROLLEY_ID, timestamp_ms, buffer_, sizeof(buffer_));
    if (written <= 0) {
        NAV_LOG_WARN(kTag, "telemetry buffer too small; line dropped");
        return;
    }
    sink_->write(buffer_);
    sink_->write("\n");
    ++published_count_;
}

void TelemetryPublisher::publishStatusText(const nav::NavigationStatus& status) {
    if (sink_ == nullptr) {
        return;
    }
    const int written = nav::statusToText(status, TROLLEY_ID, buffer_, sizeof(buffer_));
    if (written <= 0) {
        return;
    }
    sink_->write(buffer_);
}

void TelemetryPublisher::publishEvent(const char* tag, const char* message) {
    if (sink_ == nullptr) {
        return;
    }
    // Bounded formatting: snprintf never overflows and always terminates.
    std::snprintf(buffer_, sizeof(buffer_), "[EVENT][%s] %s\n",
                  tag == nullptr ? "?" : tag, message == nullptr ? "" : message);
    sink_->write(buffer_);
}

}  // namespace firmware
