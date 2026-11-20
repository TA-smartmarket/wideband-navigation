#include "telemetry.hpp"

#include <cstring>

#include "navigation/geometry.hpp"
#include "navigation/json_parser.hpp"
#include "navigation/logging.hpp"

namespace firmware {

namespace {
constexpr const char* kTag = "TELEM";

class SemaphoreGuard {
public:
    explicit SemaphoreGuard(SemaphoreHandle_t mutex) : mutex_(mutex) {
        locked_ = mutex_ != nullptr && xSemaphoreTake(mutex_, portMAX_DELAY) == pdTRUE;
    }
    ~SemaphoreGuard() {
        if (locked_) xSemaphoreGive(mutex_);
    }
private:
    SemaphoreHandle_t mutex_{nullptr};
    bool locked_{false};
};
}  // namespace

// ---------------------------------------------------------------------------
// TelemetryPublisher
// ---------------------------------------------------------------------------

void TelemetryPublisher::write(const char* text) {
    if (sink_ != nullptr) sink_->write(text);
    if (mirror_ != nullptr) mirror_->write(text);
}

void TelemetryPublisher::serviceMirror() {
    // A mirror can implement deferred delivery in write(); the concrete MQTT
    // sink is serviced explicitly by the telemetry task in main.cpp.
}

void TelemetryPublisher::publishJson(const nav::NavigationStatus& status, uint64_t timestamp_ms) {
    SemaphoreGuard guard(publish_mutex_);
    if (sink_ == nullptr) {
        return;
    }
    const int written = nav::statusToJson(status, TROLLEY_ID, timestamp_ms, buffer_, sizeof(buffer_));
    if (written <= 0) {
        NAV_LOG_WARN(kTag, "telemetry buffer too small; line dropped");
        return;
    }
    // Keep the established navigation fields flat for current consumers while
    // adding an unambiguous packet discriminator for the laptop dashboard.
    const std::size_t length = std::strlen(buffer_);
    constexpr const char kProtocolFields[] = ",\"type\":\"telemetry\",\"protocol_version\":1}\n";
    if (length == 0 || buffer_[length - 1] != '}' || length + sizeof(kProtocolFields) > sizeof(buffer_)) {
        NAV_LOG_WARN(kTag, "telemetry buffer has no room for protocol fields; line dropped");
        return;
    }
    buffer_[length - 1] = '\0';
    std::strcat(buffer_, kProtocolFields);
    write(buffer_);
    ++published_count_;
}

void TelemetryPublisher::publishWheelJson(const WheelControlSnapshot& s,
                                          uint64_t timestamp_ms) {
    SemaphoreGuard guard(publish_mutex_);
    if (sink_ == nullptr) return;
    std::snprintf(
        buffer_, sizeof(buffer_),
        "{\"type\":\"wheel_telemetry\",\"schema_version\":1,\"timestamp_ms\":%llu,"
        "\"encoder_healthy\":%s,"
        "\"left\":{\"target_rpm\":%.3f,\"actual_rpm\":%.3f,\"error_rpm\":%.3f,"
        "\"p\":%.3f,\"i\":%.3f,\"d\":%.3f,\"output_rpm\":%.3f,"
        "\"saturated\":%s,\"tracking_fault\":%s,\"raw_angle\":%u,"
        "\"revolutions\":%.6f,\"step_count\":%llu},"
        "\"right\":{\"target_rpm\":%.3f,\"actual_rpm\":%.3f,\"error_rpm\":%.3f,"
        "\"p\":%.3f,\"i\":%.3f,\"d\":%.3f,\"output_rpm\":%.3f,"
        "\"saturated\":%s,\"tracking_fault\":%s,\"raw_angle\":%u,"
        "\"revolutions\":%.6f,\"step_count\":%llu},"
        "\"odometry_pose\":{\"x_m\":%.4f,\"y_m\":%.4f,\"heading_rad\":%.5f,"
        "\"valid\":%s,\"estimated\":true},\"pose_reset_count\":%lu,\"deadline_misses\":%lu}\n",
        static_cast<unsigned long long>(timestamp_ms),
        s.encoder_healthy ? "true" : "false",
        static_cast<double>(s.left.target_rpm), static_cast<double>(s.left.measured_rpm),
        static_cast<double>(s.left.error_rpm), static_cast<double>(s.left.p_term),
        static_cast<double>(s.left.i_term), static_cast<double>(s.left.d_term),
        static_cast<double>(s.left.output_rpm), s.left.saturated ? "true" : "false",
        s.left.tracking_fault ? "true" : "false", s.left_encoder.raw_count,
        static_cast<double>(s.left_encoder.wheel_revolutions),
        static_cast<unsigned long long>(s.left_step_count),
        static_cast<double>(s.right.target_rpm), static_cast<double>(s.right.measured_rpm),
        static_cast<double>(s.right.error_rpm), static_cast<double>(s.right.p_term),
        static_cast<double>(s.right.i_term), static_cast<double>(s.right.d_term),
        static_cast<double>(s.right.output_rpm), s.right.saturated ? "true" : "false",
        s.right.tracking_fault ? "true" : "false", s.right_encoder.raw_count,
        static_cast<double>(s.right_encoder.wheel_revolutions),
        static_cast<unsigned long long>(s.right_step_count),
        static_cast<double>(s.odometry.pose.x_m), static_cast<double>(s.odometry.pose.y_m),
        static_cast<double>(s.odometry.pose.heading_rad), s.odometry.valid ? "true" : "false",
        static_cast<unsigned long>(s.pose_reset_count),
        static_cast<unsigned long>(s.deadline_misses));
    write(buffer_);
    ++published_count_;
}

void TelemetryPublisher::publishVerificationJson(unsigned source_mode, bool verified, bool uwb_valid,
                                                 float uwb_x_m, float uwb_y_m,
                                                 float uwb_quality, float disagreement_m,
                                                 uint32_t uwb_age_ms,
                                                 uint64_t timestamp_ms) {
    SemaphoreGuard guard(publish_mutex_);
    if (sink_ == nullptr) return;
    std::snprintf(buffer_, sizeof(buffer_),
        "{\"type\":\"position_verification\",\"timestamp_ms\":%llu,\"source_mode\":%u,"
        "\"verified\":%s,\"uwb_valid\":%s,\"uwb_position\":{\"x_m\":%.4f,\"y_m\":%.4f},\"uwb_quality\":%.3f,"
        "\"disagreement_m\":%.4f,\"uwb_age_ms\":%lu}\n",
        static_cast<unsigned long long>(timestamp_ms), source_mode, verified ? "true" : "false",
        uwb_valid ? "true" : "false", static_cast<double>(uwb_x_m),
        static_cast<double>(uwb_y_m), static_cast<double>(uwb_quality),
        static_cast<double>(disagreement_m),
        static_cast<unsigned long>(uwb_age_ms));
    write(buffer_);
}

void TelemetryPublisher::publishStatusText(const nav::NavigationStatus& status) {
    SemaphoreGuard guard(publish_mutex_);
    if (sink_ == nullptr) {
        return;
    }
    const int written = nav::statusToText(status, TROLLEY_ID, buffer_, sizeof(buffer_));
    if (written <= 0) {
        return;
    }
    write(buffer_);
}

void TelemetryPublisher::publishEvent(const char* tag, const char* message) {
    SemaphoreGuard guard(publish_mutex_);
    if (sink_ == nullptr) {
        return;
    }
    // Bounded formatting: snprintf never overflows and always terminates.
    std::snprintf(buffer_, sizeof(buffer_), "[EVENT][%s] %s\n",
                  tag == nullptr ? "?" : tag, message == nullptr ? "" : message);
    write(buffer_);
}

}  // namespace firmware
