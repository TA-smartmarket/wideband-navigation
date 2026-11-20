// Telemetry publishing for the firmware.
//
// Kept off the navigation task: the control loop only writes a status snapshot
// and the telemetry task formats/publishes it, so a slow serial port can never
// delay motor control.
//
// The command parser itself lives in the navigation core
// (navigation/command_parser.hpp) because it is pure text logic with no hardware
// dependency, which makes it unit-testable on the host.  This header only adds
// the firmware-side output sink.
#pragma once

#include <Arduino.h>

#include "app_config.hpp"
#include "navigation/command_parser.hpp"
#include "navigation/navigation_fsm.hpp"
#include "navigation/telemetry.hpp"
#include "serial_output.hpp"
#include "wheel_control.hpp"

namespace firmware {

/// Destination of telemetry output (the console UART by default).
class ITelemetrySink {
public:
    virtual ~ITelemetrySink() = default;
    virtual void write(const char* text) = 0;
    virtual const char* name() const = 0;
};

/// Console UART sink.
class SerialTelemetrySink : public ITelemetrySink {
public:
    explicit SerialTelemetrySink(Stream& stream) : stream_(stream) {}
    void write(const char* text) override {
        SerialOutputGuard guard;
        stream_.print(text);
    }
    const char* name() const override { return "serial"; }

private:
    Stream& stream_;
};

/// Formats and publishes telemetry for one navigation session.
class TelemetryPublisher {
public:
    explicit TelemetryPublisher(ITelemetrySink* sink, ITelemetrySink* mirror = nullptr)
        : sink_(sink), mirror_(mirror), publish_mutex_(xSemaphoreCreateMutex()) {}

    /// Emit one JSON telemetry line (the documented contract).
    void publishJson(const nav::NavigationStatus& status, uint64_t timestamp_ms);
    /// Emit measured AS5600, wheel-PID, STEP and encoder-odometry telemetry.
    void publishWheelJson(const WheelControlSnapshot& status, uint64_t timestamp_ms);
    void publishVerificationJson(unsigned source_mode, bool verified, bool uwb_valid, float uwb_x_m,
                                 float uwb_y_m, float uwb_quality, float disagreement_m,
                                 uint32_t uwb_age_ms, uint64_t timestamp_ms);

    /// Emit the human readable status block (the `status` command).
    void publishStatusText(const nav::NavigationStatus& status);

    /// Emit a short single-line event notice.
    void publishEvent(const char* tag, const char* message);

    /// Let an optional non-blocking mirror advance its transport. Call only
    /// from the telemetry/I/O task, never the navigation control task.
    void serviceMirror();

    uint32_t publishedCount() const { return published_count_; }

private:
    void write(const char* text);

    ITelemetrySink* sink_{nullptr};
    ITelemetrySink* mirror_{nullptr};
    char buffer_[TELEMETRY_BUFFER]{};
    uint32_t published_count_{0};
    SemaphoreHandle_t publish_mutex_{nullptr};
};

}  // namespace firmware
