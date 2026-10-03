// Position providers for the firmware.
//
// `SerialPositionProvider` (in the navigation core) handles the documented JSON
// contract; this header adds the firmware-side glue:
//
//   * ArduinoSerialStream   - adapts the Arduino `HardwareSerial`/`Serial` API
//                             to nav::IByteStream
//   * RealPositionProvider  - the single integration point for the real UWB/EKF
//                             subsystem (see docs/integration.md)
//   * LatestPosition        - thread-safe holder used by the position task and
//                             the navigation task
#pragma once

#include <Arduino.h>

#include "app_config.hpp"
#include "navigation/position_provider.hpp"

namespace firmware {

/// Adapts any Arduino `Stream` to the core's byte-stream interface.
class ArduinoStreamAdapter : public nav::IByteStream {
public:
    explicit ArduinoStreamAdapter(Stream& stream) : stream_(stream) {}

    std::size_t read(char* buffer, std::size_t max_bytes) override {
        const int available = stream_.available();
        if (available <= 0) {
            return 0;
        }
        std::size_t wanted = static_cast<std::size_t>(available);
        if (wanted > max_bytes) {
            wanted = max_bytes;
        }
        const std::size_t read = stream_.readBytes(buffer, wanted);
        return read;
    }

    std::size_t write(const char* data, std::size_t length) override {
        return stream_.write(reinterpret_cast<const uint8_t*>(data), length);
    }

private:
    Stream& stream_;
};

/// Single integration point for the real positioning subsystem.
///
/// The UWB/EKF team publishes the documented JSON contract on a UART; this
/// provider reuses the same parser as the mock provider, so *only the transport*
/// differs between mock and real operation.  Swapping providers never touches
/// Dijkstra, the waypoint manager, the PID or the state machine.
class RealPositionProvider : public nav::IPositionProvider {
public:
    explicit RealPositionProvider(Stream& uwb_stream, const char* expected_trolley_id)
        : stream_(uwb_stream), parser_(&stream_, expected_trolley_id) {}

    bool poll(nav::PositionMeasurement& out, uint64_t now_ms) override {
        return parser_.poll(out, now_ms);
    }

    const char* name() const override { return "uwb-uart-json"; }

    nav::SerialPositionProvider& parser() { return parser_; }

private:
    ArduinoStreamAdapter stream_;
    nav::SerialPositionProvider parser_;
};

/// Thread-safe holder for the most recent accepted measurement.
///
/// The position task is the only writer and the navigation task the only reader,
/// so a short critical section (portMUX) is enough: no queue and no dynamic
/// allocation are needed for a single latest-value slot.
class LatestPosition {
public:
    void publish(const nav::PositionMeasurement& measurement, uint64_t now_ms) {
        portENTER_CRITICAL(&mux_);
        measurement_ = measurement;
        received_at_ms_ = now_ms;
        sequence_++;
        portEXIT_CRITICAL(&mux_);
    }

    /// Copy the latest measurement.  Returns false when nothing has arrived yet
    /// or when the sample has not changed since `last_sequence`.
    bool read(nav::PositionMeasurement& out, uint64_t& received_at_ms, uint32_t& sequence) {
        portENTER_CRITICAL(&mux_);
        const bool has_new = sequence_ != sequence;
        if (has_new) {
            out = measurement_;
            received_at_ms = received_at_ms_;
            sequence = sequence_;
        }
        portEXIT_CRITICAL(&mux_);
        return has_new;
    }

    /// Milliseconds since the newest accepted sample (UINT32_MAX when none).
    uint32_t ageMs(uint64_t now_ms) {
        portENTER_CRITICAL(&mux_);
        const uint64_t received = received_at_ms_;
        const uint32_t sequence = sequence_;
        portEXIT_CRITICAL(&mux_);
        if (sequence == 0) {
            return UINT32_MAX;
        }
        return now_ms >= received ? static_cast<uint32_t>(now_ms - received) : 0;
    }

    void clear() {
        portENTER_CRITICAL(&mux_);
        sequence_ = 0;
        received_at_ms_ = 0;
        portEXIT_CRITICAL(&mux_);
    }

private:
    portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
    nav::PositionMeasurement measurement_{};
    uint64_t received_at_ms_{0};
    uint32_t sequence_{0};
};

/// Number of malformed position lines observed (troubleshooting aid).
extern volatile uint32_t g_malformed_position_lines;

}  // namespace firmware
