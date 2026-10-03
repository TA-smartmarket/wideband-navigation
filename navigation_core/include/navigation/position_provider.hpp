// Position provider abstraction + the concrete providers used in development.
//
// The navigation core only ever sees `PositionMeasurement`, therefore swapping
// the mock/serial provider for the real UWB/EKF adapter requires no change to
// the planner, follower, PID or FSM (see docs/integration.md).
#pragma once

#include <cstddef>
#include <cstdint>

#include "navigation/json_parser.hpp"
#include "navigation/types.hpp"

namespace nav {

/// Source of processed trolley positions.
class IPositionProvider {
public:
    virtual ~IPositionProvider() = default;

    /// Non-blocking poll.  Returns true when `out` was filled with a new sample.
    virtual bool poll(PositionMeasurement& out, uint64_t now_ms) = 0;

    /// Human readable provider name (telemetry / logs).
    virtual const char* name() const = 0;
};

/// Byte-stream sink abstraction so the providers do not depend on Arduino's
/// Serial or on a particular socket API.
class IByteStream {
public:
    virtual ~IByteStream() = default;
    /// Read up to `max_bytes`; returns the number of bytes actually read.
    virtual std::size_t read(char* buffer, std::size_t max_bytes) = 0;
    /// Write bytes; returns the number written.
    virtual std::size_t write(const char* data, std::size_t length) = 0;
};

/// Parses newline-delimited JSON position samples from an `IByteStream`.
///
/// Bounded line buffer, no dynamic allocation, partial lines are preserved
/// across polls so a line split by the transport is reassembled correctly.
class SerialPositionProvider : public IPositionProvider {
public:
    explicit SerialPositionProvider(IByteStream* stream, const char* expected_trolley_id = kDefaultTrolleyId)
        : stream_(stream) {
        copyBounded(expected_trolley_id_, sizeof(expected_trolley_id_), expected_trolley_id);
    }

    bool poll(PositionMeasurement& out, uint64_t now_ms) override;
    const char* name() const override { return "serial-json"; }

    /// Statistics for telemetry/troubleshooting.
    uint32_t parsedLines() const { return parsed_lines_; }
    uint32_t malformedLines() const { return malformed_lines_; }
    uint32_t droppedLines() const { return dropped_lines_; }

    /// Parse a single line (without the trailing newline).  Exposed for tests.
    bool parseLine(const char* line, std::size_t length, PositionMeasurement& out);

    static constexpr std::size_t kLineBufferSize = 320;
    /// Byte budget per poll: bounds the work done inside one control cycle so a
    /// flooding peer cannot starve the navigation task.
    static constexpr std::size_t kMaxBytesPerPoll = 1024;
    /// Depth of the reassembled-sample FIFO (a burst of lines is not lost).
    static constexpr int kPendingCapacity = 4;

private:
    void pushPending(const PositionMeasurement& measurement);

    IByteStream* stream_{nullptr};
    char line_buffer_[kLineBufferSize]{};
    std::size_t line_length_{0};
    char expected_trolley_id_[kIdFieldSize]{};
    PositionMeasurement pending_[kPendingCapacity]{};
    int pending_head_{0};
    int pending_count_{0};
    uint32_t parsed_lines_{0};
    uint32_t malformed_lines_{0};
    uint32_t dropped_lines_{0};
};

/// A fixed in-memory sample used by the firmware's mock mode and by tests.
class MockPositionProvider : public IPositionProvider {
public:
    MockPositionProvider() = default;

    /// Install the sample returned by subsequent polls.
    void setMeasurement(const PositionMeasurement& measurement);

    /// Convenience: build a valid sample from raw values.
    void setPosition(float x_m, float y_m, float quality = 0.95f, uint64_t timestamp_ms = 0);

    /// Repeat the sample on every poll instead of delivering it once.
    void setRepeat(bool repeat) { repeat_ = repeat; }

    bool poll(PositionMeasurement& out, uint64_t now_ms) override;
    const char* name() const override { return "mock"; }

    uint32_t pollCount() const { return poll_count_; }

private:
    PositionMeasurement sample_{};
    bool has_sample_{false};
    bool unread_{false};
    bool repeat_{false};
    uint32_t poll_count_{0};
};

/// Replays a pre-recorded sequence of samples (used by the integration tests
/// and the firmware hardware-in-the-loop test mode).
class ReplayPositionProvider : public IPositionProvider {
public:
    ReplayPositionProvider() = default;

    /// Load a sequence; returns false when the array is empty or too large.
    bool load(const PositionMeasurement* samples, int count);

    bool poll(PositionMeasurement& out, uint64_t now_ms) override;
    const char* name() const override { return "replay"; }

    bool finished() const { return index_ >= count_; }
    void rewind() { index_ = 0; }

private:
    PositionMeasurement samples_[512]{};
    int count_{0};
    int index_{0};
};

/// Builds a `PositionMeasurement` from a JSON line using the documented contract.
/// Returns false when the object is malformed or a required field is missing.
bool measurementFromJson(const JsonValue& root, PositionMeasurement& out);

/// Serialise a measurement to JSON (single line, no trailing newline).
/// Returns the number of characters written (excluding the NUL terminator).
int measurementToJson(const PositionMeasurement& measurement, char* out, std::size_t out_size);

}  // namespace nav
