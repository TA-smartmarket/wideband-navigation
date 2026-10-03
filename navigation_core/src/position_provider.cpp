#include "navigation/position_provider.hpp"

#include <cstdio>
#include <cstring>

namespace nav {

bool measurementFromJson(const JsonValue& root, PositionMeasurement& out) {
    if (!root.isObject()) {
        return false;
    }
    const JsonValue schema = root.member("schema_version");
    const JsonValue position = root.member("position");
    if (!schema.isNumber() || !position.isObject()) {
        return false;
    }
    const JsonValue x = position.member("x_m");
    const JsonValue y = position.member("y_m");
    if (!x.isNumber() || !y.isNumber()) {
        return false;
    }

    PositionMeasurement parsed;
    parsed.schema_version = static_cast<uint32_t>(schema.asInt(static_cast<int>(kSchemaVersion)));
    if (!root.member("trolley_id").asString(parsed.trolley_id, sizeof(parsed.trolley_id))) {
        return false;
    }
    if (!root.member("frame_id").asString(parsed.frame_id, sizeof(parsed.frame_id))) {
        return false;
    }
    const JsonValue timestamp = root.member("timestamp_ms");
    if (!timestamp.isNumber()) {
        return false;
    }
    parsed.timestamp_ms = timestamp.asUint64(0);
    parsed.position.x_m = x.asFloat(0.0f);
    parsed.position.y_m = y.asFloat(0.0f);
    parsed.quality = root.member("quality").asFloat(0.0f);
    parsed.valid = root.member("valid").asBool(false);

    out = parsed;
    return true;
}

int measurementToJson(const PositionMeasurement& measurement, char* out, std::size_t out_size) {
    if (out == nullptr || out_size == 0) {
        return 0;
    }
    const int written = std::snprintf(
        out, out_size,
        "{\"schema_version\":%u,\"trolley_id\":\"%s\",\"frame_id\":\"%s\",\"timestamp_ms\":%llu,"
        "\"position\":{\"x_m\":%.4f,\"y_m\":%.4f},\"quality\":%.3f,\"valid\":%s}",
        static_cast<unsigned>(measurement.schema_version), measurement.trolley_id,
        measurement.frame_id, static_cast<unsigned long long>(measurement.timestamp_ms),
        static_cast<double>(sanitizeFinite(measurement.position.x_m)),
        static_cast<double>(sanitizeFinite(measurement.position.y_m)),
        static_cast<double>(sanitizeFinite(measurement.quality)),
        measurement.valid ? "true" : "false");
    if (written < 0) {
        out[0] = '\0';
        return 0;
    }
    return written;
}

// ---------------------------------------------------------------------------
// SerialPositionProvider
// ---------------------------------------------------------------------------

bool SerialPositionProvider::parseLine(const char* line, std::size_t length, PositionMeasurement& out) {
    if (line == nullptr || length == 0) {
        return false;
    }
    // Accept an optional "POS:" prefix so debug commands can share the link.
    const char* cursor = line;
    std::size_t remaining = length;
    if (remaining >= 4 && std::strncmp(cursor, "POS:", 4) == 0) {
        cursor += 4;
        remaining -= 4;
    }
    while (remaining > 0 && (*cursor == ' ' || *cursor == '\t')) {
        ++cursor;
        --remaining;
    }
    if (remaining == 0) {
        return false;
    }
    const JsonValue root = jsonParse(cursor, remaining);
    if (!measurementFromJson(root, out)) {
        return false;
    }
    return true;
}

void SerialPositionProvider::pushPending(const PositionMeasurement& measurement) {
    if (pending_count_ >= kPendingCapacity) {
        // FIFO full: the consumer is not polling fast enough.  Drop the oldest
        // sample and keep the newest, which is the one navigation needs.
        pending_head_ = (pending_head_ + 1) % kPendingCapacity;
        --pending_count_;
        ++dropped_lines_;
    }
    pending_[(pending_head_ + pending_count_) % kPendingCapacity] = measurement;
    ++pending_count_;
}

bool SerialPositionProvider::poll(PositionMeasurement& out, uint64_t now_ms) {
    (void)now_ms;
    if (stream_ == nullptr) {
        return false;
    }

    // Deliver a previously reassembled sample before touching the transport.
    if (pending_count_ > 0) {
        out = pending_[pending_head_];
        pending_head_ = (pending_head_ + 1) % kPendingCapacity;
        --pending_count_;
        return true;
    }

    // Bounded byte budget per poll: guarantees the control loop cannot be
    // starved by a peer flooding the link, while still allowing a full line to
    // be reassembled from a transport that delivers few bytes per read.
    // The loop stops as soon as the transport reports "nothing available".
    std::size_t bytes_read = 0;
    while (bytes_read < kMaxBytesPerPoll) {
        if (line_length_ >= kLineBufferSize - 1) {
            // The buffer is full without a newline: the peer is sending
            // something that is not a position line.  Drop it rather than
            // corrupting the next parse.
            line_length_ = 0;
            ++dropped_lines_;
        }
        char chunk[64];
        const std::size_t budget = kMaxBytesPerPoll - bytes_read;
        const std::size_t request = budget < sizeof(chunk) ? budget : sizeof(chunk);
        const std::size_t read = stream_->read(chunk, request);
        if (read == 0) {
            break;
        }
        bytes_read += read;
        for (std::size_t i = 0; i < read; ++i) {
            const char c = chunk[i];
            if (c == '\n' || c == '\r') {
                if (line_length_ == 0) {
                    continue;  // tolerate CRLF and blank lines
                }
                line_buffer_[line_length_] = '\0';
                PositionMeasurement parsed;
                if (parseLine(line_buffer_, line_length_, parsed)) {
                    ++parsed_lines_;
                    if (parsed.trolley_id[0] != '\0' &&
                        !boundedEquals(parsed.trolley_id, expected_trolley_id_)) {
                        ++dropped_lines_;  // sample addressed to another trolley
                    } else {
                        pushPending(parsed);
                    }
                } else {
                    ++malformed_lines_;
                }
                line_length_ = 0;
                continue;
            }
            if (line_length_ < kLineBufferSize - 1) {
                line_buffer_[line_length_++] = c;
            } else {
                ++dropped_lines_;
                line_length_ = 0;
            }
        }
        if (pending_count_ > 0) {
            break;  // a complete sample is available: hand it over now
        }
    }

    if (pending_count_ > 0) {
        out = pending_[pending_head_];
        pending_head_ = (pending_head_ + 1) % kPendingCapacity;
        --pending_count_;
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// MockPositionProvider
// ---------------------------------------------------------------------------

void MockPositionProvider::setMeasurement(const PositionMeasurement& measurement) {
    sample_ = measurement;
    has_sample_ = true;
    unread_ = true;  // a newly installed sample is always delivered at least once
}

void MockPositionProvider::setPosition(float x_m, float y_m, float quality, uint64_t timestamp_ms) {
    PositionMeasurement measurement;
    measurement.position.x_m = x_m;
    measurement.position.y_m = y_m;
    measurement.quality = quality;
    measurement.timestamp_ms = timestamp_ms;
    measurement.valid = true;
    setMeasurement(measurement);
}

bool MockPositionProvider::poll(PositionMeasurement& out, uint64_t now_ms) {
    ++poll_count_;
    if (!has_sample_) {
        return false;
    }
    if (!unread_ && !repeat_) {
        return false;  // delivered once and not configured to repeat
    }
    // A mock sample carries the local clock when the caller did not supply one,
    // which keeps the FSM's staleness check meaningful.
    if (sample_.timestamp_ms == 0) {
        sample_.timestamp_ms = now_ms;
    }
    out = sample_;
    unread_ = false;
    return true;
}

// ---------------------------------------------------------------------------
// ReplayPositionProvider
// ---------------------------------------------------------------------------

bool ReplayPositionProvider::load(const PositionMeasurement* samples, int count) {
    if (samples == nullptr || count <= 0 ||
        count > static_cast<int>(sizeof(samples_) / sizeof(samples_[0]))) {
        return false;
    }
    for (int i = 0; i < count; ++i) {
        samples_[i] = samples[i];
    }
    count_ = count;
    index_ = 0;
    return true;
}

bool ReplayPositionProvider::poll(PositionMeasurement& out, uint64_t now_ms) {
    (void)now_ms;
    if (index_ >= count_) {
        return false;
    }
    out = samples_[index_++];
    return true;
}

}  // namespace nav
