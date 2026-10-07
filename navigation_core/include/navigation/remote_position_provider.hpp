// A position provider for transports that deliver whole JSON documents.
//
// `SerialPositionProvider` consumes a newline-delimited byte stream (UART).
// HTTP and MQTT deliver one complete JSON document per response/payload, with
// no trailing newline, so they need a slightly different receiver: the
// transport glue calls `feed()` with one complete document and `poll()` then
// hands it back to the navigation loop.  Parsing still goes through the single
// shared `measurementFromJson` parser, so the wire contract is identical
// across every transport.
#pragma once

#include <cstddef>
#include <cstdint>

#include "navigation/position_provider.hpp"

namespace nav {

class RemotePositionProvider : public IPositionProvider {
public:
    explicit RemotePositionProvider(const char* expected_trolley_id = kDefaultTrolleyId);

    /// Parse one complete JSON document (HTTP response body / MQTT payload).
    /// Returns false when the document is malformed or addressed to another
    /// trolley.  The parsed sample is queued for the next `poll()`.
    bool feed(const char* text, std::size_t length);

    /// Feed a NUL-terminated document (convenience wrapper).
    bool feed(const char* text);

    bool poll(PositionMeasurement& out, uint64_t now_ms) override;
    const char* name() const override { return "remote-json"; }

    uint32_t parsedCount() const { return parsed_; }
    uint32_t malformedCount() const { return malformed_; }

private:
    char expected_trolley_id_[kIdFieldSize]{};
    PositionMeasurement pending_{};
    bool has_pending_{false};
    uint32_t parsed_{0};
    uint32_t malformed_{0};
};

}  // namespace nav
