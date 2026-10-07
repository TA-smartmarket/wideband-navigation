#include "navigation/remote_position_provider.hpp"

#include <cstring>

namespace nav {

RemotePositionProvider::RemotePositionProvider(const char* expected_trolley_id) {
    copyBounded(expected_trolley_id_, sizeof(expected_trolley_id_), expected_trolley_id);
}

bool RemotePositionProvider::feed(const char* text, std::size_t length) {
    if (text == nullptr || length == 0) {
        ++malformed_;
        return false;
    }
    const JsonValue root = jsonParse(text, length);
    PositionMeasurement parsed;
    if (!measurementFromJson(root, parsed)) {
        ++malformed_;
        return false;
    }
    if (parsed.trolley_id[0] != '\0' && !boundedEquals(parsed.trolley_id, expected_trolley_id_)) {
        // Sample addressed to another trolley: drop silently (matching
        // SerialPositionProvider's behaviour), not counted as malformed.
        return false;
    }
    pending_ = parsed;
    has_pending_ = true;
    ++parsed_;
    return true;
}

bool RemotePositionProvider::feed(const char* text) {
    if (text == nullptr) {
        ++malformed_;
        return false;
    }
    return feed(text, std::strlen(text));
}

bool RemotePositionProvider::poll(PositionMeasurement& out, uint64_t now_ms) {
    (void)now_ms;
    if (!has_pending_) {
        return false;
    }
    out = pending_;
    has_pending_ = false;
    return true;
}

}  // namespace nav
