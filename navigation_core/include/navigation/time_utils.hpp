// Monotonic time helpers shared by the simulator and the firmware.
//
// The control loop measures dt with a *local* monotonic clock; timestamps that
// arrive over the network/serial are only used to detect stale samples and are
// never used to compute dt (remote clocks are not trustworthy).
#pragma once

#include <chrono>
#include <cstdint>

namespace nav {

/// Monotonic milliseconds since an arbitrary epoch.
inline uint64_t monotonicMillis() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
}

/// Monotonic nanoseconds, used to time the Dijkstra execution.
///
/// Nanosecond resolution is required because a 12-node plan takes well under a
/// microsecond on a desktop CPU: microsecond-resolution timing would report 0.
inline uint64_t monotonicNanos() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

/// Round a nanosecond duration up to whole microseconds (a non-zero duration
/// must never be reported as 0 us).
inline uint32_t nanosToMicrosRoundedUp(uint64_t nanos) {
    const uint64_t micros = (nanos + 999ull) / 1000ull;
    return micros > 0xFFFFFFFFull ? 0xFFFFFFFFu : static_cast<uint32_t>(micros);
}

}  // namespace nav
