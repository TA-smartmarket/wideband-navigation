// Telemetry serialisation.  Kept completely separate from control logic: the
// FSM produces a `NavigationStatus` snapshot and this module turns it into a
// single-line JSON document (and a human readable status block).
#pragma once

#include <cstddef>
#include <cstdint>

#include "navigation/types.hpp"

namespace nav {

/// Serialise the navigation status to the documented telemetry contract.
/// Returns the number of characters written (excluding the NUL terminator), or
/// 0 when the buffer is too small.
int statusToJson(const NavigationStatus& status,
                 const char* trolley_id,
                 uint64_t timestamp_ms,
                 char* out,
                 std::size_t out_size);

/// Multi-line, human readable status (the `status` debug command).
int statusToText(const NavigationStatus& status, const char* trolley_id, char* out,
                 std::size_t out_size);

}  // namespace nav
