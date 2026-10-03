// JSON configuration loaders.
//
// The core never touches the filesystem: the caller (firmware or simulator)
// provides the text and receives the parsed structures plus a human readable
// error message.  This keeps the ESP32 build free of file I/O and makes the
// loaders directly unit-testable.
#pragma once

#include <cstddef>

#include "navigation/graph.hpp"
#include "navigation/navigation_config.hpp"
#include "navigation/types.hpp"

namespace nav {

/// Map metadata (config/map.json) - versioning prevents a mismatched map.
struct MapMetadata {
    char map_id[32]{};
    int map_version{0};
    char frame_id[kFrameFieldSize]{};
    float width_m{0.0f};
    float height_m{0.0f};
    float origin_x_m{0.0f};
    float origin_y_m{0.0f};
};

/// Parse config/map.json.
bool loadMapMetadata(const char* text, MapMetadata& out, char* error, std::size_t error_size);

/// Parse config/graph.json.  The graph is cleared first; on failure it is left
/// empty.  Validates structure (duplicate ids, unknown endpoints, self loops,
/// negative weights, empty graph) and reports the first problem.
bool loadGraph(const char* text, Graph& out, char* error, std::size_t error_size);

/// Parse config/navigation.json and validate every parameter.
bool loadNavigationConfig(const char* text, NavigationConfig& out, char* error,
                          std::size_t error_size);

}  // namespace nav
