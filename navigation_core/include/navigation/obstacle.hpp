// Static scene obstacles and graph filtering for collision-free planning.
#pragma once

#include <cstddef>

#include "navigation/graph.hpp"

namespace nav {

inline constexpr int kMaxSceneObstacles = 64;

/// A positioning-scene obstacle. x/y are its centre, sx/sy its full size and
/// rot its counter-clockwise rotation from +X, in radians.
struct Obstacle {
    float x_m{0.0f};
    float y_m{0.0f};
    float size_x_m{0.0f};
    float size_y_m{0.0f};
    float rotation_rad{0.0f};
};

struct SceneObstacles {
    float room_width_m{0.0f};
    float room_depth_m{0.0f};
    Obstacle items[kMaxSceneObstacles]{};
    int count{0};
};

/// Parse the exact response envelope returned by GET /api/v1/scene.
/// Requires scene.room.width/depth and x/y/sx/sy for each obstacle.
/// rot is optional (default 0).
bool loadSceneObstacles(const char* text, SceneObstacles& out, char* error,
                        std::size_t error_size);

/// Copy source into out while omitting graph edges that intersect any obstacle.
/// clearance_m inflates every obstacle equally on all sides for trolley safety.
/// Returns the number of omitted edges, or -1 when an input is invalid.
int filterGraphByObstacles(const Graph& source, const SceneObstacles& obstacles,
                           float clearance_m, Graph& out);

}  // namespace nav
