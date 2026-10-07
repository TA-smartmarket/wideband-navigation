#include "navigation/obstacle.hpp"

#include <cmath>
#include <cstdio>

#include "navigation/json_parser.hpp"

namespace nav {
namespace {

void setError(char* error, std::size_t size, const char* message, int index = -1) {
    if (error == nullptr || size == 0) return;
    if (index >= 0) {
        std::snprintf(error, size, "scene.obstacles[%d]: %s", index, message);
    } else {
        std::snprintf(error, size, "%s", message);
    }
}

bool readNumber(const JsonValue& object, const char* key, float& out) {
    const JsonValue value = object.member(key);
    if (!value.isNumber()) return false;
    out = value.asFloat();
    return isFinite(out);
}

bool segmentIntersects(const Position2D& a, const Position2D& b,
                       const Obstacle& obstacle, float clearance_m) {
    // Transform the segment into the obstacle's local coordinates, then clip
    // it against the inflated axis-aligned rectangle (slab method).
    const float c = std::cos(obstacle.rotation_rad);
    const float s = std::sin(obstacle.rotation_rad);
    const auto local = [&](const Position2D& point) {
        const float dx = point.x_m - obstacle.x_m;
        const float dy = point.y_m - obstacle.y_m;
        return Position2D{c * dx + s * dy, -s * dx + c * dy};
    };
    const Position2D p0 = local(a);
    const Position2D p1 = local(b);
    const float half_x = obstacle.size_x_m * 0.5f + clearance_m;
    const float half_y = obstacle.size_y_m * 0.5f + clearance_m;
    const float delta[2] = {p1.x_m - p0.x_m, p1.y_m - p0.y_m};
    const float start[2] = {p0.x_m, p0.y_m};
    const float half[2] = {half_x, half_y};
    float enter = 0.0f;
    float exit = 1.0f;
    for (int axis = 0; axis < 2; ++axis) {
        if (std::fabs(delta[axis]) < 1.0e-7f) {
            if (start[axis] < -half[axis] || start[axis] > half[axis]) return false;
            continue;
        }
        float t0 = (-half[axis] - start[axis]) / delta[axis];
        float t1 = (half[axis] - start[axis]) / delta[axis];
        if (t0 > t1) {
            const float swap = t0;
            t0 = t1;
            t1 = swap;
        }
        if (t0 > enter) enter = t0;
        if (t1 < exit) exit = t1;
        if (enter > exit) return false;
    }
    return true;
}

}  // namespace

bool loadSceneObstacles(const char* text, SceneObstacles& out, char* error,
                        std::size_t error_size) {
    out = SceneObstacles{};
    const JsonValue root = jsonParse(text);
    if (!root.isObject()) {
        setError(error, error_size, "scene: invalid JSON object");
        return false;
    }
    const JsonValue scene = root.member("scene");
    if (!scene.isObject()) {
        setError(error, error_size, "scene: missing scene object");
        return false;
    }
    const JsonValue room = scene.member("room");
    if (!room.isObject() || !readNumber(room, "width", out.room_width_m) ||
        !readNumber(room, "depth", out.room_depth_m) || out.room_width_m <= 0.0f ||
        out.room_depth_m <= 0.0f) {
        setError(error, error_size, "scene.room width/depth must be positive finite numbers");
        return false;
    }
    const JsonValue entries = scene.member("obstacles");
    if (!entries.isArray()) {
        setError(error, error_size, "scene: obstacles must be an array");
        return false;
    }
    if (entries.size() > kMaxSceneObstacles) {
        setError(error, error_size, "scene: too many obstacles");
        return false;
    }
    for (int i = 0; i < entries.size(); ++i) {
        const JsonValue entry = entries.element(i);
        if (!entry.isObject()) {
            setError(error, error_size, "entry must be an object", i);
            return false;
        }
        Obstacle obstacle;
        if (!readNumber(entry, "x", obstacle.x_m) ||
            !readNumber(entry, "y", obstacle.y_m) ||
            !readNumber(entry, "sx", obstacle.size_x_m) ||
            !readNumber(entry, "sy", obstacle.size_y_m)) {
            setError(error, error_size, "x, y, sx and sy must be finite numbers", i);
            return false;
        }
        if (obstacle.size_x_m <= 0.0f || obstacle.size_y_m <= 0.0f) {
            setError(error, error_size, "sx and sy must be greater than zero", i);
            return false;
        }
        const JsonValue rotation = entry.member("rot");
        if (rotation.valid() &&
            (!rotation.isNumber() || !readNumber(entry, "rot", obstacle.rotation_rad))) {
            setError(error, error_size, "rot must be a finite number", i);
            return false;
        }
        out.items[out.count++] = obstacle;
    }
    return true;
}

int filterGraphByObstacles(const Graph& source, const SceneObstacles& obstacles,
                           float clearance_m, Graph& out) {
    if (!source.validate().valid || obstacles.count < 0 ||
        obstacles.count > kMaxSceneObstacles || !isFinite(clearance_m) || clearance_m < 0.0f) {
        out.clear();
        return -1;
    }
    out.clear();
    for (int i = 0; i < source.nodeCount(); ++i) {
        const GraphNode& node = source.nodeAt(i);
        if (out.addNode(node.id, node.x_m, node.y_m, node.type) < 0) return -1;
    }
    int blocked = 0;
    for (int i = 0; i < source.edgeCount(); ++i) {
        const GraphEdge& edge = source.edgeAt(i);
        const GraphNode* from = source.findNode(edge.from);
        const GraphNode* to = source.findNode(edge.to);
        bool intersects = false;
        for (int j = 0; j < obstacles.count && !intersects; ++j) {
            intersects = segmentIntersects(Position2D{from->x_m, from->y_m},
                                            Position2D{to->x_m, to->y_m},
                                            obstacles.items[j], clearance_m);
        }
        if (intersects) {
            ++blocked;
        } else if (out.addEdge(edge.from, edge.to, edge.weight_m, edge.bidirectional) < 0) {
            out.clear();
            return -1;
        }
    }
    return blocked;
}

}  // namespace nav
