"""Pure, dependency-free models used by the dashboard API.

The dashboard deliberately keeps planning and map validation outside the web
handlers.  That makes the important safety checks testable without a browser or
an ESP32 attached.
"""
from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
from typing import Any


class ValidationError(ValueError):
    """A configuration or command violates a dashboard safety constraint."""


def finite_positive(value: Any, name: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValidationError(f"{name} must be a finite number") from exc
    if not math.isfinite(number) or number <= 0:
        raise ValidationError(f"{name} must be greater than zero")
    return number


def validate_map(document: dict[str, Any]) -> dict[str, Any]:
    if document.get("frame_id") != "smart_market_map":
        raise ValidationError("map frame_id must be smart_market_map")
    document["width_m"] = finite_positive(document.get("width_m"), "width_m")
    document["height_m"] = finite_positive(document.get("height_m"), "height_m")
    document["origin"] = "bottom-left"
    return document


def _obstacle_bounds(obstacle: dict[str, Any]) -> tuple[float, float, float, float]:
    # The established scene schema stores a centre point and sx/sy dimensions.
    x = float(obstacle["x"])
    y = float(obstacle["y"])
    width = finite_positive(obstacle.get("sx"), "obstacle width")
    height = finite_positive(obstacle.get("sy"), "obstacle height")
    if not all(math.isfinite(v) for v in (x, y)):
        raise ValidationError("obstacle position must be finite")
    return x - width / 2, y - height / 2, x + width / 2, y + height / 2


def validate_scene(document: dict[str, Any], map_document: dict[str, Any]) -> dict[str, Any]:
    room = document.setdefault("scene", {}).setdefault("room", {})
    room["width"] = finite_positive(map_document["width_m"], "width_m")
    room["depth"] = finite_positive(map_document["height_m"], "height_m")
    obstacles = document["scene"].setdefault("obstacles", [])
    identifiers: set[str] = set()
    for obstacle in obstacles:
        identifier = str(obstacle.get("id", "")).strip()
        if not identifier or identifier in identifiers:
            raise ValidationError("each obstacle needs a unique id")
        identifiers.add(identifier)
        left, bottom, right, top = _obstacle_bounds(obstacle)
        if left < 0 or bottom < 0 or right > map_document["width_m"] or top > map_document["height_m"]:
            raise ValidationError(f"obstacle {identifier} must remain inside map bounds")
    return document


def segment_hits_rect(a: tuple[float, float], b: tuple[float, float], rect: tuple[float, float, float, float]) -> bool:
    """Return whether a segment intersects or enters an axis-aligned rectangle."""
    left, bottom, right, top = rect
    dx, dy = b[0] - a[0], b[1] - a[1]
    t0, t1 = 0.0, 1.0
    for p, q in ((-dx, a[0] - left), (dx, right - a[0]), (-dy, a[1] - bottom), (dy, top - a[1])):
        if abs(p) < 1e-12:
            if q < 0:
                return False
            continue
        t = q / p
        if p < 0:
            if t > t1:
                return False
            t0 = max(t0, t)
        else:
            if t < t0:
                return False
            t1 = min(t1, t)
    return True


@dataclass(frozen=True)
class Route:
    nodes: list[int]
    distance_m: float


def plan_route(graph: dict[str, Any], scene: dict[str, Any], start: int, destination: int) -> Route:
    """Dijkstra route on the existing graph, excluding edges blocked by racks.

    The firmware and simulator continue to use navigation_core for autonomous
    following.  This function is the dashboard preview and deliberately follows
    the same Euclidean weight convention as config/graph.json.
    """
    positions = {int(n["id"]): (float(n["x_m"]), float(n["y_m"])) for n in graph.get("nodes", [])}
    if start not in positions or destination not in positions:
        raise ValidationError("start and destination must be graph node ids")
    blocked = [_obstacle_bounds(item) for item in scene.get("scene", {}).get("obstacles", [])]
    adjacency: dict[int, list[tuple[int, float]]] = {node: [] for node in positions}
    for edge in graph.get("edges", []):
        source, target = int(edge["from"]), int(edge["to"])
        if source not in positions or target not in positions:
            continue
        if any(segment_hits_rect(positions[source], positions[target], rect) for rect in blocked):
            continue
        weight = float(edge.get("weight_m", math.dist(positions[source], positions[target])))
        adjacency[source].append((target, weight))
        if edge.get("bidirectional", True):
            adjacency[target].append((source, weight))
    queue: list[tuple[float, int]] = [(0.0, start)]
    distance = {start: 0.0}
    predecessor: dict[int, int] = {}
    while queue:
        current_distance, node = heapq.heappop(queue)
        if current_distance != distance.get(node):
            continue
        if node == destination:
            break
        for neighbor, weight in adjacency[node]:
            candidate = current_distance + weight
            if candidate < distance.get(neighbor, math.inf):
                distance[neighbor] = candidate
                predecessor[neighbor] = node
                heapq.heappush(queue, (candidate, neighbor))
    if destination not in distance:
        raise ValidationError("destination is unreachable with current obstacles")
    result = [destination]
    while result[-1] != start:
        result.append(predecessor[result[-1]])
    result.reverse()
    return Route(result, distance[destination])


def validate_motion(linear_mps: Any, angular_radps: Any, maximum_linear: float, maximum_angular: float) -> tuple[float, float]:
    linear = float(linear_mps)
    angular = float(angular_radps)
    if not math.isfinite(linear) or not math.isfinite(angular):
        raise ValidationError("motion command must be finite")
    if abs(linear) > maximum_linear or abs(angular) > maximum_angular:
        raise ValidationError("motion command exceeds configured safety limit")
    return linear, angular
