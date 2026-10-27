"""Pure, dependency-free models used by the dashboard API.

The dashboard deliberately keeps planning and map validation outside the web
handlers.  That makes the important safety checks testable without a browser or
an ESP32 attached.
"""
from __future__ import annotations

from dataclasses import dataclass
import heapq
import itertools
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


def finite_number(value: Any, name: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValidationError(f"{name} must be a finite number") from exc
    if not math.isfinite(number):
        raise ValidationError(f"{name} must be a finite number")
    return number


def validate_map(document: dict[str, Any]) -> dict[str, Any]:
    if document.get("frame_id") != "smart_market_map":
        raise ValidationError("map frame_id must be smart_market_map")
    document["width_m"] = finite_positive(document.get("width_m"), "width_m")
    document["height_m"] = finite_positive(document.get("height_m"), "height_m")
    document["origin"] = "bottom-left"
    return document


def _obstacle_bounds(obstacle: dict[str, Any]) -> tuple[float, float, float, float]:
    """Axis-aligned bounds of a possibly rotated scene obstacle."""
    x = finite_number(obstacle["x"], "obstacle x")
    y = finite_number(obstacle["y"], "obstacle y")
    width = finite_positive(obstacle.get("sx"), "obstacle width")
    depth = finite_positive(obstacle.get("sy"), "obstacle depth")
    rotation = finite_number(obstacle.get("rot", 0.0), "obstacle rotation")
    cosine, sine = abs(math.cos(rotation)), abs(math.sin(rotation))
    extent_x = (width * cosine + depth * sine) / 2
    extent_y = (width * sine + depth * cosine) / 2
    return x - extent_x, y - extent_y, x + extent_x, y + extent_y


def validate_scene(document: dict[str, Any], map_document: dict[str, Any], graph: dict[str, Any] | None = None) -> dict[str, Any]:
    room = document.setdefault("scene", {}).setdefault("room", {})
    room["width"] = finite_positive(map_document["width_m"], "width_m")
    room["depth"] = finite_positive(map_document["height_m"], "height_m")
    room["height"] = finite_positive(room.get("height", 2.7), "room height")
    obstacles = document["scene"].setdefault("obstacles", [])
    identifiers: set[str] = set()
    product_identifiers: set[str] = set()
    graph_nodes = None if graph is None else {int(node["id"]) for node in graph.get("nodes", [])}
    for obstacle in obstacles:
        identifier = str(obstacle.get("id", "")).strip()
        if not identifier or identifier in identifiers:
            raise ValidationError("each obstacle needs a unique id")
        identifiers.add(identifier)
        obstacle["x"] = finite_number(obstacle.get("x"), "obstacle x")
        obstacle["y"] = finite_number(obstacle.get("y"), "obstacle y")
        obstacle["sx"] = finite_positive(obstacle.get("sx"), "obstacle width")
        obstacle["sy"] = finite_positive(obstacle.get("sy"), "obstacle depth")
        obstacle["sz"] = finite_positive(obstacle.get("sz", 1.5), "obstacle height")
        # Rack elevation is derived, never user-controlled: every rack rests on
        # the floor and changing its height keeps its lower face at Z=0.
        obstacle["z"] = obstacle["sz"] / 2
        obstacle["rot"] = finite_number(obstacle.get("rot", 0.0), "obstacle rotation")
        obstacle["atten"] = finite_number(obstacle.get("atten", 1.0), "obstacle attenuation")
        if not 0.0 <= obstacle["atten"] <= 1.0:
            raise ValidationError("obstacle attenuation must be between 0 and 1")
        if obstacle["sz"] > room["height"]:
            raise ValidationError(f"obstacle {identifier} must remain inside room height")
        left, bottom, right, top = _obstacle_bounds(obstacle)
        if left < 0 or bottom < 0 or right > map_document["width_m"] or top > map_document["height_m"]:
            raise ValidationError(f"obstacle {identifier} must remain inside map bounds")
        for product in obstacle.get("products", []):
            product_id = str(product.get("id", "")).strip()
            name = str(product.get("name", "")).strip()
            try:
                node_id = int(product.get("node_id"))
            except (TypeError, ValueError) as exc:
                raise ValidationError(f"product {product_id or '?'} needs a valid node_id") from exc
            if not product_id or product_id in product_identifiers or not name:
                raise ValidationError("each product needs a unique id and non-empty name")
            if node_id <= 0 or (graph_nodes is not None and node_id not in graph_nodes):
                raise ValidationError(f"product {product_id} references an unknown graph node")
            product_identifiers.add(product_id)
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


def segment_hits_obstacle(a: tuple[float, float], b: tuple[float, float], obstacle: dict[str, Any]) -> bool:
    """Test a map segment against an oriented rectangle obstacle."""
    cx, cy = float(obstacle["x"]), float(obstacle["y"])
    rotation = -float(obstacle.get("rot", 0.0))
    cosine, sine = math.cos(rotation), math.sin(rotation)

    def local(point: tuple[float, float]) -> tuple[float, float]:
        dx, dy = point[0] - cx, point[1] - cy
        return dx * cosine - dy * sine, dx * sine + dy * cosine

    half_width = float(obstacle["sx"]) / 2
    half_depth = float(obstacle["sy"]) / 2
    return segment_hits_rect(local(a), local(b), (-half_width, -half_depth, half_width, half_depth))


def point_hits_obstacle(point: tuple[float, float], obstacle: dict[str, Any]) -> bool:
    """Return whether a point is inside a rotated obstacle footprint."""
    cx, cy = float(obstacle["x"]), float(obstacle["y"])
    rotation = -float(obstacle.get("rot", 0.0))
    cosine, sine = math.cos(rotation), math.sin(rotation)
    dx, dy = point[0] - cx, point[1] - cy
    local_x = dx * cosine - dy * sine
    local_y = dx * sine + dy * cosine
    return abs(local_x) <= float(obstacle["sx"]) / 2 and abs(local_y) <= float(obstacle["sy"]) / 2


def fit_nodes_to_room(nodes: list[dict[str, Any]], map_document: dict[str, Any],
                      scene: dict[str, Any]) -> list[dict[str, Any]]:
    """Lay nodes out as a regular grid that fits the room and avoids racks.

    Whole rows move together when their ideal Y coordinate intersects an
    obstacle. This preserves straight horizontal and vertical graph lines.
    """
    if not nodes:
        return []
    width = finite_positive(map_document.get("width_m"), "width_m")
    depth = finite_positive(map_document.get("height_m"), "height_m")
    obstacles = scene.get("scene", {}).get("obstacles", [])
    ordered = sorted(nodes, key=lambda node: int(node["id"]))
    columns = math.ceil(math.sqrt(len(ordered)))
    rows = math.ceil(len(ordered) / columns)
    margin_x = min(1.5, width * .1)
    margin_y = min(1.5, depth * .2)
    xs = ([width / 2] if columns == 1 else
          [margin_x + column * (width - 2 * margin_x) / (columns - 1)
           for column in range(columns)])
    ideal_ys = ([depth / 2] if rows == 1 else
                [margin_y + row * (depth - 2 * margin_y) / (rows - 1)
                 for row in range(rows)])
    step = max(.05, min(width, depth) / 80)
    row_spacing = max(.22, min(width, depth) * .025)
    fitted_ys: list[float] = []

    def row_is_free(y: float) -> bool:
        return (0 <= y <= depth
                and all(abs(y - previous) >= row_spacing for previous in fitted_ys)
                and all(not point_hits_obstacle((x, y), obstacle)
                        for x in xs for obstacle in obstacles))

    for ideal_y in ideal_ys:
        fitted_y = None
        maximum_offset = max(ideal_y, depth - ideal_y)
        offset = 0.0
        while offset <= maximum_offset + step and fitted_y is None:
            candidates = [ideal_y] if offset == 0 else [ideal_y + offset, ideal_y - offset]
            fitted_y = next((candidate for candidate in candidates if row_is_free(candidate)), None)
            offset += step
        if fitted_y is None:
            raise ValidationError("room has insufficient free space for a tidy node grid")
        fitted_ys.append(fitted_y)

    positions: dict[int, tuple[float, float]] = {}
    for index, node in enumerate(ordered):
        positions[int(node["id"])] = (xs[index % columns], fitted_ys[index // columns])
    return [{**node, "x_m": positions[int(node["id"])][0], "y_m": positions[int(node["id"])][1]}
            for node in nodes]


def validate_graph(document: dict[str, Any], map_document: dict[str, Any], scene: dict[str, Any] | None = None) -> dict[str, Any]:
    """Validate editable graph nodes and their explicit connections."""
    graph_map = document.setdefault("map", {})
    graph_map.update({
        "map_id": map_document.get("map_id", graph_map.get("map_id", "SMART_MARKET_MAIN")),
        "map_version": map_document.get("map_version", graph_map.get("map_version", 1)),
        "frame_id": "smart_market_map",
        "width_m": float(map_document["width_m"]),
        "height_m": float(map_document["height_m"]),
    })
    nodes = document.setdefault("nodes", [])
    identifiers: set[int] = set()
    names: set[str] = set()
    obstacles = [] if scene is None else scene.get("scene", {}).get("obstacles", [])
    for node in nodes:
        try:
            identifier = int(node.get("id"))
        except (TypeError, ValueError) as exc:
            raise ValidationError("each graph node needs a positive integer id") from exc
        if identifier <= 0 or identifier in identifiers:
            raise ValidationError("each graph node needs a unique positive integer id")
        name = str(node.get("name", "")).strip()
        if not name or name.casefold() in names:
            raise ValidationError("each graph node needs a unique non-empty name")
        x = finite_number(node.get("x_m"), f"node {identifier} x_m")
        y = finite_number(node.get("y_m"), f"node {identifier} y_m")
        if x < 0 or y < 0 or x > map_document["width_m"] or y > map_document["height_m"]:
            raise ValidationError(f"graph node {identifier} must remain inside map bounds")
        if any(point_hits_obstacle((x, y), obstacle) for obstacle in obstacles):
            raise ValidationError(f"graph node {identifier} cannot be inside an obstacle")
        node.update({"id": identifier, "name": name, "type": str(node.get("type", "waypoint")).strip() or "waypoint", "x_m": x, "y_m": y})
        identifiers.add(identifier)
        names.add(name.casefold())

    edge_keys: set[tuple[int, int]] = set()
    for edge in document.setdefault("edges", []):
        try:
            source, target = int(edge.get("from")), int(edge.get("to"))
        except (TypeError, ValueError) as exc:
            raise ValidationError("graph edges need valid from/to node ids") from exc
        if source == target or source not in identifiers or target not in identifiers:
            raise ValidationError("graph edges must connect two different existing nodes")
        key = (min(source, target), max(source, target))
        if key in edge_keys:
            raise ValidationError("duplicate graph edge")
        edge_keys.add(key)
        edge.update({"from": source, "to": target, "bidirectional": True})
        edge.pop("weight_m", None)
    return document


def build_visibility_edges(nodes: list[dict[str, Any]], scene: dict[str, Any]) -> list[dict[str, Any]]:
    """Build a stable grid graph with visible fallback links.

    Moving one node must not silently replace all of its logical neighbours.
    Grid adjacency therefore follows node order, independent of coordinates.
    """
    obstacles = scene.get("scene", {}).get("obstacles", [])
    positions = {int(node["id"]): (float(node["x_m"]), float(node["y_m"])) for node in nodes}
    candidates: list[tuple[float, int, int]] = []
    for first, second in itertools.combinations(nodes, 2):
        a = (float(first["x_m"]), float(first["y_m"]))
        b = (float(second["x_m"]), float(second["y_m"]))
        if not any(segment_hits_obstacle(a, b, obstacle) for obstacle in obstacles):
            source, target = sorted((int(first["id"]), int(second["id"])))
            candidates.append((math.dist(a, b), source, target))
    candidates.sort()

    visible_pairs = {(source, target) for _, source, target in candidates}
    ordered = sorted(positions)
    columns = math.ceil(math.sqrt(len(ordered))) if ordered else 0
    selected: set[tuple[int, int]] = set()
    for index, identifier in enumerate(ordered):
        if index % columns < columns - 1 and index + 1 < len(ordered):
            pair = tuple(sorted((identifier, ordered[index + 1])))
            if pair in visible_pairs:
                selected.add(pair)
        if index + columns < len(ordered):
            pair = tuple(sorted((identifier, ordered[index + columns])))
            if pair in visible_pairs:
                selected.add(pair)

    parent = {int(node["id"]): int(node["id"]) for node in nodes}

    def find(identifier: int) -> int:
        while parent[identifier] != identifier:
            parent[identifier] = parent[parent[identifier]]
            identifier = parent[identifier]
        return identifier

    def union(source: int, target: int) -> None:
        source_root, target_root = find(source), find(target)
        if source_root != target_root:
            parent[source_root] = target_root

    for source, target in selected:
        union(source, target)
    # If an obstacle blocks a logical grid edge, add the shortest visible link
    # needed to keep the graph connected. Unnecessary diagonals are omitted.
    for _, source, target in candidates:
        source_root, target_root = find(source), find(target)
        if source_root != target_root:
            union(source, target)
            selected.add((source, target))
    return [{"from": source, "to": target, "bidirectional": True} for source, target in sorted(selected)]


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
    blocked = scene.get("scene", {}).get("obstacles", [])
    adjacency: dict[int, list[tuple[int, float]]] = {node: [] for node in positions}
    for edge in graph.get("edges", []):
        source, target = int(edge["from"]), int(edge["to"])
        if source not in positions or target not in positions:
            continue
        if any(segment_hits_obstacle(positions[source], positions[target], obstacle) for obstacle in blocked):
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
