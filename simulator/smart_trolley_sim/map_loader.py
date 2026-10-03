"""Smart Market map and graph loading for the simulator.

Navigation never uses these structures for planning - the C++ core owns the
graph.  This module only provides rendering metadata (node coordinates, node
types, edge topology) and the map rectangle, and it validates that the JSON
matches the shared coordinate contract.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

from .config import REPO_ROOT

FRAME_ID = "smart_market_map"


class MapError(RuntimeError):
    """Raised when the map or graph document violates the shared contract."""


@dataclass(frozen=True)
class Node:
    node_id: int
    x_m: float
    y_m: float
    type: str = "intersection"
    #: Optional human-readable identifier, e.g. "produk-susu".  Lets a
    #: destination be given by name instead of a numeric node id.
    name: str = ""


@dataclass(frozen=True)
class Edge:
    from_id: int
    to_id: int
    weight_m: float
    bidirectional: bool = True


@dataclass
class MarketMap:
    """Map rectangle plus the navigation graph used for rendering."""

    map_id: str
    map_version: int
    frame_id: str
    width_m: float
    height_m: float
    origin_x_m: float = 0.0
    origin_y_m: float = 0.0
    nodes: dict[int, Node] = field(default_factory=dict)
    edges: list[Edge] = field(default_factory=list)
    graph_json: str = ""
    #: Lower-cased node name -> node id, for name-based destination selection.
    node_names: dict[str, int] = field(default_factory=dict)

    # -- convenience -------------------------------------------------------
    def node(self, node_id: int) -> Node | None:
        return self.nodes.get(int(node_id))

    def node_id_by_name(self, name: str) -> int | None:
        """Resolve a node name (case-insensitive) to its id."""
        return self.node_names.get(str(name).strip().lower())

    def node_label(self, node_id: int) -> str:
        """Human label for a node: its name when it has one, else "#id"."""
        node = self.nodes.get(int(node_id))
        if node is None:
            return f"#{node_id}"
        return node.name or f"#{node.node_id}"

    def node_position(self, node_id: int) -> tuple[float, float] | None:
        node = self.nodes.get(int(node_id))
        return None if node is None else (node.x_m, node.y_m)

    def destination_nodes(self) -> list[Node]:
        return [n for n in self.nodes.values() if n.type == "destination"]

    def edge_polyline(self, edge: Edge) -> tuple[tuple[float, float], tuple[float, float]] | None:
        a = self.node_position(edge.from_id)
        b = self.node_position(edge.to_id)
        if a is None or b is None:
            return None
        return a, b

    def bounds(self) -> tuple[float, float, float, float]:
        """Map rectangle as (min_x, min_y, max_x, max_y) in meters."""
        return (
            self.origin_x_m,
            self.origin_y_m,
            self.origin_x_m + self.width_m,
            self.origin_y_m + self.height_m,
        )

    def route_polyline(self, route: list[int]) -> list[tuple[float, float]]:
        points = []
        for node_id in route:
            position = self.node_position(node_id)
            if position is not None:
                points.append(position)
        return points


def _resolve(path: str | Path) -> Path:
    candidate = Path(path)
    return candidate if candidate.is_absolute() else REPO_ROOT / candidate


def load_market_map(map_path: str | Path, graph_path: str | Path) -> MarketMap:
    """Load map.json + graph.json and validate them against the contract."""
    map_file = _resolve(map_path)
    graph_file = _resolve(graph_path)
    if not map_file.is_file():
        raise MapError(f"map file not found: {map_file}")
    if not graph_file.is_file():
        raise MapError(f"graph file not found: {graph_file}")

    with map_file.open(encoding="utf-8") as handle:
        map_document = json.load(handle)
    with graph_file.open(encoding="utf-8") as handle:
        graph_text = handle.read()
    graph_document = json.loads(graph_text)

    frame_id = str(map_document.get("frame_id", ""))
    if frame_id != FRAME_ID:
        raise MapError(f'map.json frame_id must be "{FRAME_ID}", got "{frame_id}"')
    width = float(map_document.get("width_m", 0.0))
    height = float(map_document.get("height_m", 0.0))
    if width <= 0.0 or height <= 0.0:
        raise MapError("map.json width_m/height_m must be positive")

    market_map = MarketMap(
        map_id=str(map_document.get("map_id", "UNKNOWN")),
        map_version=int(map_document.get("map_version", 0)),
        frame_id=frame_id,
        width_m=width,
        height_m=height,
        origin_x_m=float(map_document.get("origin_x_m", 0.0)),
        origin_y_m=float(map_document.get("origin_y_m", 0.0)),
        graph_json=graph_text,
    )

    nodes_document = graph_document.get("nodes")
    if not isinstance(nodes_document, list) or not nodes_document:
        raise MapError("graph.json must contain a non-empty \"nodes\" array")
    for entry in nodes_document:
        node_id = int(entry["id"])
        if node_id in market_map.nodes:
            raise MapError(f"duplicate node id {node_id} in graph.json")
        node = Node(
            node_id=node_id,
            x_m=float(entry["x_m"]),
            y_m=float(entry["y_m"]),
            type=str(entry.get("type", "intersection")),
            name=str(entry.get("name", "")).strip(),
        )
        market_map.nodes[node_id] = node
        if node.name:
            key = node.name.lower()
            existing = market_map.node_names.get(key)
            if existing is not None:
                raise MapError(
                    f'graph.json node name "{node.name}" is used by both node '
                    f"{existing} and node {node_id}"
                )
            market_map.node_names[key] = node_id

    for entry in graph_document.get("edges", []):
        from_id = int(entry["from"])
        to_id = int(entry["to"])
        if from_id not in market_map.nodes or to_id not in market_map.nodes:
            raise MapError(f"edge {from_id}->{to_id} references an unknown node")
        a = market_map.nodes[from_id]
        b = market_map.nodes[to_id]
        weight = entry.get("weight_m")
        if weight is None:
            weight = ((b.x_m - a.x_m) ** 2 + (b.y_m - a.y_m) ** 2) ** 0.5
        weight = float(weight)
        if weight < 0.0:
            raise MapError(f"edge {from_id}->{to_id} has a negative weight")
        market_map.edges.append(
            Edge(
                from_id=from_id,
                to_id=to_id,
                weight_m=weight,
                bidirectional=bool(entry.get("bidirectional", True)),
            )
        )

    # Cross-check: a graph version mismatch must be reported, not silently used.
    graph_meta = graph_document.get("map", {})
    if isinstance(graph_meta, dict):
        graph_version = graph_meta.get("map_version")
        if graph_version is not None and int(graph_version) != market_map.map_version:
            raise MapError(
                f"map.json map_version={market_map.map_version} does not match "
                f"graph.json map.map_version={graph_version}"
            )
        graph_frame = graph_meta.get("frame_id")
        if graph_frame is not None and str(graph_frame) != FRAME_ID:
            raise MapError(f'graph.json map.frame_id must be "{FRAME_ID}"')

    return market_map


def describe_map(market_map: MarketMap) -> str:
    """One-line summary used by the CLI and the logs."""
    destinations = ", ".join(str(n.node_id) for n in market_map.destination_nodes())
    return (
        f"map {market_map.map_id} v{market_map.map_version} "
        f"({market_map.width_m:.1f} x {market_map.height_m:.1f} m), "
        f"{len(market_map.nodes)} nodes, {len(market_map.edges)} edges, "
        f"destination nodes: [{destinations}]"
    )
