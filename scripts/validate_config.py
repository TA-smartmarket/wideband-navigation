#!/usr/bin/env python3
"""Validate the JSON configuration files against the shared contract.

Checks everything the C++ loaders enforce, plus cross-file consistency
(map.json <-> graph.json version and frame) and the physical sanity of the
navigation parameters.  Intended for CI and for a pre-flash sanity check.

Usage
-----
    python scripts/validate_config.py
    python scripts/validate_config.py --config-dir config
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
FRAME_ID = "smart_market_map"

NODE_TYPES = {
    "intersection", "aisle", "destination", "parking", "entry", "exit", "checkout",
}


class ValidationError(Exception):
    """A configuration problem, reported with the offending field."""


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ValidationError(message)


def _number(document: dict, key: str, where: str) -> float:
    value = document.get(key)
    _require(isinstance(value, (int, float)) and not isinstance(value, bool),
             f"{where}: \"{key}\" must be a number")
    return float(value)


def validate_map(document: dict) -> dict:
    _require(isinstance(document, dict), "map.json: root must be an object")
    _require(isinstance(document.get("map_id"), str) and document["map_id"],
             "map.json: map_id is required")
    version = document.get("map_version")
    _require(isinstance(version, int) and version >= 1,
             "map.json: map_version must be an integer >= 1")
    _require(document.get("frame_id") == FRAME_ID,
             f'map.json: frame_id must be "{FRAME_ID}"')
    width = _number(document, "width_m", "map.json")
    height = _number(document, "height_m", "map.json")
    _require(width > 0.0, "map.json: width_m must be > 0")
    _require(height > 0.0, "map.json: height_m must be > 0")
    return {"width_m": width, "height_m": height, "map_version": version,
            "map_id": document["map_id"]}


def validate_graph(document: dict, map_info: dict) -> dict:
    _require(isinstance(document, dict), "graph.json: root must be an object")

    meta = document.get("map")
    if isinstance(meta, dict):
        if "map_version" in meta:
            _require(int(meta["map_version"]) == map_info["map_version"],
                     f'graph.json: map.map_version ({meta["map_version"]}) does not match '
                     f'map.json ({map_info["map_version"]})')
        if "frame_id" in meta:
            _require(meta["frame_id"] == FRAME_ID,
                     f'graph.json: map.frame_id must be "{FRAME_ID}"')

    nodes = document.get("nodes")
    _require(isinstance(nodes, list) and nodes, 'graph.json: "nodes" must be a non-empty array')

    ids: set[int] = set()
    names: dict[str, int] = {}
    for entry in nodes:
        _require(isinstance(entry, dict), "graph.json: every node must be an object")
        node_id = entry.get("id")
        _require(isinstance(node_id, int) and node_id >= 0,
                 "graph.json: node id must be an integer >= 0")
        _require(node_id not in ids, f"graph.json: duplicate node id {node_id}")
        ids.add(node_id)
        x = _number(entry, "x_m", f"graph.json node {node_id}")
        y = _number(entry, "y_m", f"graph.json node {node_id}")
        node_type = entry.get("type", "intersection")
        _require(node_type in NODE_TYPES,
                 f'graph.json node {node_id}: unknown type "{node_type}"')
        margin = 1.0
        _require(-margin <= x <= map_info["width_m"] + margin,
                 f"graph.json node {node_id}: x_m={x} is outside the map")
        _require(-margin <= y <= map_info["height_m"] + margin,
                 f"graph.json node {node_id}: y_m={y} is outside the map")
        # Optional human handle used for --destination <name>.  It must be a
        # unique, non-empty token so name lookup is unambiguous.
        name = entry.get("name")
        if name is not None:
            _require(isinstance(name, str) and name.strip(),
                     f"graph.json node {node_id}: \"name\" must be a non-empty string")
            key = name.strip().lower()
            # Build the message lazily: it indexes names[key], which does not
            # exist yet on the happy path.
            _require(key not in names,
                     f'graph.json: node name "{name}" is used by both node '
                     f"{names.get(key)} and node {node_id}")
            names[key] = node_id

    coordinates = {
        entry["id"]: (float(entry["x_m"]), float(entry["y_m"])) for entry in nodes
    }

    edges = document.get("edges", [])
    _require(isinstance(edges, list), 'graph.json: "edges" must be an array')
    seen: set[tuple[int, int]] = set()
    for entry in edges:
        _require(isinstance(entry, dict), "graph.json: every edge must be an object")
        from_id = entry.get("from")
        to_id = entry.get("to")
        _require(from_id in coordinates, f"graph.json: edge from unknown node {from_id}")
        _require(to_id in coordinates, f"graph.json: edge to unknown node {to_id}")
        _require(from_id != to_id, f"graph.json: self loop at node {from_id}")
        key = (from_id, to_id)
        _require(key not in seen, f"graph.json: duplicate edge {from_id}->{to_id}")
        seen.add(key)
        weight = entry.get("weight_m")
        if weight is not None:
            _require(isinstance(weight, (int, float)) and not isinstance(weight, bool),
                     f"graph.json: edge {from_id}->{to_id} weight_m must be a number")
            _require(float(weight) >= 0.0,
                     f"graph.json: edge {from_id}->{to_id} weight_m must be >= 0")
        # Warn (not fail) when an explicit weight differs a lot from the metric
        # distance: usually a typo, but a deliberate detour is legal.
        a = coordinates[from_id]
        b = coordinates[to_id]
        euclidean = ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) ** 0.5
        if weight is not None and euclidean > 1e-6:
            ratio = float(weight) / euclidean
            if ratio < 0.5 or ratio > 2.0:
                print(f"  warning: edge {from_id}->{to_id} weight {float(weight):.2f} m is "
                      f"{ratio:.2f}x the Euclidean distance ({euclidean:.2f} m)")

    # Connectivity: every node must be reachable from node 1.
    adjacency: dict[int, list[int]] = {node_id: [] for node_id in coordinates}
    for entry in edges:
        adjacency[entry["from"]].append(entry["to"])
        if entry.get("bidirectional", True):
            adjacency[entry["to"]].append(entry["from"])
    start = nodes[0]["id"]
    reached = {start}
    stack = [start]
    while stack:
        current = stack.pop()
        for neighbour in adjacency[current]:
            if neighbour not in reached:
                reached.add(neighbour)
                stack.append(neighbour)
    unreachable = sorted(ids - reached)
    if unreachable:
        print(f"  warning: nodes not reachable from node {start}: {unreachable} "
              f"(intentional for the unreachable-destination scenario)")

    destinations = [e["id"] for e in nodes if e.get("type") == "destination"]
    return {"node_count": len(nodes), "edge_count": len(edges),
            "destination_nodes": destinations}


def validate_navigation(document: dict, map_info: dict) -> dict:
    _require(isinstance(document, dict), "navigation.json: root must be an object")

    position = document.get("position", {})
    quality = _number(position, "minimum_quality", "navigation.json position")
    _require(0.0 <= quality <= 1.0, "navigation.json: position.minimum_quality must be in [0, 1]")
    timeout = _number(position, "timeout_ms", "navigation.json position")
    _require(timeout > 0, "navigation.json: position.timeout_ms must be > 0")
    baseline = _number(position, "heading_min_displacement_m", "navigation.json position")
    _require(baseline >= 0.0, "navigation.json: position.heading_min_displacement_m must be >= 0")
    if "heading_correction_gain" in position:
        gain = _number(position, "heading_correction_gain", "navigation.json position")
        _require(0.0 < gain <= 1.0,
                 "navigation.json: position.heading_correction_gain must be in (0, 1]")

    path = document.get("path", {})
    snap = _number(path, "max_graph_snap_distance_m", "navigation.json path")
    _require(snap > 0, "navigation.json: path.max_graph_snap_distance_m must be > 0")
    replan_snap = path.get("replan_max_graph_snap_distance_m")
    if replan_snap is not None:
        _require(float(replan_snap) >= snap,
                 "navigation.json: path.replan_max_graph_snap_distance_m must be >= "
                 "max_graph_snap_distance_m")
    waypoint = _number(path, "waypoint_tolerance_m", "navigation.json path")
    destination = _number(path, "destination_tolerance_m", "navigation.json path")
    _require(waypoint > 0, "navigation.json: path.waypoint_tolerance_m must be > 0")
    _require(destination > 0, "navigation.json: path.destination_tolerance_m must be > 0")
    soft = _number(path, "max_cross_track_error_m", "navigation.json path")
    hard = _number(path, "replan_cross_track_error_m", "navigation.json path")
    _require(0 < soft <= hard,
             "navigation.json: path.max_cross_track_error_m must be in (0, replan_cross_track_error_m]")

    motion = document.get("motion", {})
    max_linear = _number(motion, "max_linear_speed_mps", "navigation.json motion")
    min_linear = _number(motion, "min_linear_speed_mps", "navigation.json motion")
    max_angular = _number(motion, "max_angular_speed_radps", "navigation.json motion")
    rotate = _number(motion, "rotate_in_place_threshold_deg", "navigation.json motion")
    _require(max_linear > 0, "navigation.json: motion.max_linear_speed_mps must be > 0")
    _require(0 <= min_linear <= max_linear,
             "navigation.json: motion.min_linear_speed_mps must be in [0, max_linear_speed_mps]")
    _require(max_angular > 0, "navigation.json: motion.max_angular_speed_radps must be > 0")
    _require(0 <= rotate <= 180,
             "navigation.json: motion.rotate_in_place_threshold_deg must be in [0, 180]")

    robot = document.get("robot", {})
    radius = _number(robot, "wheel_radius_m", "navigation.json robot")
    base = _number(robot, "wheel_base_m", "navigation.json robot")
    wheel_speed = _number(robot, "max_wheel_speed_mps", "navigation.json robot")
    _require(radius > 0, "navigation.json: robot.wheel_radius_m must be > 0")
    _require(base > 0, "navigation.json: robot.wheel_base_m must be > 0")
    _require(wheel_speed > 0, "navigation.json: robot.max_wheel_speed_mps must be > 0")
    # The chassis must realise the twists the speed policy can produce.  The naive
    # bound max_linear + 0.5*max_angular*base assumes both maxima are demanded at
    # once, which the policy forbids: it derates the forward speed as the heading
    # error grows and commands zero forward speed when rotating in place.  The two
    # corners it can actually request are full speed straight ahead and full yaw
    # rate on the spot.
    straight_wheel = max_linear
    spin_wheel = 0.5 * max_angular * base
    required_wheel_speed = max(straight_wheel, spin_wheel)
    _require(wheel_speed >= required_wheel_speed - 1e-6,
             f"navigation.json: robot.max_wheel_speed_mps ({wheel_speed}) is below the "
             f"wheel speed required by the motion limits ({required_wheel_speed:.3f} = "
             f"max(v_max, 0.5*omega_max*wheel_base))")

    control = document.get("control", {})
    nav_rate = _number(control, "navigation_rate_hz", "navigation.json control")
    telem_rate = _number(control, "telemetry_rate_hz", "navigation.json control")
    _require(0 < nav_rate <= 200, "navigation.json: control.navigation_rate_hz must be in (0, 200]")
    _require(0 < telem_rate <= 100,
             "navigation.json: control.telemetry_rate_hz must be in (0, 100]")

    if "map" in document:
        map_block = document["map"]
        if "width_m" in map_block:
            _require(abs(float(map_block["width_m"]) - map_info["width_m"]) < 1e-6,
                     "navigation.json: map.width_m disagrees with map.json")
        if "height_m" in map_block:
            _require(abs(float(map_block["height_m"]) - map_info["height_m"]) < 1e-6,
                     "navigation.json: map.height_m disagrees with map.json")

    return {"navigation_rate_hz": nav_rate, "max_linear_speed_mps": max_linear}


def validate_simulator(document: dict) -> dict:
    _require(isinstance(document, dict), "simulator.json: root must be an object")
    window = document.get("window", {})
    width = int(window.get("width_px", 1400))
    height = int(window.get("height_px", 850))
    _require(width > 400 and height > 300, "simulator.json: window is too small to be usable")
    simulation = document.get("simulation", {})
    dt = float(simulation.get("dt_s", 0.02))
    _require(0.001 <= dt <= 0.2, "simulator.json: simulation.dt_s must be in [0.001, 0.2]")
    speed = float(simulation.get("speed_multiplier", 1.0))
    _require(0.1 <= speed <= 20.0,
             "simulator.json: simulation.speed_multiplier must be in [0.1, 20]")
    uwb = document.get("uwb", {})
    rate = float(uwb.get("update_rate_hz", 10.0))
    _require(0.5 <= rate <= 100.0, "simulator.json: uwb.update_rate_hz must be in [0.5, 100]")
    noise = float(uwb.get("noise_std_m", 0.05))
    _require(0.0 <= noise <= 2.0, "simulator.json: uwb.noise_std_m must be in [0, 2]")
    dropout = float(uwb.get("dropout_probability", 0.0))
    _require(0.0 <= dropout <= 1.0, "simulator.json: uwb.dropout_probability must be in [0, 1]")
    clearance = float(document.get("obstacle_clearance_m", 0.2))
    _require(clearance >= 0.0,
             "simulator.json: obstacle_clearance_m must be >= 0")
    return {"dt_s": dt, "update_rate_hz": rate}


def load_json(path: Path) -> dict:
    try:
        with path.open(encoding="utf-8") as handle:
            return json.load(handle)
    except FileNotFoundError:
        raise ValidationError(f"missing file: {path}") from None
    except json.JSONDecodeError as exc:
        raise ValidationError(f"{path}: invalid JSON ({exc})") from None


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config-dir", default="config",
                        help="directory containing map.json, graph.json, navigation.json, "
                             "simulator.json (default: config)")
    args = parser.parse_args(argv)

    config_dir = Path(args.config_dir)
    if not config_dir.is_absolute():
        config_dir = REPO_ROOT / config_dir

    files = {
        "map": config_dir / "map.json",
        "graph": config_dir / "graph.json",
        "navigation": config_dir / "navigation.json",
        "simulator": config_dir / "simulator.json",
    }

    failures = 0
    try:
        map_info = validate_map(load_json(files["map"]))
        print(f"[ok] map.json        {map_info['map_id']} v{map_info['map_version']} "
              f"{map_info['width_m']:.1f} x {map_info['height_m']:.1f} m")
    except ValidationError as exc:
        print(f"[FAIL] {exc}")
        return 1

    try:
        graph_info = validate_graph(load_json(files["graph"]), map_info)
        print(f"[ok] graph.json      {graph_info['node_count']} nodes, "
              f"{graph_info['edge_count']} edges, destinations "
              f"{graph_info['destination_nodes']}")
    except ValidationError as exc:
        print(f"[FAIL] {exc}")
        failures += 1

    try:
        nav_info = validate_navigation(load_json(files["navigation"]), map_info)
        print(f"[ok] navigation.json {nav_info['navigation_rate_hz']:.0f} Hz control loop, "
              f"max {nav_info['max_linear_speed_mps']:.2f} m/s")
    except ValidationError as exc:
        print(f"[FAIL] {exc}")
        failures += 1

    try:
        sim_info = validate_simulator(load_json(files["simulator"]))
        print(f"[ok] simulator.json  dt {sim_info['dt_s'] * 1000:.1f} ms, "
              f"UWB {sim_info['update_rate_hz']:.0f} Hz")
    except ValidationError as exc:
        print(f"[FAIL] {exc}")
        failures += 1

    if failures:
        print(f"\n{failures} configuration file(s) invalid")
        return 1
    print("\nAll configuration files are valid.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
